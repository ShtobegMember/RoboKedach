"""
rpi_main.py - Raspberry Pi main process manager.
Launches camera server, IMU & voltage/current streamer, and motor engine as separate processes.
Monitors health and auto-restarts on failure.
"""

import multiprocessing
import threading
import queue
import socket
import struct
import select
import time
import sys
import smbus2

from camera_server import run_server
from imu_driver import LSM6DSV16X
import robot_controller
from robot_controller import RobotConfig, MotorController, MovementController, Direction


# ========================== Configuration ==========================
PC_IP = "192.168.1.1"

IMU_PORT = 65432
MOTOR_PORT = 65433
VM_PORT = 65434

# INA226 (Voltage Monitor) on I2C bus 3
INA226_BUS        = 3
INA226_ADDR       = 0x40
INA226_SHUNT_OHMS = 0.01   # R010 = 10 mOhm
INA226_REG_CONFIG = 0x00
INA226_REG_BUS_V  = 0x02
INA226_REG_CURRENT = 0x04
INA226_REG_CAL    = 0x05
INA226_BUS_V_LSB  = 1.25e-3   # 1.25 mV/bit
INA226_CURRENT_LSB = 0.00025  # 0.25 mA/bit


# ========================== INA226 Helpers ==========================
def ina226_read_signed(bus, reg):
    """Atomic 16-bit signed read from INA226."""

    data = bus.read_i2c_block_data(INA226_ADDR, reg, 2)
    return struct.unpack(">h", bytes(data))[0]


def ina226_write(bus, reg, value):
    """Write 16-bit big-endian value to INA226 register."""

    data = struct.pack(">H", value & 0xFFFF)
    bus.write_i2c_block_data(INA226_ADDR, reg, list(data))


def ina226_init(bus):
    """Configure INA226: 16-sample averaging, 1.1ms conversion, continuous mode."""

    config = (0b010 << 12) | (0b100 << 9) | (0b100 << 6) | 0b111
    ina226_write(bus, INA226_REG_CONFIG, config)
    cal = int(0.00512 / (INA226_CURRENT_LSB * INA226_SHUNT_OHMS))
    ina226_write(bus, INA226_REG_CAL, cal)


# ========================== VM Streamer Thread ==========================
def vm_streamer(server_ip, port):
    """Stream INA226 voltage/current to the PC over TCP. Runs as a daemon thread."""

    try:
        bus = smbus2.SMBus(INA226_BUS)
        ina226_init(bus)
        print("VM: INA226 initialized on I2C bus 3.")
    except Exception as e:
        print(f"VM: Failed to init INA226: {e}. Thread exiting.")
        return

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print(f"VM: Connecting to PC at {server_ip}:{port}...")
                s.connect((server_ip, port))
                print("VM: Connected. Streaming power data.")

                while True:
                    try:
                        raw_v = ina226_read_signed(bus, INA226_REG_BUS_V)
                        voltage = raw_v * INA226_BUS_V_LSB
                        raw_i = ina226_read_signed(bus, INA226_REG_CURRENT)
                        current = raw_i * INA226_CURRENT_LSB
                        s.sendall(struct.pack('<2f', voltage, current))
                    except OSError as e:
                        print(f"VM: I2C read error: {e}")
                    time.sleep(0.5)

        except ConnectionRefusedError:
            print("VM: PC not ready. Retrying in 3s...")
            time.sleep(3)
        except (ConnectionResetError, BrokenPipeError):
            print("VM: Connection lost. Reconnecting in 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"VM: Error: {e}. Retrying in 3s...")
            time.sleep(3)


# ========================== Camera Process ==========================
def camera_process():
    """Run the MJPEG camera server (Flask on port 5000)."""

    run_server()


# ========================== IMU Process (+ VM Thread) ==========================
def imu_vm_process(server_ip, imu_port, vm_port):
    """Stream IMU data to the PC over TCP. Spawns VM streamer as a daemon thread."""

    # Launch INA226 streamer inside this process
    vm_thread = threading.Thread(target=vm_streamer, args=(server_ip, vm_port), daemon=True)
    vm_thread.start()

    sensor = LSM6DSV16X()
    sensor.initialize()

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print(f"IMU: Connecting to PC at {server_ip}:{imu_port}...")
                s.connect((server_ip, imu_port))
                print("IMU: Connected. Waiting for commands.")

                is_streaming = False

                while True:
                    ready, _, _ = select.select([s], [], [], 0.01)

                    if ready:
                        data = s.recv(1024)
                        if not data:
                            print("IMU: Connection closed by server.")
                            break

                        command = data.decode('utf-8').strip()
                        if command == "START":
                            is_streaming = True
                            print("IMU: Streaming started.")
                        elif command == "STOP":
                            is_streaming = False
                            print("IMU: Streaming paused.")

                    if is_streaming:
                        imu_data = sensor.get_data()
                        if imu_data is not None:
                            payload = struct.pack(
                                '<6f',
                                imu_data['ax'], imu_data['ay'], imu_data['az'],
                                imu_data['gx'], imu_data['gy'], imu_data['gz']
                            )
                            s.sendall(payload)

                        time.sleep(0.005)

        except ConnectionRefusedError:
            print("IMU: PC not ready. Retrying in 3s...")
            time.sleep(3)
        except (ConnectionResetError, BrokenPipeError):
            print("IMU: Connection lost. Reconnecting in 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"IMU: Error: {e}. Retrying in 3s...")
            time.sleep(3)


# ========================== Motor Process ==========================
def motor_process(port):
    """
    Motor engine: TCP server receiving movement commands from the PC.
    Monkey-patches get_key so drive_distance() abort works over the network
    instead of reading from stdin.
    """

    # Abort mechanism — replaces terminal-based get_key()
    abort_event = threading.Event()

    def network_get_key(timeout=None):
        """Drop-in replacement for robot_controller.get_key."""

        if abort_event.is_set():
            abort_event.clear()
            return ' '  # Spacebar = abort in _monitor_movement
        if timeout:
            time.sleep(timeout)
        return None

    robot_controller.get_key = network_get_key

    # Initialize motor hardware with retry
    motor_ctrl = None
    move_ctrl = None
    while motor_ctrl is None:
        try:
            config = RobotConfig()
            motor_ctrl = MotorController(config)
            move_ctrl = MovementController(motor_ctrl)
            print("MOTOR: Hardware initialized.")
        except ConnectionError as e:
            print(f"MOTOR: {e}. Retrying in 3s...")
            time.sleep(3)

    cmd_queue = queue.Queue()

    def socket_reader(conn):
        """Background thread: reads commands from PC, routes ABORT to event."""
        
        try:
            buf = ""
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                buf += data.decode('utf-8')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    cmd = line.strip()
                    if cmd == "ABORT":
                        abort_event.set()
                    elif cmd:
                        cmd_queue.put(cmd)
        except Exception:
            pass

    # TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen()
    server.settimeout(1.0)
    print(f"MOTOR: Listening on port {port}")

    while True:
        try:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue

            print(f"MOTOR: PC connected from {addr}")

            reader = threading.Thread(target=socket_reader, args=(conn,), daemon=True)
            reader.start()

            try:
                conn.sendall(f"SPEED:{motor_ctrl.current_speed}\nREADY\n".encode())
            except Exception:
                continue

            try:
                while reader.is_alive():
                    try:
                        cmd = cmd_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    d = Direction
                    try:
                        moved = False

                        if cmd == "FWD":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.FORWARD, d.FORWARD)
                            moved = True
                        elif cmd == "BWD":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.BACKWARD, d.BACKWARD)
                            moved = True
                        elif cmd == "LEFT":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.FORWARD, d.STOP)
                            moved = True
                        elif cmd == "RIGHT":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.STOP, d.FORWARD)
                            moved = True
                        elif cmd == "FWD90":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.FORWARD, d.FORWARD, 0.25)
                            moved = True
                        elif cmd == "BWD90":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.BACKWARD, d.BACKWARD, 0.25)
                            moved = True
                        elif cmd == "LEFT90":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.FORWARD, d.STOP, 0.25)
                            moved = True
                        elif cmd == "RIGHT90":
                            conn.sendall(b"BUSY\n")
                            move_ctrl.drive_distance(d.STOP, d.FORWARD, 0.25)
                            moved = True
                        elif cmd == "SPEED_UP":
                            motor_ctrl.adjust_speed(config.speed_increment)
                        elif cmd == "SPEED_DOWN":
                            motor_ctrl.adjust_speed(-config.speed_increment)
                        elif cmd == "RESET_ENC":
                            motor_ctrl.reset_encoders()

                        # Drain stale movement commands after a blocking move
                        if moved:
                            while not cmd_queue.empty():
                                try:
                                    cmd_queue.get_nowait()
                                except queue.Empty:
                                    break

                        conn.sendall(f"SPEED:{motor_ctrl.current_speed}\nREADY\n".encode())

                    except IOError as e:
                        print(f"MOTOR: Movement error: {e}")
                        try:
                            conn.sendall(f"ERROR:{e}\nREADY\n".encode())
                        except Exception:
                            break

            except (ConnectionResetError, BrokenPipeError, OSError):
                pass

            motor_ctrl.stop_all()
            abort_event.clear()
            # Drain leftover commands
            while not cmd_queue.empty():
                try:
                    cmd_queue.get_nowait()
                except queue.Empty:
                    break
            print("MOTOR: PC disconnected. Waiting for reconnection...")

        except Exception as e:
            print(f"MOTOR: Error: {e}")
            time.sleep(1)


# ========================== Main ==========================
def main():
    print("=" * 50)
    print("  RoboKedach - Raspberry Pi Main Controller")
    print("=" * 50)

    procs = {}

    def start_process(name, target, args=()):
        p = multiprocessing.Process(target=target, args=args, name=name)
        p.daemon = True
        p.start()
        procs[name] = (p, target, args)
        print(f"  [{name}] started (PID {p.pid})")
        return p

    start_process("CameraServer", camera_process)
    start_process("IMUStreamer", imu_vm_process, (PC_IP, IMU_PORT, VM_PORT))
    start_process("MotorEngine", motor_process, (MOTOR_PORT,))

    print("\nAll processes running. Press Ctrl+C to stop.\n")

    try:
        while True:
            for name, (proc, target, args) in list(procs.items()):
                if not proc.is_alive():
                    print(f"WARNING: {name} died (exit code {proc.exitcode}). Restarting...")
                    start_process(name, target, args)
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nShutting down...")
        for name, (proc, _, _) in procs.items():
            print(f"  Stopping {name}...")
            proc.terminate()
        for _, (proc, _, _) in procs.items():
            proc.join(timeout=3)
        print("All processes stopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
