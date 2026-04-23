import os
import shutil
import subprocess
import time


def main_build():
    # Run PyInstaller
    command = [
        "pyinstaller", 
        "--name", "robokedach", 
        "--onefile", 
        "--noconsole", 
        "--icon=robokedach_icon.ico", 
        "--add-data", "robokedach_icon.ico;.", 
        "PRODUCT/pc_main.py"
    ]

    print("Building executable...")
    subprocess.run(command)

    print("Cleaning up temporary files...")

    # Pause to ensure PyInstaller releases all file handles
    time.sleep(2)

    # ---------------------------------------------------------
    # The Nuclear Option: Native Windows Force Delete
    # ---------------------------------------------------------
    def force_delete_dir(dir_name):
        if os.path.exists(dir_name):
            try:
                # cmd /c tells Windows to run the command and terminate
                # rmdir /s /q means "Remove Directory, Sub-directories, Quietly"
                subprocess.run(['cmd', '/c', 'rmdir', '/s', '/q', dir_name], shell=True)
            except Exception as e:
                print(f"Warning: Could not delete {dir_name}. Error: {e}")

    # 1. Force delete the build directory
    force_delete_dir("build")

    # 2. Remove the spec file 
    if os.path.exists("robokedach.spec"):
        try:
            os.remove("robokedach.spec")
        except Exception:
            pass

    # 3. Move the executable and force delete the dist directory
    if os.path.exists("dist/robokedach.exe"):
        # Clear out any old versions first
        if os.path.exists("robokedach.exe"):
            try:
                os.remove("robokedach.exe")
            except Exception:
                pass
                
        # Pull the new exe out
        shutil.move("dist/robokedach.exe", "robokedach.exe")
        
        # Force delete the dist directory
        force_delete_dir("dist")

    print("Build complete! Only robokedach.exe remains.")


if __name__ == "__main__":
    main_build()
