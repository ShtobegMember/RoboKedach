import cv2
import time


def main(port):
    # Open camera with V4L2 driver
    cap = cv2.VideoCapture(port, cv2.CAP_V4L2)

    # Settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # Buffer setting (helps reduce lag/corruption on Pi)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    print("Camera active. Press 'q' to quit.")

    while True:
        try:
            # We wrap the read in a try/except block to catch the crash
            ret, frame = cap.read()

            # If the frame is empty/failed, skip this loop iteration
            if not ret or frame is None:
                print("Warning: Skipped a bad frame.")
                continue

            cv2.imshow('Pi Camera Live', frame)

            if cv2.waitKey(1) == ord('q'):
                break

        except cv2.error:
            # This catches the specific "imdecode" crash you saw
            print("Error: Corrupted frame received. Ignoring...")
            time.sleep(0.1)     # Brief pause to let the buffer clear
            continue
    cap.release()
    cv2.destroyAllWindows()


try:
    main(0)


except:
    main(1)
