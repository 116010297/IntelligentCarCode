import time
import picamera
with picamera.PiCamera() as camera:
    camera.start_preview()
    try:
        for i, filename in enumerate(
                camera.capture_continuous('image{counter:02d}.jpg')):
            print(filename)
            if i == 50:
                break
    finally:
        camera.stop_preview()