import cv2
import numpy as np

from cscore import CameraServer as CS

def main():

    # CS.enableLogging()
    camera = CS.startAutomaticCapture()
    camera.setResolution(640, 480)

    cvSink = CS.getVideo()
    outputStream = CS.putVideo("Rectangle", 640, 480)

    mat = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    while True:
        time, mat = cvSink.grabFrame(mat)
        if time == 0:
            outputStream.notifyError(cvSink.getError())
            continue

        # cv2.rectangle(mat, (100, 100), (400, 400), (255, 255, 255), 5)

        outputStream.putFrame(mat)