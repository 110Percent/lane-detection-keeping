import numpy as np

import cv2

class CameraController:
    def canny(self, image):
        return cv2.Canny(image, 100, 200)
    
    def show_image(self, image):
        cv2.imshow("test", image)
        cv2.waitKey(1)

