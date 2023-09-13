import os
import pathlib
import numpy as np

import cv2

VIDEO_PATH = str(
    pathlib.Path(os.path.join("..", "..", "assets", "dashcam.webm")).resolve()
)
CANNY_THRESHOLD = 200

cap = cv2.VideoCapture(VIDEO_PATH)

while cap.isOpened():
    ret, frame = cap.read()
    cropped = frame[600:-50, :]
    img_hls = cv2.cvtColor(cropped, cv2.COLOR_BGR2HLS)[:, :, 1].astype(np.float32)
    pts1 = np.float32([[850, 0], [1070, 0], [0, 420], [1920, 420]])
    pts2 = np.float32([[0, 0], [1920, 0], [0, 420], [1920, 420]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    perspectived = cv2.warpPerspective(img_hls, M, (1920, 420))
    edge_x = cv2.Scharr(perspectived, cv2.CV_64F, 1, 0)
    edge_x = np.absolute(edge_x)
    edge_x = np.uint8(255 * edge_x / np.max(edge_x))
    binary = np.zeros_like(edge_x)

    threshold_up = 15
    threshold_down = 60

    threshold_delta = threshold_down - threshold_up

    for y in range(420):
        binary_line = binary[y, :]
        edge_line = edge_x[y, :]
        threshold_line = threshold_down * y / 420
        binary_line[edge_line >= threshold_line] = 255
        binary[y, :] = binary_line

    cv2.imshow("frame", binary)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
