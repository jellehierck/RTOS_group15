import cv2
from matplotlib import pyplot as plt
import numpy as np
#import requests

#url = "http://192.168.178.213:8080/shot.jpg"

while True:

    #img_resp = requests.get(url)
    #img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    imageFrame = cv2.imdecode(img_arr, -1) #picamera

    # Opening image
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    green_lower = np.array([40, 70, 70], np.uint8)
    green_upper = np.array([70, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    kernal = np.ones((5, 5), "uint8")

    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)

    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 2500:
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            position = x + (0.5 * w)
            cv2.putText(imageFrame, "Green Position", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            print(position)

    cv2.imshow("Detecting color green", hsvFrame)

    if cv2.waitKey(1) == 27:
        break






