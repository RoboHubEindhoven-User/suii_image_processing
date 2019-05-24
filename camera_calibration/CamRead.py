import numpy as np
import cv2

cap = cv2.VideoCapture(0)
img_counter = 0
 
while(True):
    ret, frame = cap.read()
 
    k = cv2.waitKey(1)
    cv2.imshow('Webcam', frame)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

    elif k%256 == 32:
        # SPACE pressed
        img_name = "calibration_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

frame.release()

cv2.destroyAllWindows()