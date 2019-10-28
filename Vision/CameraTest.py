import numpy as np
import cv2 as cv

# Camera setup
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_BRIGHTNESS, 0.5)
font = cv.FONT_HERSHEY_SIMPLEX


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    #Get frame center
    cx = 320
    cy = 240

    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #thresh = cv.inRange(frame_hsv, (110, 80, 100), (200, 250, 250))
    thresh = cv.inRange(frame_hsv, (30, 30, 120), (90, 120, 210))
    image, contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # Filter out contours with fewer than 4 points
    cnts = []
    mnts = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > 150:
            # print(area)
            cnts.append(cnt)
            M = cv.moments(cnt)
            mx = int(M['m10']/M['m00'])
            my = int(M['m01']/M['m00'])
            mnts.append([mx,my])

    frame = cv.drawContours(frame, cnts, -1, (255, 255, 0), 3)

    for mnt in mnts:
        mx = mnt[0]
        my = mnt[1]
        frame = cv.circle(frame, (mx, my), 3, (0, 0, 255), -1, cv.LINE_AA)
        frame = cv.putText(frame, 'Tracked Point', (mx - 130, my- 90), font, 1.0, (0, 255, 0), 2, cv.LINE_AA)

    frame = cv.circle(frame, (cx, cy), 2, (0, 255, 0))

    print(frame_hsv[cy, cx])
    
    # Display the resulting frame
    cv.imshow('frame', frame)
    cv.imshow('Threshold', thresh)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
