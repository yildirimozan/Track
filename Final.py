import cv2
import numpy as np
import serial

ser = serial.Serial('COM4', baudrate = 9600, timeout=1,)
cap = cv2.VideoCapture('Video.mp4')
font = cv2.FONT_HERSHEY_SIMPLEX
while (1):

    _, frame = cap.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([18, 94, 140])
    upper_yellow = np.array([48, 255, 255])

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(gray_image, 200, 255)
    mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)

    height = frame.shape[0]
    width = frame.shape[1]
    polygons = np.array([(0, height), (400, 500), ((width - 520), 500), (width, height)])
    msk = np.zeros_like(mask_yw_image)
    cv2.fillPoly(msk, [polygons], 255)
    masked = cv2.bitwise_and(mask_yw_image, msk)
    edges = cv2.Canny(masked, 100, 200)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap = 70)
    l_angle = 0
    r_angle = 0
   
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
            if(x1 != x2):
                m = (y2 - y1) / (x2 - x1)
                if m < 0: #Direction of the line is right
                    l_angle = (np.arctan(m) * 180 / np.pi) + 90
                else :
                    r_angle = (np.arctan(m) * 180 / np.pi) + 90
                if ((l_angle != 0) and (r_angle != 0)) :
                    tot_angle = l_angle + r_angle
                    print(tot_angle)
                    if 168<tot_angle<179:
                        cv2.putText(frame, "Yon : Duz ", (200, 150), font, 1, (0, 0, 0))
                        cv2.putText(frame, "Direksiyon acisi ~ 90 ", (200, 100), font, 1, (0, 0, 0))
                        ser.write('90\n'.encode())
                    elif 184<tot_angle:
                        cv2.putText(frame, "Yon : Sag ", (200, 200), font, 1, (0, 0, 0))
                        if 184<tot_angle<190:
                            cv2.putText(frame, ("Direksiyon acisi ~ 95 "), (200, 100), font, 1, (0, 0, 0))
                            ser.write('95\n'.encode())
                        elif 192<tot_angle<200:
                            cv2.putText(frame, ("Direksiyon acisi ~ 100"), (200, 100), font, 1, (0, 0, 0))
                            ser.write('100\n'.encode())
                    elif tot_angle <162 :
                        cv2.putText(frame, "Yon : Sol ", (200, 250), font, 1, (0, 0, 0))
                        if 162<tot_angle<152:
                            cv2.putText(frame, ("Direksiyon acisi ~ 85"), (200, 100), font, 1, (0, 0, 0))
                            ser.write('85\n'.encode())
                        elif 150<tot_angle:
                            cv2.putText(frame, ("Direksiyon acisi ~ 80"), (200, 100), font, 1, (0, 0, 0))
                            ser.write('80\n'.encode())
    cv2.imshow('frame', frame)
    cv2.imshow('masked', masked)

    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

cap.release()
