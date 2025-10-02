import numpy as np 
import cv2 
      
from my_utils import *
from itertools import combinations
import math
from camera_params import *

window_name = "Multiple Color Detection in Real-TIme"
orig_window = "Original Window"

raspiCam = 1
pcCam = 2

cameraType = pcCam
_size = (800, 600)

if cameraType == raspiCam:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    camera = PiCamera()
    camera.resolution = _size
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size= _size)
elif cameraType == pcCam:
    cap = cv2.VideoCapture(0)


# Reading the video from the 
# webcam in image frames 
# _, imageFrame = webcam.read() 
# imageFrame = cv2.imread("images\\20240711_111126.jpg")
# imageFrame = cv2.imread("images\\20240711_111208.jpg")
# imageFrame = cv2.imread("images\\20240801_101249.jpg")
# imageFrame = cv2.imread("images\\20240801_101222.jpg")
# imageFrame = cv2.imread("images\\20240801_100951.jpg")

h, w = _size
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# origFrame = imageFrame

hLow = 0
hMax = 10
sLow = 0
sMax = 11
vLow = 245
vMax = 255

def readCamera():
    if cameraType == pcCam:
        _ , frame = cap.read()
        frame = cv2.resize(frame, _size)
    elif cameraType == raspiCam:
        camera.capture(rawCapture, format="bgr")
        frame = rawCapture.array

    # for camera undistortion
    # frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    # x, y, w, h = roi
    # frame = frame[y:y+h, x:x+w]

    frame = cv2.GaussianBlur(frame, (13,13 ), 0)
    return frame

def findSquares(sFrame):
    pFrame = sFrame.copy()
    # Convert the imageFrame in  
    # BGR(RGB color space) to  
    # HSV(hue-saturation-value) 
    # color space 
    hsvFrame = cv2.cvtColor(pFrame, cv2.COLOR_BGR2HSV) 
    imageFrame = sFrame.copy()

    # Set range for red color and  
    # define mask 
    # red_lower = np.array([136, 87, 111], np.uint8) 
    # red_upper = np.array([180, 255, 255], np.uint8) 

    # red_lower = np.array([0, 170, 220], np.uint8) 
    # red_upper = np.array([10, 180, 240], np.uint8) 
    # red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

    red_lower = np.array([hLow, sLow, vLow], np.uint8) 
    red_upper = np.array([hMax, sMax, vMax], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
        
    # Morphological Transform, Dilation 
    # for each color and bitwise_and operator 
    # between imageFrame and mask determines 
    # to detect only that particular color 
    kernel = np.ones((5, 5), "uint8") 
        
    # For red color 
    red_mask = cv2.dilate(red_mask, kernel) 
    res_red = cv2.bitwise_and(pFrame, pFrame,  
                                mask = red_mask)  

    # Creating contour to track red color 
    contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
    cent_arr = []
    
    # contours, _ = sort_contours(contours)
    contours = sort_contours_clockwise(contours)

    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 50): 
            x, y, w, h = cv2.boundingRect(contour) 
            cent = (int(x + w / 2), int(y + h / 2))
            cent_arr.append(cent)
            imageFrame = cv2.rectangle(pFrame, (x, y),  
                                        (x + w, y + h),  
                                        (0, 0, 255), 2) 
                
            # cv2.putText(imageFrame, "Red Colour", (x, y), 
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
            #             (0, 0, 255))     
        

    num_points = len(cent_arr)
    angles = []

    summ = []

    if num_points != 0:
        for p in cent_arr:

            pp = [x for x in cent_arr if x != p]


            for p2, p3 in combinations(pp, 2):
                # combo = (p,) + combo
                angle = calculate_angle(p2, p, p3)
                angles.append(((p2, p, p3), angle))

                if((angle > 85.0 and angle < 95.0) or (angle > 265 and angle < 285)):
                    d1 = calculate_distance(p2, p)
                    d2 = calculate_distance(p, p3)

                    # print(d1, d2)

                    if (math.isclose(d1, d2, abs_tol=10)):
                        # print("heree")
                        # print(d1)
                        summ.append(d1)
                        cv2.line(imageFrame, p, p2, (0, 255, 0), 5)
                        cv2.line(imageFrame, p3, p, (0, 255, 0), 5)

        pix = np.mean(summ)        
        # print(pix)

        # print(calculate_f(pix, 55.5, 3))
        # print("Real dist: ", calculate_real_distance(pix, 615.823, 3))

    return hsvFrame, imageFrame

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def mouseRGB_callback(event,x,y,flags,param):
    global hMax, hLow, sMax, sLow, vMax, vLow

    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        hVal = hFrame[y,x,0]
        sVal = hFrame[y,x,1]
        gVal = hFrame[y,x,2]
        
        hMax = hVal + 10
        hLow = hVal - 10
        sMax = sVal + 10
        sLow = sVal - 10
        vMax = gVal + 10
        vLow = gVal - 10

        hMax = clamp(hMax, 0, 255)
        hLow = clamp(hLow, 0, 255)
        sMax = clamp(sMax, 0, 255)
        sLow = clamp(sLow, 0, 255)
        vMax = clamp(vMax, 0, 255)
        vLow = clamp(vLow, 0, 255)

        print(hMax, hLow, sMax, sLow, vMax, vLow)

        print("X: {0}, Y: {1}, H: {2}, S: {3}, V: {4}".format(x, y, hVal, sVal, gVal))



# Program Termination 
cv2.namedWindow(window_name)
cv2.namedWindow(orig_window)
cv2.setMouseCallback(orig_window, mouseRGB_callback)

while True:

    origFrame = readCamera()

    hFrame, iFrame = findSquares(origFrame)

    cv2.imshow(orig_window, origFrame) 
    cv2.imshow(window_name, iFrame) 
    k = cv2.waitKey(10)

    if (k == 27): break

