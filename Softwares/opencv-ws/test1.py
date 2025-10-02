import numpy as np 
import cv2 
      


window_name = "Multiple Color Detection in Real-TIme"


# Reading the video from the 
# webcam in image frames 
# _, imageFrame = webcam.read() 
imageFrame = cv2.imread("/home/baser-ubuntu/Desktop/python-ws/opencv-ws/test123.png")

# Convert the imageFrame in  
# BGR(RGB color space) to  
# HSV(hue-saturation-value) 
# color space 
hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

# Set range for red color and  
# define mask 
# red_lower = np.array([136, 87, 111], np.uint8) 
# red_upper = np.array([180, 255, 255], np.uint8) 

red_lower = np.array([0, 170, 220], np.uint8) 
red_upper = np.array([10, 180, 240], np.uint8) 
red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

# Set range for green color and  
# define mask 
green_lower = np.array([25, 52, 72], np.uint8) 
green_upper = np.array([102, 255, 255], np.uint8) 
green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

# Set range for blue color and 
# define mask 
blue_lower = np.array([94, 80, 2], np.uint8) 
blue_upper = np.array([120, 255, 255], np.uint8) 
blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
    
# Morphological Transform, Dilation 
# for each color and bitwise_and operator 
# between imageFrame and mask determines 
# to detect only that particular color 
kernel = np.ones((5, 5), "uint8") 
    
# For red color 
red_mask = cv2.dilate(red_mask, kernel) 
res_red = cv2.bitwise_and(imageFrame, imageFrame,  
                            mask = red_mask) 
    
# For green color 
green_mask = cv2.dilate(green_mask, kernel) 
res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                            mask = green_mask) 
    
# For blue color 
blue_mask = cv2.dilate(blue_mask, kernel) 
res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
                            mask = blue_mask) 

# Creating contour to track red color 
contours, hierarchy = cv2.findContours(red_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE) 
    
for pic, contour in enumerate(contours): 
    area = cv2.contourArea(contour) 
    if(area > 300): 
        x, y, w, h = cv2.boundingRect(contour) 
        imageFrame = cv2.rectangle(imageFrame, (x, y),  
                                    (x + w, y + h),  
                                    (0, 0, 255), 2) 
            
        cv2.putText(imageFrame, "Red Colour", (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                    (0, 0, 255))     
     
            
def mouseRGB_callback(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = hsvFrame[y,x,0]
        colorsG = hsvFrame[y,x,1]
        colorsR = hsvFrame[y,x,2]
        colors = hsvFrame[y,x]
        print("Red: ",colorsR)
        print("Green: ",colorsG)
        print("Blue: ",colorsB)
        print("BRG Format: ",colors)
        print("Coordinates of pixel: X: ",x,"Y: ",y)

# Program Termination 
cv2.namedWindow(window_name)
cv2.imshow(window_name, imageFrame) 
cv2.setMouseCallback(window_name,mouseRGB_callback)

cv2.waitKey(0)

