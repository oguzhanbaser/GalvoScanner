import cv2
import numpy as np

def sort_contours(cnts, method="left-to-right"):
	# initialize the reverse flag and sort index
	reverse = False
	i = 0
	# handle if we need to sort in reverse
	if method == "right-to-left" or method == "bottom-to-top":
		reverse = True
	# handle if we are sorting against the y-coordinate rather than
	# the x-coordinate of the bounding box
	if method == "top-to-bottom" or method == "bottom-to-top":
		i = 1
	# construct the list of bounding boxes and sort them from top to
	# bottom
	boundingBoxes = [cv2.boundingRect(c) for c in cnts]
	
	(cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
		key=lambda b:b[1][i], reverse=reverse))
	# return the list of sorted contours and bounding boxes
	
	return (cnts, boundingBoxes)

def sort_contours_clockwise(contours):
    # Find the center of the contours
    centers = []
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX, cY))
        else:
            centers.append((0, 0))

    # Calculate the center of the entire set
    center_point = np.mean(centers, axis=0)

    # Sort contours based on the angle
    def angle_from_center(point):
        return np.arctan2(point[1] - center_point[1], point[0] - center_point[0])

    sorted_contours = sorted(contours, key=lambda contour: angle_from_center(cv2.boundingRect(contour)[:2]))

    return sorted_contours

def calculate_angle(p1, p2, p3):
    # Convert points to vectors
    v1 = np.array(p1) - np.array(p2)
    v2 = np.array(p3) - np.array(p2)
    
    # Calculate the angle between the vectors
    angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    angle = np.degrees(angle)
    
    # Ensure angle is in the range [0, 360]
    if angle < 0:
        angle += 360
    
    return angle

def calculate_distance(p1, p2):
    # Calculate the Euclidean distance
    distance = np.linalg.norm(np.array(p1) - np.array(p2))
    return distance

def calculate_f(pPix, pDist, pLen):
     return (pPix * pDist) / pLen

def calculate_real_distance(pPix, pF, pLen):
     return (pF * pLen) / pPix