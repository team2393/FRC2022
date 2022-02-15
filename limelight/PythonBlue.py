import cv2
import numpy as np
import math

# global variables go here:
kernel = np.ones((3, 3), np.uint8)

# Color threshold
hsv_min = (90, 70, 70)
hsv_max = (120, 255, 255)

# Shape filter
aspect_min = 0.5
aspect_max = 1.5
fullness_min = 60
fullness_max = 100
circularity_min = 0.6

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    # norm = cv2.normalize(image, 0.0, 255.0, cv2.NORM_MINMAX)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(hsv, hsv_min, hsv_max)
   
    # Erode standalone pixels
    threshold = cv2.erode(threshold, kernel, iterations = 1)

    # Dilate holes within solid areas
    threshold = cv2.dilate(threshold, kernel, iterations = 1)

    # Find contours and filter them
    contours, _ = cv2.findContours(threshold, 
                     cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    bestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    best = -1
    largest_area = 0
    for i in range(len(contours)):
        contour = contours[i]

        # Show every contour in cyan
        cv2.drawContours(image, contours, i, (255, 255, 0), 1)
        area = cv2.contourArea(contour)
        if area < largest_area:
            continue

        # Filter on aspect ratio 0 (tall) .. 1 (square) .. 20 (wide)
        x,y,w,h = cv2.boundingRect(contour)
        aspect = w / h
        if aspect < aspect_min  or  aspect > aspect_max:
            continue

        # Filter on fullness (percent): 0% (hollow) .. 100% (solid, full)
        fullness = 100.0 * area / (w * h)
        if fullness < fullness_min  or  fullness > fullness_max:
            continue
        
        perimeter = cv2.arcLength(contour, True)
        # Circularity = 4*Math.PI*area / perimeter^2
        # Circle:
        #      4*pi*(pi*r*r)/(2*pi*r)^2 =
        #      4*pi*pi*r*r/(4*pi*pi*r*r) = 1
        #
        # Square:
        #      4*pi*d*d/(4*d)^2 =
        #      4*pi*d*d/(16*d*d) = pi/4 = 0.78
        circularity = 4*math.pi*area / (perimeter*perimeter)
        if circularity < circularity_min:
            continue;

        # Show contour that made it this far
        cv2.drawContours(image, contours, i, (255, 0, 255), 1)

        largest_area = area
        best = i
    
    if best >= 0:
        bestContour = contours[best]
        x,y,w,h = cv2.boundingRect(bestContour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        llpython = [1,x,y,w,h,9,8,7]  
  
    cv2.putText(image, 
        'Blue2?', 
        (0, 230), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .5, (0, 255, 0), 1, cv2.LINE_AA)
       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return bestContour, image, llpython