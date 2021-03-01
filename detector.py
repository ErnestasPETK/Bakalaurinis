import cv2
import numpy as np
import imutils
from kalmanfilter import KalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.common import Q_discrete_white_noise



def fx (x,dt):

    #function that returns the state x transformed by the state transition function. dt is the time step in seconds.


    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=float)

    return np.dot(F,x)

def hx(x):

    #Measurement function. Converts state vector x into a measurement vector of shape (dim_z).

    return np.array(x[0], x[2])



def click(event, x, y, flags, param):
    global targeting
    global targeting_x
    global targeting_y

    targeting_x = 0
    targeting_y = 0

    if event == cv2.EVENT_LBUTTONDOWN:
        targeting = True
        targeting_x = x
        targeting_y = y
        print ( "button down" )

    elif event == cv2.EVENT_LBUTTONUP:
        targeting = False

        print ( "button up" )


    if targeting is True:
        print("targeting is active")

targeting = False

cv2.setMouseCallback("Rocket-Tracking", click)

def detect (frame,debugMode):

    scale = 0.5


    peri_min = 10
    approx_min = 2

    #target coordinates
    target_x = 0
    target_y = 0

    #search zone
    sz = 1000

    #search scope, mouse click is a center of such search scope
    d_sq = 500

    indexnr = 0
    index = 0

    # object size limitations
    wh_size = 350

    is_dinamic1_target_size = False
    is_dinamic2_target_size = True
    target_size = 100

    use_kalman_filter = False

    KF = KalmanFilter(1, 1, 1, 2, 1, 1)


    #convert frame from BGR to gray
    #frame = cv2.resize(frame, (0, 0), None, scale, scale)
    rgbed = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

    gray = cv2.cvtColor(rgbed,cv2.COLOR_BGR2GRAY)
    #gray = cv2.resize(gray, (0, 0), None, scale, scale)
    #Edge detection using Canny method
    blurred = cv2.GaussianBlur(gray,(3, 3), 0)
    #median = cv2.medianBlur(blurred, 9)
    img_edges=cv2.Canny(blurred, 50, 150, 1)


    #img_edges = cv2.resize(img_edges, (0, 0), None, scale, scale)
    #Convert to black and white image
    ret,img_thresh = cv2.threshold(img_edges, 245, 255, cv2.THRESH_BINARY)
    #kernel = np.ones((1,1), np.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (1, 1))
     # array([[0, 0, 1, 0, 0],
     #       [0, 0, 1, 0, 0],
     #       [1, 1, 1, 1, 1],
     #       [0, 0, 1, 0, 0],
     #       [0, 0, 1, 0, 0]], dtype=uint8)

    #dilated = cv2.dilate(img_thresh,kernel , iterations = 1)
    #eroded = cv2.erode(dilated, kernel, iterations=2)
    opening = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    #find contours
    #contours, _ = cv2.findContours(opening,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours= cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = imutils.grab_contours(contours)
    #print(contours)
    # set the accepted minimum & maximum area of a detected object
    min_area_thresh = 35
    max_area_thresh = 300
    min_peri_thresh = 40
    max_peri_thresh = 300
    min_radius_thresh = 30
    max_radius_thresh = 100


    centers= []
    indexlist = []


    # sort contours to see if they match our parameters and if they do, add them to the centers list
    for c in contours:
        #contour area
        area = cv2.contourArea(c)
        area = float (area)

        peri = cv2.arcLength(c, True)
        # print ( peri, "peri")

        # epsilon is maximum distance from contour to approximated contour

        epsilon = 0.01 * cv2.arcLength(c, True)

        # array of contours

        approx = cv2.approxPolyDP(c, epsilon, True)
        # print( " approx ", approx)
        # contour bounding contour


        (x, y, w, h) = cv2.boundingRect(approx)

        # contour aspect ratio
        aspectRatio = w / float(h)

        #contour enclosing circle
        (x_circle, y_circle), radius = cv2.minEnclosingCircle(approx)







        if (area > min_area_thresh) and (area < max_area_thresh) and (peri > min_peri_thresh) and (peri < max_peri_thresh) and (radius>min_radius_thresh) and (radius< max_radius_thresh):


            #M = cv2.moments(c)
            #cx = float(M['m10'] / M['m00'])
            #cy = float(M['m01'] / M['m00'])

            indexlist.append(indexnr)
            indexnr += 1
            #centers.append(np.array([[cx], [cy]]))
            centers.append(np.array([[x_circle],[y_circle]]))

    #img_thresh = cv2.resize(img_thresh, (0, 0), None, scale, scale)
    #img_edges = cv2.resize(img_edges,(0,0), None, scale, scale)
    #cv2.imshow('edges', img_edges)
    opening = cv2.resize(opening, (0, 0), None, scale, scale)
    #eroded= cv2.resize(eroded, (0,0), None, scale, scale)
    cv2.imshow('opening',opening)

    centers = sorted(centers, key = lambda x: x[1])
#cv2.imshow('eroded', eroded)
    return centers, indexlist





    #cv2.destroyAllWindows()


