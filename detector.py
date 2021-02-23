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
    if contours is not None:
        #print( " contour is not none ")
        #sorting each contour and extracting information for further processing
        for cnts in contours:
            #print (cnts , "cnts")
            #perimeter
            peri = cv2.arcLength(cnts, True)
            #print ( peri, "peri")
            #epsilon is maximum distance from contour to approximated contour

            epsilon = 0.01*cv2.arcLength(cnts, True)

            #array of contours

            approx = cv2.approxPolyDP(cnts,epsilon,True)
            #print( " approx ", approx)
            #contour bounding contour

            (x,y,w,h) = cv2.boundingRect(approx)

            #contour aspect ratio
            aspectRatio = w / float(h)

            #contour area
            area = cv2.contourArea(cnts)
            hullArea = cv2.contourArea(cv2.convexHull(cnts))

            if hullArea > 0:
                solidity = area / float(hullArea)
            else:
                solidity = 0

            t_ob_arr = []

            #











            #

            for t_ob in t_ob_arr:

                print ("t_ob", t_ob)
                # giving the perimeter some wiggle room, since moving targets can often change their parameters from the viewers perspective
                peri_q = peri > peri_min and peri >= t_ob[0] - 10 and peri <= t_ob[0] + 50

                print(peri_q," peri_q ")

                # setting the minimal amount of edges in a contour
                approx_q = len(approx) > approx_min and len(approx) >= t_ob[1] - 10 and len(approx) <= t_ob[1] + 50

                print (approx_q," approx_q ")

                # setting the coefficient of the length and width of the contour
                aspectratio_q = aspectRatio >= t_ob[2] - 0.1 and aspectRatio <= t_ob[2] + 0.1

                print(aspectratio_q, " aspectratio_q ")

                area_q = area >= t_ob[3] - 10 and area <= t_ob[3] + 50

                print(area_q, " area_q ")

                hullarea_q = hullArea >= t_ob[4] - 10 and hullArea <= t_ob[4] + 50

                print(hullarea_q , " hullarea_q ")

                solidity_q = solidity >= t_ob[5] - 0.5

                print (solidity_q, " solidity_q ")

                if is_dinamic1_target_size:
                    x_q = x >= target_x - w and x <= target_x + w
                    y_q = y >= target_y - h and y <= target_y + h
                if is_dinamic2_target_size:
                    x_q = x >= target_x - hullArea / w and x <= target_x + hullArea / w
                    y_q = y >= target_y - hullArea / h and y <= target_y + hullArea / h
                if is_dinamic1_target_size is False and is_dinamic2_target_size is False:
                    x_q = x >= target_x - target_size and x <= target_x + target_size
                    y_q = y >= target_y - target_size and y <= target_y + target_size

                # contour parameters, one of the parameters must fit the requirements
                params = (peri_q or approx_q or aspectratio_q or area_q or hullarea_q or solidity_q)

                # object search zone parameters
                search_zone_xy = x >= target_x - sz and x <= target_x + sz and y >= target_y - sz and y <= target_y + sz

                # object size limitations
                wh_zone = w < wh_size and h < wh_size

                # if all the conditions are being fulfilled
                if params and (x_q and y_q) and search_zone_xy and wh_zone:

                    # display the object contour by drawing a rectangle
                    cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(frame, "Raketa", (int(x + 50), int(y + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                [50, 200, 250])

                    t_ob_arr[index][0] = peri
                    t_ob_arr[index][1] = len(approx)
                    t_ob_arr[index][2] = aspectRatio
                    t_ob_arr[index][3] = area
                    t_ob_arr[index][4] = hullArea
                    t_ob_arr[index][5] = solidity
                    t_ob_arr[index][6] = x
                    t_ob_arr[index][7] = y
                    index += 1

                    if index > 0:

                        x_midle = 0
                        y_midle = 0


                        for i in range(index):
                            x_midle = t_ob_arr[i][6] + x_midle
                            y_midle = t_ob_arr[i][7] + y_midle
                        x_midle = x_midle / index
                        y_midle = y_midle / index
                        if use_kalman_filter:
                            KF.predict()
                            KF.update(x_midle)
                            KF.predict()
                            KF.update(y_midle)
                            target_x = KF.x[0]
                            target_y = KF.x[0]
                        else:
                            target_x = x_midle

                            #target_y = y_midlecv.KalmanFilter

                    color = (0, 0, 255)
                    cv2.drawContours(frame, [approx], -1, color, 1)

    if targeting == True:
        t_ob_arr = []

        # two Kalman filters  for the target coordinates in the targeting scope, filter is being recreated when a new target is being found

        if use_kalman_filter:
            dt = 0.5
            # create sigma points to use in the filter. This is standard for Gaussian processes
            points = MerweScaledSigmaPoints(4, alpha=.01, beta=2., kappa=-1)

            kfx = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=dt, fx=fx, hx=hx, points=points)
            kfy = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=dt, fx=fx, hx=hx, points=points)

            kfx.x = np.array([-1., 1., -1., 1])  # initial state
            kfx.P *= 0.5  # initial uncertainty
            z_std = 0.05
            kfx.R = np.diag([z_std ** 2, z_std ** 2])  # 1 standard
            kfx.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01 ** 2, block_size=2)

            kfy.x = np.array([-1., 1., -1., 1])  # initial state
            kfy.P *= 0.5  # initial uncertainty
            z_std = 0.05
            kfy.R = np.diag([z_std ** 2, z_std ** 2])  # 1 standard
            kfy.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01 ** 2, block_size=2)

        # going through the every target in the targeting scope and append it to the t_ob_arr array
        if cnts is not None:
            for cn in cnts:

                peri = cv2.arcLength(cn, True)
                approx = cv2.approxPolyDP(cn, 0.001 * peri, True)
                (x, y, w, h) = cv2.boundingRect(approx)
                aspectRatio = w / float(h)

                area = cv2.contourArea(cn)
                hullArea = cv2.contourArea(cv2.convexHull(cn))
                if hullArea > 0:
                    solidity = area / float(hullArea)
                else:
                    solidity = 0

                if peri > peri_min and len(
                        approx) > approx_min and x > targeting_x - d_sq and x < targeting_x + d_sq and y > targeting_y - d_sq and y < targeting_y + d_sq:
                    target_x = x
                    target_y = y
                    # if color_detection:
                    #     crop_img = frame_orig[y:y + h, x:x + w]
                    #     r, g, b = crop_img.mean(axis=0).mean(axis=0)
                    # else:
                    r = 0
                    g = 0
                    b = 0

                    t_ob_arr.append([peri, len(approx), aspectRatio, area, hullArea, solidity, x, y, r, g, b])
                    cv2.rectangle(frame, (x - d_sq, y - d_sq), (x + d_sq, y + d_sq), (0, 255, 255), 2, 8)






    #print(contours)
    # set the accepted minimum & maximum area of a detected object
    min_area_thresh = 35
    max_area_thresh = 300
    min_peri_thresh = 40
    max_peri_thresh = 300



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

        epsilon = 0.01 * cv2.arcLength(cnts, True)

        # array of contours

        approx = cv2.approxPolyDP(cnts, epsilon, True)
        # print( " approx ", approx)
        # contour bounding contour

        (x, y, w, h) = cv2.boundingRect(approx)

        # contour aspect ratio
        aspectRatio = w / float(h)


        if (area > min_area_thresh) and (area < max_area_thresh) and (peri > min_peri_thresh) and (peri < max_peri_thresh):

            M = cv2.moments(c)
            cx = float(M['m10'] / M['m00'])
            cy = float(M['m01'] / M['m00'])

            indexlist.append(indexnr)
            indexnr += 1
            centers.append(np.array([[cx], [cy]]))


    #img_thresh = cv2.resize(img_thresh, (0, 0), None, scale, scale)
    #img_edges = cv2.resize(img_edges,(0,0), None, scale, scale)
    #cv2.imshow('edges', img_edges)
    opening = cv2.resize(opening, (0, 0), None, scale, scale)
    #eroded= cv2.resize(eroded, (0,0), None, scale, scale)
    cv2.imshow('opening',opening)

    centers = sorted(centers, key = lambda x: x[0])

    print(f"*************************"
          f"\n"
          f" centers {centers}"
          f"\n"
          f"*************************")
    #cv2.imshow('eroded', eroded)
    return centers, indexlist





    #cv2.destroyAllWindows()


