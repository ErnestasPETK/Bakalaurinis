from kalmanfilter import KalmanFilter
from detector import detect
from detector import click
from kalmanfilter import raketa
# from numba import jit, cuda

def convert (coordinates_list):
    return tuple(coordinates_list)

import cv2
import numpy as np


# @jit(target = "cuda")
class DObjects:


    xp = 0
    yp = 0

    xe = 0
    ye = 0
    zone_x = 150
    zone_y = 150
    xl_search_zone = 0
    xh_search_zone = 640
    yl_search_zone = 0
    yh_search_zone = 360


    kalman_ob = KalmanFilter(0.05, 4, 4, 4, 4, 4)

    def update_sz(self,xl_search_zone, xh_search_zone, yl_search_zone, yh_search_zone):
        self.xl_search_zone = xl_search_zone
        self.xh_search_zone = xh_search_zone
        self.yl_search_zone = yl_search_zone
        self.yh_search_zone = yh_search_zone
        return print( "  self. search zones ", self.xl_search_zone, self.xh_search_zone, self.yl_search_zone, self.yh_search_zone)


    def get_sz(self):
        return print(f" self.xl_search_zone, self.xh_search_zone, self.yl_search_zone, self.yh_search_zone {self.xl_search_zone, self.xh_search_zone, self.yl_search_zone, self.yh_search_zone}")

    def get_xe(self):
        return self.xe

    def get_ye(self):
        return self.ye

    def get_xp(self):
        return self.xp

    def get_yp(self):
        return self.yp

    def set_xcen(self, xcen):
        self.xcen= xcen

    def set_ycen(self, ycen):
        self.ycen= ycen

    def get_xcen(self):
        return self.xcen

    def get_ycen(self):
        return self.ycen

    def updateKalman(self, center_point_lt):
        return self.kalman_ob.update(center_point_lt)

    def update_x_y(self, center_point_lt):
         (self.xp, self.yp) = self.updateKalman(center_point_lt)
         # return (self.xp, self.yp)

    def predict_x_y(self):
         (self.xe, self.ye) = self.kalman_ob.predict()
         # return (self.xe, self.ye)




def main():
    # dobjects_arr = []
    # dobjects_arr.append(DObjects())
    # print(dobjects_arr)
    HighSpeed = 100
    ControlSpeedVar = 100  # while lowest: 1 - Highest: 100
    dobjects_arr = []
    debugMode = 1

    # Create KalmanFilter object KF
    # KalmanFilter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)

    '''
    Set the parameters values as: dt = 0.1 pip , u_x = 1, u_y=1, std_acc = 1, y_std_meas =1. 
    Will try to set different values in the future after observing the performance.

    '''
    count = 0

    VideoCap = cv2.VideoCapture('video3.mp4')

    while (True):
        # read frame
        ret, frame = VideoCap.read()
        #
        cv2.setMouseCallback("Rocket-Tracking", click)
        # detect object

        centers, indexlist = detect(frame, debugMode)
        #print("centers : ", centers, "end ")
        #print("indexlist:   ", indexlist, "end   ")

        centers = convert(centers)
         # creating 10 objects only in the first frame

        if count == 0:
            for i in range(10):

                dobjects_arr.append(DObjects())
                print (f" i  {i} ")
                #assign 9 seach zones
                dobject = dobjects_arr[i]
                if i < 3:

                    xl_sz = dobject.xl_search_zone
                    xh_sz = dobject.xh_search_zone
                    yl_sz = dobject.yl_search_zone
                    yh_sz = dobject.yh_search_zone
                    yl_sz = yl_sz + (360 * i)
                    yh_sz = yh_sz + (360 * i)

                    print (f" xl_sz, xh_sz, yl_sz, yh_sz  {xl_sz, xh_sz, yl_sz, yh_sz} ")

                    dobject.update_sz(xl_sz, xh_sz, yl_sz, yh_sz)
                elif 2 < i < 6:
                    xl_sz = dobject.xl_search_zone
                    xh_sz = dobject.xh_search_zone
                    yl_sz = dobject.yl_search_zone
                    yh_sz = dobject.yh_search_zone

                    xl_sz = xl_sz + (640 * 1)
                    xh_sz = xh_sz + (640 * 1)
                    yl_sz = yl_sz + (360 * (i-3))
                    yh_sz = yh_sz + (360 * (i-3))

                    print(f" xl_sz, xh_sz, yl_sz, yh_sz  {xl_sz, xh_sz, yl_sz, yh_sz} ")

                    dobject.update_sz(xl_sz, xh_sz, yl_sz, yh_sz)
                elif 5 < i <9:
                    xl_sz = dobject.xl_search_zone
                    xh_sz = dobject.xh_search_zone
                    yl_sz = dobject.yl_search_zone
                    yh_sz = dobject.yh_search_zone
                    xl_sz = xl_sz + (640 * 2)
                    xh_sz = xh_sz + (640 * 2)
                    yl_sz = yl_sz + (360 * (i-6))
                    yh_sz = yh_sz + (360 * (i-6))

                    print(f" xl_sz, xh_sz, yl_sz, yh_sz  {xl_sz, xh_sz, yl_sz, yh_sz} ")
                    dobject.update_sz(xl_sz, xh_sz, yl_sz, yh_sz)

            print(f"*************************"
                  f"\n"
                  f"\n"
                  f" dobjects_arr {dobjects_arr}"
                  f"\n"
                  f"\n"
                  f"*************************")



            for dobject in dobjects_arr:

                dobject.get_sz()
                print( "  OBJECT NAME  " , dobject)

                for center in centers:
                    x_c = center[0]
                    y_c = center[1]

                    if  dobject.xl_search_zone <  x_c < dobject.xh_search_zone and dobject.yl_search_zone < dobject.yh_search_zone:

                        dobject.set_xcen(x_c)
                        dobject.set_ycen(y_c)

                        dobject.update_x_y((x_c, y_c))
            count += 1



        print(f"*************************"
              f"\n"
              f"\n"
              f"  reset "
              f"\n"
              f"\n"
              f"*************************")


    # in centroids are detected then track them
        if (len(centers) > 0):


            for Dobject in dobjects_arr:

                print(f"\n"
                      f" object  {Dobject}"
                      f"\n"
                      f"*************************")

                print(f"*************************"
                      f"\n"
                      f"\n"
                      f"  search zones :   " 
                    
                      f"\n"
                      f"\n"
                      f"*************************")
                Dobject.get_sz()
                for center in centers:
                    print(f"\n"
                          f" center point  {center}"
                          f"\n"
                          f"*************************")
                    x_c = center[0]
                    y_c = center[1]


                    # Jeigu raketa yra "objekto" paieskos zonoje, tuomet imame koordinates ir atvaizduojam jas

                    if Dobject.xl_search_zone < x_c < Dobject.xh_search_zone and Dobject.yl_search_zone < y_c < Dobject.yh_search_zone:
                        Dobject.set_xcen(x_c)
                        Dobject.set_ycen(y_c)

                        Dobject.update_x_y((x_c, y_c))


                        if Dobject.xcen > x_c - Dobject.zone_x and Dobject.xcen < x_c + Dobject.zone_x:
                            if Dobject.ycen > y_c - Dobject.zone_y and Dobject.ycen < y_c + Dobject.zone_y:
                                print("in zone")

                                # *******************************************************************************************#
                                # for center in centers:

                                x_coordinate = center[0]
                                y_coordinate = center[1]

                                Dobject.set_xcen(x_coordinate)
                                Dobject.set_ycen(y_coordinate)

                                print(f" x_coordinate  {x_coordinate}")
                                print(f" y_coordinate  {y_coordinate}")

                                cv2.circle(frame, (int(Dobject.get_xcen()), int(Dobject.get_ycen())), 15, (0, 191, 255), 2)

                                                # KALMAN predict

                                Dobject.predict_x_y()
                                x = Dobject.get_xe()
                                y = Dobject.get_ye()

                                                # Draw a circle as the predicted object position

                                cv2.circle(frame, (int(x), int(y)), 25, (255, 0, 0), 2)

                                                # KALMAN update
                                center_x = center[0]
                                center_y = center[1]
                                Dobject.update_x_y((center_x, center_y))
                                x1 = Dobject.get_xp()
                                y1 = Dobject.get_yp()

                                                # Draw a rectangle as the estimated object position
                                cv2.rectangle(frame, (int(x1 - 15), int(y1 - 15)), (int(x1 + 15), int(y1 + 15)),
                                                              (0, 0, 255), 2)
                                                #
                                cv2.putText(frame, f"  Estimated Position ", (int(x1 + 15), int(y1 + 10)), 0, 0.5,
                                                            (0, 0, 255), 2)
                                cv2.putText(frame, f"  Predicted Position ", (int(x + 15), int(y + 0)), 0, 0.5, (255, 0, 0),
                                                            2)
                                cv2.putText(frame, f"  Measured Position  ", (int(x_coordinate), int(y_coordinate)), 0, 0.5,(0, 191, 255), 2)
                                #     # jeigu gautas centras yra objekte esancio predicted x
                                # if x_c > dobject.xp + dobject.zone_x and x_c < dobject.xp - dobject.zone_x and y_c > dobject.yp + dobject.zone_y and y_c < dobject.yp - dobject.zone_y:
                                #     centers_iter = iter(centers)
                                #     center = next(centers_iter)
                                #     print(f"*************************"
                                #           f"\n"
                                #           f" center after iter {center}"
                                #           f"\n"
                                #           f"*************************")

            count+=1


        else:
            # predict
            for dobject in dobjects_arr:
                dobject.kalman_ob.predict()
                x = dobject.get_xe()
                y = dobject.get_ye()
                # Draw a rectangle as the predicted object position
                cv2.rectangle(frame, (int(x - 40), int(y - 40)), (int(x + 40), int(y + 40)), (255, 0, 0), 2)

        cv2.imshow('Rocket-Tracking', frame)
        if cv2.waitKey(2) & 0xFF == ord('q'):
            VideoCap.release()
            cv2.destroyAllWindows()
            break

        cv2.waitKey(HighSpeed - ControlSpeedVar + 1)


if __name__ == "__main__":
    # execute main
    main()

