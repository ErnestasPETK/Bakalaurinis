from kalmanfilter import KalmanFilter
from detector import detect
from detector import click
# from numba import jit, cuda




def convert (coordinates_list):
    return tuple(coordinates_list)

import cv2
import numpy as np

# @jit(target = "cuda")
class DObjects:

    r_list = []
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

    def __init__(self):
        self.kalman_ob = DObjects.kalman_ob
        self.xcen = np.array([41])
        self.ycen = np.array([41])
        self.yp = np.array([41])
        self.xp = np.array([41])
        self.ye = np.array([41])
        self.xe = np.array([41])

    def draw(self,frame_o):

        # circle the detected position
        cv2.circle(frame_o, (int(self.xcen), int(self.ycen)), 15, (0, 191, 255), 2)

        # KALMAN predict
        self.predict_x_y()
        cv2.circle(frame_o, (int(self.yp), int(self.xp)), 20, (0, 0, 255), 2)

        self.update_x_y((self.xcen, self.ycen))



        # Draw a circle as the predicted and estimated object position


        cv2.circle(frame_o, (int(self.xe), int(self.ye)), 25, (255, 0, 0), 2)
#pakeiciau xp i xe
        cv2.putText(frame_o, f"  Estimated Position ", (int(self.ye + 15), int(self.ye + 0)), 0, 0.5,(0, 0, 255), 2)
        cv2.putText(frame_o, f"  Predicted Position ", (int(self.xp + 15), int(self.yp + 0)), 0, 0.5, (255, 0, 0), 2)
        cv2.putText(frame_o, f"  Measured Position  ", (int(self.xcen), int(self.ycen)), 0, 0.5, (0, 191, 255), 2)


    def update_sz(self,xl_search_zone, xh_search_zone, yl_search_zone, yh_search_zone):
        self.xl_search_zone = xl_search_zone
        self.xh_search_zone = xh_search_zone
        self.yl_search_zone = yl_search_zone
        self.yh_search_zone = yh_search_zone
        return print( "  self. search zones ", self.xl_search_zone, self.xh_search_zone, self.yl_search_zone, self.yh_search_zone)

    def get_sz_xl(self):
        return self.xl_search_zone

    def get_sz_xh(self):
        return self.xh_search_zone

    def get_sz_yl(self):
        return self.yl_search_zone

    def get_sz_yh(self):
        return self.yh_search_zone

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
         (self.xe, self.ye) = self.updateKalman(center_point_lt)
         #(self.xp, self.yp) = self.updateKalman(center_point_lt)
         # return (self.xp, self.yp)

    def predict_x_y(self):
         (self.xp, self.yp) = self.kalman_ob.predict()
         #(self.xe, self.ye) = self.kalman_ob.predict()
         # return (self.xe, self.ye)
    def getparam(self):
        return print (f"\n"
                      f"\n"
                      f" Object parameters:  "
                      f"\n"
                      f" xe {self.xe} ye {self.ye}"
                      f"\n"
                      f" xp {self.xp} yp {self.yp} "
                      f"\n"
                      f" xcen {self.xcen} ycen {self.ycen}"
                      f"\n")



class raketa(DObjects):


        print (" Child Class constructed for a rocket" )



def main():

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
    n_t_count = 0
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

                elif i ==9:
                    xl_sz = dobject.xl_search_zone
                    xh_sz = dobject.xh_search_zone
                    yl_sz = dobject.yl_search_zone
                    yh_sz = dobject.yh_search_zone
                    xl_sz = xl_sz
                    xh_sz = xh_sz + (640 * 2)
                    yl_sz = yl_sz
                    yh_sz = yh_sz + (360 * 2)
                    print(f" xl_sz, xh_sz, yl_sz, yh_sz  {xl_sz, xh_sz, yl_sz, yh_sz} ")
                    dobject.update_sz(xl_sz, xh_sz, yl_sz, yh_sz)



            print(f"*************************"
                  f"\n"
                  f"\n"
                  f" dobjects_arr {dobjects_arr}"
                  f"\n"
                  f"\n"
                  f"*************************")

            print(f"*************************"
                  f"\n"
                  f"\n"
                  f" centers  {centers}"
                  f"\n"
                  f"\n"
                  f"*************************")

            # pereinam per objektus ir raketu centrus. Jeigu raketos centras patenka i objekto paieskos zona, centro koordinates priskiriamos objeko self.
            # centro koordinatems

            for dobject in dobjects_arr:

                dobject.get_sz()

                for center in centers:
                     x_c = center[0]
                     y_c = center[1]


                     if  dobject.get_sz_xl() <  x_c < dobject.get_sz_xh() and dobject.get_sz_yl() < y_c < dobject.get_sz_yh():

                        print ( f" atitiko ")
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
                Dobject.getparam()

                print(f"*************************"
                      f"\n"
                      f"\n"
                      f"  search zone :   " 
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

                    print ( f" x_c y_c  {x_c , y_c } ")

                    # Jeigu raketa yra "objekto" paieskos zonoje, tuomet imame koordinates ir atvaizduojam jas

                    if  Dobject.get_sz_xl() <  x_c < Dobject.get_sz_xh() and Dobject.get_sz_yl() < y_c < Dobject.get_sz_yh():

                        #sukurti atskira objekta vienam atrastam konturui ir updatiniti jo koordinates jeigu patenka i jo predicted vietos zona



                        if len(Dobject.r_list)> 0:
                            for rocket in Dobject.r_list:
                                if x_c < rocket.get_xcen() + rocket.zone_x and x_c > rocket.get_xcen() - rocket.zone_x and y_c < rocket.get_ycen() + rocket.zone_y and y_c > rocket.get_ycen() - rocket.zone_y:

                                    print( f" rocket in zone of another rocket  " )

                                    rocket.update_x_y((x_c,y_c))
                                else:
                                    print( f" rocket located with enough distance from any other rocket ")
                                    rocketobj = raketa()
                                    rocketobj.set_xcen(x_c)
                                    rocketobj.set_ycen(y_c)
                                    rocketobj.update_x_y((x_c, y_c))
                                    Dobject.r_list.append(rocketobj)


                            for rockets in Dobject.r_list:
                                #rockets.draw(frame)
                                print ( f'  raketos parametrai   ')
                                rockets.getparam()


                        else:
                            rocketobj = raketa()
                            rocketobj.set_xcen(x_c)
                            rocketobj.set_ycen(y_c)
                            rocketobj.update_x_y((x_c,y_c))
                            Dobject.r_list.append( rocketobj)



                        Dobject.set_xcen(x_c)
                        Dobject.set_ycen(y_c)

                        print( " raketa paieskos zonoje " )

                        Dobject.update_x_y((x_c, y_c))


                        print (f"  Object:  {Dobject}  ")
                        Dobject.getparam()
                        Dobject.draw(frame)
            for DObject in dobjects_arr:
                DObject.getparam()
                DObject.draw(frame)


        else:
            # predict
            for dobject in dobjects_arr:
                dobject.kalman_ob.predict()
                x = dobject.get_xe()
                y = dobject.get_ye()

                print ( f"    x = dobject.get_xe() y = dobject.get_ye()    {x , y}")
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

