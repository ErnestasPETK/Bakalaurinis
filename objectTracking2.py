from kalmanfilter import KalmanFilter
from detector import detect
from detector import click
#from numba import jit, cuda
import cv2

# Klase, kurioje bus laikomos kiekvienos raketos objekto koordinates, KF(Kalman Filter) naudojamas objekto sekimui bei movement predicting
class rocket:


    x_set = False
    updated= False
    def __init__(self, KF):
        self.KF = KF
        self.rocket_timer = 0
        self.xe = [0]
        self.ye = [0]
        self.x_p = [0]
        self.y_p = [0]
        self.xcen = 0
        self.ycen = 0
    def set_xcen(self,xcen):
        self.xcen = xcen
    def set_ycen(self,ycen):
        self.ycen = ycen
    def get_xcen(self):
        return self.xcen
    def get_ycen(self):
        return self.ycen
    def rocket_predict(self):
        self.x_p,self.y_p = self.KF.predict()

        return self.x_p, self.y_p
    def coords_update(self,xye):
        self.xe,self.ye = self.KF.update(xye)
        self.rocket_timer = 0

        return self.xe,self.ye
    def set_xe(self,xe):
        self.xe = xe
    def set_ye(self,ye):
        self.ye = ye
    def get_xp(self):
        return self.x_p
    def get_yp(self):
        return self.y_p

    def set_xp(self, x_p):
        self.x_p = x_p

    def set_yp(self,y_p):
        self.y_p = y_p

    def getparam(self):
        return print(f"\n"
                     f"\n"
                     f" Object parameters:  "
                     f"\n"
                     f" xe {self.xe} ye {self.ye}"
                     f"\n"
                     f" xp {self.x_p} yp {self.y_p} "
                     f"\n"
                     f" xcen {self.xcen} ycen {self.ycen}"
                     f"\n")
    def timer(self):
        self.rocket_timer+=1


#@jit(target = "cuda")
def main():

    HighSpeed = 100
    ControlSpeedVar = 100 #while lowest: 1 - Highest: 100

    debugMode = 1

    # Create KalmanFilter object KF
    # KalmanFilter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)

    '''
    Set the parameters values as: dt = 0.1 pip , u_x = 1, u_y=1, std_acc = 1, y_std_meas =1. 
    Will try to set different values in the future after observing the performance.

    '''
    i = 0
    count = 0
    rocket_list = []
    VideoCap = cv2.VideoCapture('video3_cut.mp4')
    zone = 80
    for i in range(10):
        KF = KalmanFilter(0.05, 4, 4, 4, 4, 4)
        rocket_list.append(rocket(KF))
        print(f" i  {i} ")



    while(True):
        #read frame
        ret, frame = VideoCap.read()
        #
        cv2.setMouseCallback("Rocket-Tracking", click)
        #detect object

        # Gauname tinkamu konturu centrus
        centers, indexlist = detect(frame,debugMode)

        print (f" \n \n     reset     \n \n ")
        print("centers : ", centers, "end ")
        print( "indexlist:   ", indexlist, "end   ")

        #in centroids are detected then track them
        if(len(centers)>0):

            #perrenkam centrus

            for center in centers:

                x_coordinate = center[0]
                y_coordinate = center[1]

                print(f" x_coordinate  {x_coordinate}")
                print(f" y_coordinate  {y_coordinate}")

                print ( f" rocket_list \n \n  {rocket_list}  \n \n  *************************" )

                #jeigu egzistuoja raketos liste objektai:


                    # perrenkam kiekviena objekta
                for raketa in rocket_list:

                    r_c_x = raketa.get_xcen()
                    r_c_y = raketa.get_ycen()
                    print ( f"  rcx  {r_c_x} rcy  {r_c_y} ")

                    # 10 skirtingu objektu

                    if (int(x_coordinate) < int(r_c_x) + zone and int(x_coordinate) > int(r_c_x) - zone and \
                            int(y_coordinate) < int(r_c_y) + zone and int(y_coordinate) > int(r_c_y) - zone):

                        print(f" rocket in zone of an existing rocket  ")
                        raketa.set_xcen(x_coordinate)
                        raketa.set_ycen(y_coordinate)
                        raketa.rocket_predict()
                        raketa.coords_update((x_coordinate, y_coordinate))

                        raketa.updated = True

                        print ( " \n  break \n ")

                        break


                    else:

                        if (r_c_x == 0 and r_c_y == 0):
                            raketa.set_xcen(x_coordinate)
                            raketa.set_ycen(y_coordinate)
                            raketa.coords_update((x_coordinate,y_coordinate))
                            raketa.set_xp(x_coordinate)
                            raketa.set_yp(y_coordinate)

                            break

                        else:
                            print(" \n  Continue  \n")

                            continue







        else:

            for Eraketa in rocket_list:
                r_c_x = Eraketa.get_xcen()
                r_c_y = Eraketa.get_ycen()

                print(f" ELSE  rcx  {r_c_x} rcy  {r_c_y} ")
                if (r_c_x != 0 and r_c_y != 0):
                    (x_predicted, y_predicted) = Eraketa.rocket_predict()
                    x_center = Eraketa.get_xcen()
                    y_center = Eraketa.get_ycen()
                    (x_estimated, y_estimated) = Eraketa.coords_update((x_center, y_center))

                    print ( f" x_estimated  {x_estimated} y_estimated {y_estimated}  ")
                    print(f" x_predicted  {x_predicted} y_predicted {y_predicted}  ")

                    # KALMAN predict
                    # Draw a circle as the predicted object position
                    cv2.circle(frame, (int(x_center), int(y_center)), 15, (0, 191, 255), 2)
                    cv2.circle(frame, (0,0), 50 , (0,255,0), 2)
                    cv2.circle(frame, (int(x_estimated), int(y_estimated)), 20, (0, 0, 255), 2)
                    cv2.circle(frame, (int(x_predicted), int(y_predicted)), zone, (255, 0, 0), 2)
                    cv2.putText(frame, f"  Predicted Position ", (int(x_predicted + 15), int(y_predicted + 0)), 0, 0.5,(255, 0, 0), 2)
                else:
                    continue

        for RAKETA in rocket_list:
            r_c_x = RAKETA.get_xcen()
            r_c_y = RAKETA.get_ycen()

            if (r_c_x != 0) and (r_c_y != 0):
                (x_predicted, y_predicted) = RAKETA.rocket_predict()
                x_center = RAKETA.get_xcen()
                print ( " ANTRAS FORAS")
                y_center = RAKETA.get_ycen()
                cv2.circle(frame, (int(x_center), int(y_center)), 15, (0, 191, 255), 2)

            # KALMAN predict
            # Draw a circle as the predicted object position

                cv2.circle(frame, (int(x_predicted), int(y_predicted)), 25, (255, 0, 0), 2)

            # KALMAN update
                (x_estimated, y_estimated) = RAKETA.coords_update((x_center,y_center))

                print(f" 2nd x_estimated  {x_estimated} y_estimated {y_estimated}  ")
                print(f" 2nd x_predicted  {x_predicted} y_predicted {y_predicted}  ")

                # Draw a rectangle as the estimated object position
                cv2.circle(frame, (int(x_estimated),int(y_estimated)), 20, (0, 0, 255), 2)
                #
                cv2.putText(frame, f"  Estimated Position ", (int(x_estimated + 15), int(x_estimated + 10)), 0, 0.5, (0, 0, 255), 2)
                cv2.putText(frame, f"  Predicted Position ", (int(x_predicted + 15), int(y_predicted + 0)), 0, 0.5, (255, 0, 0), 2)
                cv2.putText(frame,f"  Measured Position  ", (int (x_center), int (y_center)), 0, 0.5, (0, 191, 255), 2)
                RAKETA.getparam()
            else:

                print ( "  second continue  ")
                continue
        # sutrumpinam listó ilgi
        # while i <= 20:
        #     for Object in rocket_list:
        #         Object.timer()
        #         if Object.rocket_timer > 40:
        #             del Object
        #         else:
        #             print( f' timer {Object.rocket_timer}')
        #     i+= 1
        # i = 0
        #

        cv2.imshow('Rocket-Tracking', frame)
        if cv2.waitKey(2) & 0xFF == ord('q'):
            VideoCap.release()
            cv2.destroyAllWindows()
            break

        cv2.waitKey(HighSpeed-ControlSpeedVar+1)
if __name__ == "__main__":
    #execute main
    main()

