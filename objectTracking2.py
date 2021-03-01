from kalmanfilter import KalmanFilter
from detector import detect
from detector import click
#from numba import jit, cuda
import cv2

class rocket:

    zone_x = 150
    zone_y = 150
    x_set = False
    updated= False
    def __init__(self, KF):
        self.KF = KF

    def set_xcen(self,xcen):
        self.xcen = xcen
    def set_ycen(self,ycen):
        self.ycen = ycen
    def get_xcen(self):
        print(f"\n  xcen \n  {self.xcen} ")
        return self.xcen
    def get_ycen(self):
        print(f" ycen  \n  {self.ycen}  \n")
        return self.ycen
    def rocket_predict(self):
        self.x_p,self.y_p = self.KF.predict()

        return self.x_p, self.y_p
    def coords_update(self,xye):
        self.xe,self.ye = self.KF.update(xye)

        return self.xe,self.ye
    def set_xe(self,xe):
        self.xe = xe
    def set_ye(self,ye):
        self.ye = ye

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
    count = 0
    rocket_list = []
    VideoCap = cv2.VideoCapture('video3_cut.mp4')


    while(True):
        #read frame
        ret, frame = VideoCap.read()
        #
        cv2.setMouseCallback("Rocket-Tracking", click)
        #detect object

        centers, indexlist = detect(frame,debugMode)
        print (f" \n \n     reset     \n \n ")
        print("centers : ", centers, "end ")
        print( "indexlist:   ", indexlist, "end   ")


        #in centroids are detected then track them
        if(len(centers)>0):

            for center in centers:

                x_coordinate = center[0]
                y_coordinate = center[1]

                print(f" x_coordinate  {x_coordinate}")
                print(f" y_coordinate  {y_coordinate}")

                print ( f" rocket_list \n \n  {rocket_list}  \n \n  *************************" )


                if len(rocket_list) > 0 and len(rocket_list) < 10:
                    for raketa in rocket_list:
                        if int(x_coordinate) < int(raketa.get_xcen()) + 250 and int(x_coordinate) > int(raketa.get_xcen()) - int(raketa.zone_x) and int(y_coordinate) < int(raketa.get_ycen()) + int(raketa.zone_y) and int(y_coordinate) > int(raketa.get_ycen()) - int(raketa.zone_y):

                            print(f" rocket in zone of an existing rocket  ")

                            raketa.coords_update((x_coordinate,y_coordinate))
                            raketa.set_xcen(x_coordinate)
                            raketa.set_ycen(y_coordinate)
                            raketa.updated = True
                        else:
                            print ( f" new rocket object ")
                            KF = KalmanFilter(0.05, 4, 4, 4, 4, 4)
                            OBraketa = rocket(KF)
                            OBraketa.set_xcen(x_coordinate)
                            OBraketa.set_ycen(y_coordinate)
                            OBraketa.set_xe(x_coordinate)
                            OBraketa.set_ye(y_coordinate)
                            rocket_list.append(OBraketa)


                else:
                    KF = KalmanFilter(0.05, 4, 4, 4, 4, 4)
                    OBraketa = rocket(KF)
                    OBraketa.set_xcen(x_coordinate)
                    OBraketa.set_ycen(y_coordinate)

                    rocket_list.append(OBraketa)



                # object name generation
                # name = 'raketa_nr_{}'.format(i)
                # print (" name   ", name )
            if len(rocket_list) > 0 and len(rocket_list) < 10:
                for RAKETA in rocket_list:
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


                # Draw a rectangle as the estimated object position
                    cv2.circle(frame, (int(x_estimated),int(y_estimated)), 20, (0, 0, 255), 2)
                    #
                    cv2.putText(frame, f"  Estimated Position ", (int(x_estimated + 15), int(x_estimated + 10)), 0, 0.5, (0, 0, 255), 2)
                    cv2.putText(frame, f"  Predicted Position ", (int(x_predicted + 15), int(y_predicted + 0)), 0, 0.5, (255, 0, 0), 2)
                    cv2.putText(frame,f"  Measured Position  ", (int (x_center), int (y_center)), 0, 0.5, (0, 191, 255), 2)
                    RAKETA.getparam()

        else:
            if len(rocket_list) > 0:
                for Eraketa in rocket_list:
                    (x_predicted, y_predicted) = Eraketa.rocket_predict()
                    # KALMAN predict
                    # Draw a circle as the predicted object position

                    cv2.circle(frame, (int(x_predicted), int(y_predicted)), 25, (255, 0, 0), 2)
                    cv2.putText(frame, f"  Predicted Position ", (int(x_predicted + 15), int(y_predicted + 0)), 0, 0.5,
                                (255, 0, 0), 2)



        if len(rocket_list) >10:
            rocket_list=[]
        print ( f" rocket_list \n \n  {rocket_list}  \n \n  *************************" )

        cv2.imshow('Rocket-Tracking', frame)
        if cv2.waitKey(2) & 0xFF == ord('q'):
            VideoCap.release()
            cv2.destroyAllWindows()
            break

        cv2.waitKey(HighSpeed-ControlSpeedVar+1)
if __name__ == "__main__":
    #execute main
    main()

