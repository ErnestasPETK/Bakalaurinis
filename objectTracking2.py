from kalmanfilter import KalmanFilter
from detector import detect
from detector import click
#from kalmanfilter import raketa
#from numba import jit, cuda
import cv2



#@jit(target = "cuda")
def main():

    HighSpeed = 100
    ControlSpeedVar = 100 #while lowest: 1 - Highest: 100

    debugMode = 1

    # Create KalmanFilter object KF
    # KalmanFilter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)
    KF = KalmanFilter(0.05, 4, 4, 4, 4, 4)
    '''
    Set the parameters values as: dt = 0.1 pip , u_x = 1, u_y=1, std_acc = 1, y_std_meas =1. 
    Will try to set different values in the future after observing the performance.

    '''
    count = 0

    VideoCap = cv2.VideoCapture('video3.mp4')


    while(True):
        #read frame
        ret, frame = VideoCap.read()
        #
        cv2.setMouseCallback("Rocket-Tracking", click)
        #detect object

        centers, indexlist = detect(frame,debugMode)
        print("centers : ", centers, "end ")
        print( "indexlist:   ", indexlist, "end   ")


        #in centroids are detected then track them
        if(len(centers)>0):


            if count < len(indexlist) - 1:

                count += 1

            else:

                print(f"count: {count}")
                count = 0

            # object name generation
            # name = 'raketa_nr_{}'.format(i)
            # print (" name   ", name )


            x_coordinate = centers[0][0]
            y_coordinate = centers[0][1]

            print(f" x_coordinate  {x_coordinate}")
            print(f" y_coordinate  {y_coordinate}")


            cv2.circle(frame, (int(x_coordinate), int(y_coordinate)), 15, (0, 191, 255), 2)

            # KALMAN predict

            (x, y) = KF.predict()
            print ( f"   PREDICT X {x}  Y {y}  ")

            # Draw a circle as the predicted object position

            cv2.circle(frame, (int(x), int(y)), 25, (255, 0, 0), 2)

            # KALMAN update
            print (f"  centers[0]   {centers[0]}")
            (x1, y1) = KF.update(centers[0])
            print( f"  UPDATE  X1 {x1}  Y1  {y1} ")
            # Draw a rectangle as the estimated object position
            cv2.rectangle(frame, (int(x1 - 15), int(y1 - 15)), (int(x1 + 15), int(y1 + 15)), (0, 0, 255), 2)
            #
            cv2.putText(frame, f"  Estimated Position ", (int(x1 + 15), int(y1 + 10)), 0, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f"  Predicted Position ", (int(x + 15), int(y + 0)), 0, 0.5, (255, 0, 0), 2)
            cv2.putText(frame,f"  Measured Position  ", (int (x_coordinate), int (y_coordinate)), 0, 0.5, (0, 191, 255), 2)


        else:
            # predict

            (x, y) = KF.predict()

            # Draw a rectangle as the predicted object position

            cv2.rectangle(frame, (int(x - 40), int(y - 40)), (int(x + 40), int(y + 40)), (255, 0, 0), 2)





        cv2.imshow('Rocket-Tracking', frame)
        if cv2.waitKey(2) & 0xFF == ord('q'):
            VideoCap.release()
            cv2.destroyAllWindows()
            break

        cv2.waitKey(HighSpeed-ControlSpeedVar+1)
if __name__ == "__main__":
    #execute main
    main()

