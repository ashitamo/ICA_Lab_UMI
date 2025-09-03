from dt_apriltags import Detector
import numpy as np
import os
import cv2
import math
from scipy.spatial.transform import Rotation as R

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

'''
mtx: [[839.72913173   0.         311.87321929]
 [  0.         835.51579792 255.38762288]
 [  0.           0.           1.        ]]'''
at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("實際設定的解析度:", (actual_width, actual_height))
# 檢查實際設定的FPS
actual_fps = cap.get(cv2.CAP_PROP_FPS)
print("實際設定的FPS:", actual_fps)

camera_params = [839.72913173*2,835.51579792*2,311.87321929*2,255.38762288*2]
while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = at_detector.detect(gray,True,camera_params,0.21)
    cv2.circle(frame,(int(camera_params[2]),int(camera_params[3])),5,(0,0,255),-1)
    if len(results) != 0:
        cv2.circle(frame,(int(results[0].center[0]),int(results[0].center[1])),5,(0,255,0),-1)

        cv2.line(frame,(int(results[0].corners[0][0]),int(results[0].corners[0][1])),
                       (int(results[0].corners[1][0]),int(results[0].corners[1][1])),(255,0,0),2)
        
        cv2.line(frame,(int(results[0].corners[1][0]),int(results[0].corners[1][1])),
                       (int(results[0].corners[2][0]),int(results[0].corners[2][1])),(255,0,0),2)
        
        cv2.line(frame,(int(results[0].corners[2][0]),int(results[0].corners[2][1])),
                       (int(results[0].corners[3][0]),int(results[0].corners[3][1])),(255,0,0),2)
        
        cv2.line(frame,(int(results[0].corners[3][0]),int(results[0].corners[3][1])),
                       (int(results[0].corners[0][0]),int(results[0].corners[0][1])),(255,0,0),2)
        
        # print(results[0])
        # results[0].pose_t = -results[0].pose_R @ results[0].pose_t
        print('x',results[0].pose_t[0])
        print('y',results[0].pose_t[1])
        print('z',results[0].pose_t[2])
        print(rotationMatrixToEulerAngles(results[0].pose_R)/np.pi*180)
        print(results[0].pose_R)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


