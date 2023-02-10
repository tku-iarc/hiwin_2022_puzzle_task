import pyrealsense2 as rs           # 版本目前不支援python3.10
from calendar import c
from operator import mod
#from re import X
#from tkinter import Y
import numpy as np
from ctypes import *
import time
import rospy
import pyrealsense2 as rs           # 版本目前不支援python3.10
import cv2
'''
    DO(int DO_Num, int x)                                                         # 1 -> on ; 0 -> off                                          
    HOME(int state)                                                               # 1 RUN
    PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle)            # 0 -> joint ; 1 -> coordinate
    LIN(int type, double *XYZ, int vel, int acc, int TOOL, int BASE)               # 0 -> joint ; 1 -> coordinate
    CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE) 
    JOG(int joint,int dir)
'''

so_file = "/home/weng/RoboticArm_Puzzle/main/modbus_file/Hiwin_API.so"
modbus = CDLL(so_file)



# '''
#     相機設定
# '''
# # Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# # Start streaming
# pipeline.start(config)

# sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
# sensor.set_option(rs.option.auto_exposure_priority, True)

# filePath = "/home/weng/ICLAB/"
# id = "1"
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('/home/weng/ICLAB/output8.avi', fourcc, 30.0, (1280, 720))


# def camera():
#     frames = pipeline.wait_for_frames()
#     color_frame = frames.get_color_frame()
#     # Convert images to numpy arrays 把图像转换为numpy data
#     color_image = np.asanyarray(color_frame.get_data())
#     # cv2.line(color_image, (640, 0), (640, 720), (0, 0, 255), 1)
#     # cv2.line(color_image, (0, 360), (1280, 360), (0, 0, 255), 1)

#     out.write(color_image)

#     cv2.namedWindow('MyD415')
#     cv2.imshow('MyD415', color_image)

#     key = cv2.waitKey(1)


#     imageName = "temp.jpg"

#     imagePath = filePath + imageName



#     if key & 0xFF == ord('q') or key == 27:
#         out.release()
#         cv2.destroyAllWindows()
#         pipeline.stop()
#     elif key == ord('s'):  # 按下s键时保存并退出
#         cv2.imwrite(imagePath, color_image)
#         #cv2.destroyAllWindows()



def PTP_Move(X, Y, Z, pitch, roll, yaw, speed=50, acceleration=1):
    print("PTP Move ...")
    PTP_XYZ   = [X, Y, Z, pitch, roll, yaw]                 # XYZABC
    C_PTP_XYZ = (c_double * len(PTP_XYZ))(*PTP_XYZ)         # C Array
    #PTP_Angle = [X, Y, Z, pitch, roll, yaw]                 # ANGLE
    #C_PTP_Angle = (c_double * len(PTP_Angle))(*PTP_Angle)       # C Array
    modbus.PTP(1, speed, acceleration, 0, 1, C_PTP_XYZ)
    # aa = modbus.Arm_State_REGISTERS()
    bb = modbus.Read_REGISTERS(403)
    print(bb)
    time.sleep(3)
    # aa = modbus.Arm_State_REGISTERS()
    bb = modbus.Read_REGISTERS(403)
    print(bb)
    while 1:
        bb= modbus.Read_REGISTERS(403)
        print(bb)
        if(modbus.Arm_State_REGISTERS() == 1):
            break
    print("END PTP Move")



# def LIN_Move(Point, speed=50, acceleration=1):
#     print(bcolors.Move + "    LIN Move ... " + bcolors.RESET ,end='')
#     C_LIN_XYZ = (c_double * len(Point))(*Point)         # C Array

#     modbus.LIN(1, speed, acceleration, 1, 0, C_LIN_XYZ)
#     time.sleep(3)
#     while 1:
#         if(modbus.Arm_State_REGISTERS() == 1):
#             break
#         time.sleep(0.1)
#     print(bcolors.EndMove + "END LIN Move!" + bcolors.RESET)



if __name__ == "__main__":

    # setting
    point = [204.049, 368, 110, 180, 0, 90]
    num=0
    Arm_state = 0
    sucker_Port = 300                      # 吸盤開關(D0)
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()
    # move
    #PTP_Move(0, 0, 0, 0, -90, 90, 10, 1)


    while 1:
        # key = cv2.waitKey(10)
        
        # if key & 0xFF == ord('m'):
        num = input("input: ")
        
        #camera()
        
        if num == '1':
            # LIN_Move(point,50,1)
            PTP_Move(250, 185, 0, 180, 0, 90, 50, 1)  #影像校正上方
        elif num == '2':
            PTP_Move(250, 185, -10, 180, 0, 90, 5, 1)   #影像校正
        elif num == '3':
            PTP_Move(-12.825, 91.778, 261.325, -180, 0, 90, 30, 1)   #左拍照
        elif num == '4':
            PTP_Move(416.058, 91.778, 261.325, -180, 0, 90, 30, 1)   #右拍照
        elif num == '5':
            PTP_Move(416.058, 91.778, 40, -179.091, 0, 90, 5, 1)   #右拍照
        else:
            pass
        # num=0
        #52.055,33.044
        #48.335,32.659
        
    # PTP_Move(118.859, 286.726, -95.501, -179.978, 0.035, 89.997, 10, 1)    #中右下
    # time.sleep(1.5)
    # PTP_Move(323.184, 286.726, -95.501, -179.978, 0.035, 89.997, 10, 1)    #
    # time.sleep(1.5)
    
    # -150.862 523.017 -19.107 180 0 -90.467
    #time.sleep(1.5)
    modbus.DO(sucker_Port,1) # 1 -> Son ; 0 -> off
    time.sleep(1.5)
    modbus.DO(sucker_Port,0) # 1 -> on ; 0 -> off
    time.sleep(1.5)

    #end
    modbus.Modbus_Close()
    print("Modbus Close") 

    

