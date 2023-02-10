from cv2 import imshow
import pyrealsense2 as rs           # 版本目前不支援python3.10
import cv2
from calendar import c
from operator import mod
import numpy as np
from ctypes import *
import time
import copy
import math



"""
    設定
"""
# 手臂控制so檔
so_file = "/home/weng/RoboticArm_Puzzle/main/modbus_file/Hiwin_API.so"
modbus = CDLL(so_file)

# Realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
sensor.set_option(rs.option.auto_exposure_priority, True)


# arm_speed
arm_speed = 10
step = 0.1

# 目前座標
Point_now = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]            # 預設為家座標

# 計算座標(下一個點)
Point_count = [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000]          # 預設為家座標

# 基本點位
Point_basic = [ [0000.000, 0368.000, 0293.500, -180.000, 0000.000, 0090.000],       # 0: 家
                [-262.891, 0397.663, 0115.898, 0180.000, 0000.046, 0090.003],       # 1: 左收集區
                [0165.239, 0397.754, 0115.813, -179.991, 0000.046, 0090.003],       # 2: 右收集區
                [-048.228, 0446.993, 0115.813, -179.991, 0000.046, 0090.003],       # 3: 拼圖區
                [-325.385, 0252.389, -099.000, 0180.000, 0000.000, -180.000],       # 4: 影像校正初始點
                [0231.243, 0262.206, -099.000, 0180.000, 0000.000, -179.963],
                [-331.925, 0543.728, -099.000, 0180.000, 0000.000, 0179.963]]       







'''
    手臂移動
    輸入: 模式(PTP or LINE)、 速度、 加速度
    輸出: 無
    更新全域變數: Point_now
'''
def arm_move(mode='PTP', speed=50, acceleration=20):
    global Point_now

    if mode=='PTP':
        PTP_Move(Point_count, speed, acceleration)
    else:
        LIN_Move(Point_count, speed, acceleration)
    
    Point_now = copy.deepcopy(Point_count)


'''
    PTP移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def PTP_Move(Point, speed=50, acceleration=20):
    C_PTP_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.PTP(1, speed, acceleration, 1, 0, C_PTP_XYZ)
    # modbus.Arm_State_REGISTERS()
    # time.sleep(0.1)
    # while 1:
    #     if(modbus.Arm_State_REGISTERS() == 1):
    #         break
        # time.sleep(0.01)

'''
    LIN移動模式
    輸入: 點位、 速度、 加速度
    輸出: 無
    更新全域變數: 無
'''
def LIN_Move(Point, speed=50, acceleration=20):
    C_LIN_XYZ = (c_double * len(Point))(*Point)         # C Array

    modbus.LIN(1, speed, acceleration, 1, 0, C_LIN_XYZ)
    # time.sleep(0.1)
    # while 1:
    #     if(modbus.Arm_State_REGISTERS() == 1):
    #         break
        # time.sleep(0.01)







if __name__ == "__main__":
    # arm setting
    # Arm_state = 0
    # sucker_Port = 301                                   # 吸盤開關(D0)
    # modbus.DO.argtypes = [c_int, c_int]
    # modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    # modbus.LIN.argtypes = [c_int, c_int, c_int, c_int, c_int]
    # modbus.libModbus_Connect()
    # modbus.Holding_Registers_init()

    # modbus.DO(sucker_Port,1) # 1 -> on ; 0 -> off


    img = cv2.imread("/home/weng/RoboticArm_Puzzle/main/Figure_2.png")

    


    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        cv2.line(color_image, (640, 0), (640, 720), (0, 0, 255), 1)
        cv2.line(color_image, (0, 360), (1280, 360), (0, 0, 255), 1)

        cv2.line(color_image, (920, 260), (920, 460), (0, 0, 255), 1)
        cv2.line(color_image, (360, 260), (360, 460), (0, 0, 255), 1)
        cv2.line(color_image, (540, 640), (740, 640), (0, 0, 255), 1)
        cv2.line(color_image, (540, 80), (740, 80), (0, 0, 255), 1)

        cv2.namedWindow('MyD415')
        cv2.imshow('MyD415', color_image)
        key = cv2.waitKey(1)
        

        if key & 0xFF == ord('q'):
            break

        if key&0xFF == 44:
            Point_count = Point_basic[4]
            arm_move('PTP', speed=50, acceleration=100)
        elif key&0xFF == 46:
            Point_count = Point_basic[5]
            arm_move('PTP', speed=50, acceleration=100)
        elif key&0xFF == 47:
            Point_count = Point_basic[6]
            arm_move('PTP', speed=50, acceleration=100)


        if key & 0xFF != 255:
            print(key & 0xFF)
            if key&0xFF >= 49 and key&0xFF <= 53:
                if key & 0xFF == 49:
                    arm_speed = 5
                    step = 0.1
                    print("set speed = 5, step = 0.1")
                elif key & 0xFF == 50:
                    arm_speed = 5
                    step = 0.3
                    print("set speed = 5, step = 0.3")
                elif key & 0xFF == 51:
                    arm_speed = 10
                    step = 0.5
                    print("set speed = 10, step = 0.3")
                elif key & 0xFF == 52:
                    arm_speed = 20
                    step = 1
                    print("set speed = 20, step = 0.3")
                elif key & 0xFF == 53:
                    arm_speed = 50
                    step = 5
                    print("set speed = 50, step = 0.3")
            
            if key&0xFF >= 81 and key&0xFF <= 84:
                if key & 0xFF == 82:
                    Point_count[1] += step
                    print("forward")
                elif key & 0xFF == 84:
                    Point_count[1] -= step
                    print("back")
                elif key & 0xFF == 81:
                    Point_count[0] -= step
                    print("left")
                elif key & 0xFF == 83:
                    Point_count[0] += step
                    print("right")
                print(Point_count[0],Point_count[1])
                arm_move('PTP', speed=arm_speed, acceleration=100)
            
            if key&0xFF == 119:
                Point_count[2] += step
                arm_move('PTP', speed=arm_speed, acceleration=100)
                print("up")
            elif key&0xFF == 115:
                Point_count[2] -= step
                arm_move('PTP', speed=arm_speed, acceleration=100)
                print("down")
            

        
        # arm_move('PTP', speed=50, acceleration=20)
