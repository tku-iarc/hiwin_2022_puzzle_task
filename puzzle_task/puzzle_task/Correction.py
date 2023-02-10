import pyrealsense2 as rs           # 版本目前不支援python3.10
import cv2
import numpy as np
import random
import math


# video = cv2.VideoCapture("/home/weng/ICLAB/output2.avi")

# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out_video = cv2.VideoWriter('demo_Correction_canny.avi', fourcc, 30.0, (1280,  720))


'''
    用完後必關！！！！！！！！重要！！！！！
'''
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# pipeline.start(config)
# sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
# sensor.set_option(rs.option.auto_exposure_priority, True)



def detect(img):
    gradient = Canny(img)
    outline_img,angle,center = detect_angle(img,gradient)

    return angle, center, outline_img



def Canny(orig_img):
    # 灰階
    # gray_img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    gauss_img = cv2.GaussianBlur(orig_img, (3, 3), 0)

    # Canny邊緣運算
    low_threshold = 100
    high_threshold = 120
    canny_img = cv2.Canny(gauss_img, low_threshold, high_threshold)

    # 閉運算(緩解Canny斷線問題)
    kernel = np.ones((5,5),np.uint8)
    gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

    return gradient


def detect_angle(orig_img, canny_img):
    outline_img = orig_img.copy()

    # 輪廓檢測(使用Canny檢測後影像繼續檢測)
    ret,thresh = cv2.threshold(canny_img,180,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    cnt = 0
    check = 0

    for i in range(len(contours)):
        if cv2.contourArea(contours[i])<300000 and cv2.contourArea(contours[i])>80000:
            cnt = contours[i]
            check = 1
            # print('area=',cv2.contourArea(contours[i]))

    if check == 1:
        rect = cv2.minAreaRect(cnt)     # 生成最小外接矩形
        box = cv2.boxPoints(rect)       # 生成外接矩形各點座標
        box = np.int0(box)              # 轉換為整數
        cv2.drawContours(outline_img, [box], 0, (0, 0, 255), 2)
        cv2.putText(outline_img,'1. '+str(box[0]),tuple(box[0]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(outline_img,'2. '+str(box[1]),tuple(box[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(outline_img,'3. '+str(box[2]),tuple(box[2]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(outline_img,'4. '+str(box[3]),tuple(box[3]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        tem_length_1to2 = math.sqrt(pow(box[0][0]-box[1][0],2) + pow(box[0][1]-box[1][1],2)).real
        tem_length_1to4 = math.sqrt(pow(box[0][0]-box[3][0],2) + pow(box[0][1]-box[3][1],2)).real
        if tem_length_1to2 > tem_length_1to4:
            star = (0,3)
            end = (1,2)
        else:
            star = (0,1)
            end = (2,3)
        x_start = ( (box[star[0]][0]+box[star[1]][0])/2 )
        y_start = ( (box[star[0]][1]+box[star[1]][1])/2 )
        x_end = ( (box[end[0]][0]+box[end[1]][0])/2 )
        y_end = ( (box[end[0]][1]+box[end[1]][1])/2 )
        origin_coordinate_x = int((x_start+x_end)/2)
        origin_coordinate_y = int((y_start+y_end)/2)
        if y_start>y_end:
            # 左上角(0,0)
            tem = x_start
            x_start = x_end
            x_end = tem
            tem = y_start
            y_start = y_end
            y_end = tem
        else:
            pass
        cv2.line(outline_img, (int(x_start),int(y_start)), (int(x_end),int(y_end)), (0, 100, 255), 1)

        angle_1 = round(math.degrees(math.atan2( -(y_start-((y_start+y_end)/2)) , x_start-((x_start+x_end)/2) )), 5)
        angle_2 = round(math.degrees(math.atan2( -(y_end - ((y_start+y_end)/2)) , x_end - ((x_start+x_end)/2) )), 5)
        # print('angle= ',angle_1,'or',angle_2)
        cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int(x_start),int(y_start)), (255, 0, 0), 2)
        cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int(x_end),int(y_end)), (0, 0, 255), 2)
        cv2.putText(outline_img,str(angle_1),(int(x_start),int(y_start)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 20, 0), 2, cv2.LINE_AA)
        cv2.putText(outline_img,str(angle_2),(int(x_end),int(y_end)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 20, 100), 2, cv2.LINE_AA)
        
        x_offset = ((x_start+x_end)/2)-(1280/2)
        y_offset = ((y_start+y_end)/2)-(720/2)
        if x_offset>0:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2)+200,int((y_start+y_end)/2)), (50, 255, 0), 3)
            cv2.putText(outline_img,'X['+str(x_offset)+']',(int((x_start+x_end)/2)+210,int((y_start+y_end)/2)-10),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)
        else:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2)-200,int((y_start+y_end)/2)), (50, 255, 0), 3)
            cv2.putText(outline_img,'X['+str(x_offset)+']',(int((x_start+x_end)/2)-210,int((y_start+y_end)/2)-10),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)

        if y_offset>0:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2),int((y_start+y_end)/2)+200), (50, 255, 0), 3)
            cv2.putText(outline_img,'Y['+str(y_offset)+']',(int((x_start+x_end)/2-10),int((y_start+y_end)/2)+220),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)
        else:
            cv2.arrowedLine(outline_img, (int((x_start+x_end)/2),int((y_start+y_end)/2)), (int((x_start+x_end)/2),int((y_start+y_end)/2)-200), (50, 255, 0), 3)
            cv2.putText(outline_img,'Y['+str(y_offset)+']',(int((x_start+x_end)/2-10),int((y_start+y_end)/2)-220),cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 0), 2, cv2.LINE_AA)


        # print(x_offset, y_offset)

        return outline_img, angle_1, [x_offset,y_offset]
    else:
        return orig_img,0,[0,0]



if __name__=="__main__":

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    # Convert images to numpy arrays 把图像转换为numpy data
    color_image = np.asanyarray(color_frame.get_data())

    image = cv2.imread('D:/ICLAB/Correction/555.png')

    while 1:
        # ret, image = video.read()
        frames = pipeline.wait_for_frames()
        image = frames.get_color_frame()
        image = np.asanyarray(image.get_data())

        gradient = Canny(image)
        outline_img,angle,center = detect_angle(image,gradient)
        print(angle,center)

        # out_video.write(gradient)
        cv2.imshow('gradient', gradient)
        cv2.imshow('outline_img', outline_img)
        # cv2.waitKey(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# video.release()
# out_video.release()
cv2.destroyAllWindows()
