import time
import math
import random
import os
from time import sleep

import numpy as np
import cv2
import glob

from dronekit import connect, VehicleMode
import serial
from picamera import PiCamera
from picamera.array import PiRGBArray

# 工具函数，求方差
def Variance(l):
    ex = sum(l)/len(l)
    s = 0
    for i in l:
        s = s + (i - ex)**2
    return (s / len(l))**0.5

# 初始化相机，设置分辨率和曝光时间。
# 使用usb 链接串口，写入数据
def Initialize():
    global camera,ser,f
##    f = open("log.txt","a")
    camera = PiCamera() # 初始化相机
    camera.resolution = (640,544) # 分辨率设置为 640 544
    camera.framerate = 60 # 设置曝光时间为 60 microsends
    print('Camera initialized...')
##    f.write('Camera initialized...\n')
    
    ser = serial.Serial("/dev/ttyUSB0",115200) # 使用 USB链接串行口
    ser.write('038300000000/') #向端口写数据
    print('Serial initialized...\n')
##    f.write('Serial initialized...\n\n')
    
# 拍照，并转化为 bgr 格式的array 对象
def Snapshot():    
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture,'bgr', use_video_port = True) # 捕获 bgr 格式的 rawCapture 数据流
    image = rawCapture.array
##    cv2.imwrite("rawcapture.jpg",image) 
    return image


def Calculate(image, threshold, rangefinder, range_pic_ratio):
    x_size = image.shape[1] # 返回轴和秩 取秩
    x_pic_length = rangefinder/range_pic_ratio
    
    output_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) # 用这个函数把图像从RGB转到HSV夜色空间
    ret, thresh = cv2.threshold(output_gray,threshold,255,cv2.THRESH_BINARY) # 输出阈值
    useless, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) # 获取 conters，用点表示的一个 vector
    # 每个轮廓contours[i]对应hierarchy中hierarchy[i][0]~hierarchy[i][3],分别表示后一个轮廓，前一个轮廓，父轮廓，内嵌轮廓的索引，如果没有对应项，则相应的hierarchy[i]设置为负数
##    cv2.drawContours(image, contours, -1, (255,255,0), 3) # 画轮廓的
##    cv2.imwrite("contours.jpg",image) 
    contours_OK = []

    for i in range(len(contours)):
##        if cv2.contourArea(contours[i])>9 and cv2.contourArea(contours[i])<10000:  ##TA
        contours_OK.append((cv2.contourArea(contours[i]),contours[i])) # 寻找每一个轮廓的最小包围圈

    # 找到包围圆最大的那个 contours 
    contours_OK = sorted(contours_OK,key = lambda cons_ok:cons_ok[0],reverse = True)
    if len(contours_OK) == 4:
        (x_car,y_car),radius_car = (None,None),None
        center_car = None
        center_sign = None
        radius_sign = None
        # 最大的就是是灯的位置
        (x_sign,y_sign),radius_sign = cv2.minEnclosingCircle(contours_OK[0][1])
        x_sign = int(x_sign)
        y_sign = int(y_sign)
        center_sign = (int(x_sign),int(y_sign))
        radius_sign = int(radius_sign)
        contours_OK.pop(0)

        # 剩下的三个是 车的三个角
        cnt1 = contours_OK[0][1]
        cnt2 = contours_OK[1][1]
        cnt3 = contours_OK[2][1]
        
        (x1,y1),radius1 = cv2.minEnclosingCircle(cnt1)
        (x1,y1),radius1 = (int(x1),int(y1)),int(radius1)
        (x2,y2),radius2 = cv2.minEnclosingCircle(cnt2)
        (x2,y2),radius2 = (int(x2),int(y2)),int(radius2)
        (x3,y3),radius3 = cv2.minEnclosingCircle(cnt3)
        (x3,y3),radius3 = (int(x3),int(y3)),int(radius3)

        distance1 = ((x1-x2)**2+(y1-y2)**2)**0.5
        distance2 = ((x1-x3)**2+(y1-y3)**2)**0.5
        distance3 = ((x2-x3)**2+(y2-y3)**2)**0.5

        # 计算 car 的 tail 的坐标位置 和  head 的坐标位置
        if min(distance1,distance2,distance3) == distance1:
            car_tail = (int((x1+x2)/2),int((y1+y2)/2))
            car_head = (x3,y3)
            
        elif min(distance1,distance2,distance3) == distance2:
            car_tail = (int((x1+x3)/2),int((y1+y3)/2))
            car_head = (x2,y2)
        else:
            car_tail = (int((x2+x3)/2),int((y2+y3)/2))
            car_head = (x1,y1)
        # 计算 car 重心的位置 和 中心到边角的距离（即 radius)
        if car_tail!=None and car_head!=None:
            center_car = (int((car_head[0]+car_tail[0])/2),int((car_head[1]+car_tail[1])/2))
            radius_car = int(0.5*((car_head[0]-car_tail[0])**2+(car_head[1]-car_tail[1])**2)**0.5)

        x_car = center_car[0]
        y_car = center_car[1]
##        print 'Target found!'
####        f.write('target found!\n')
##        cv2.line(image, car_tail, car_head, (255,0,255), 3)
##        cv2.circle(image,car_head,3,(0,255,255),5)
##        cv2.circle(image,car_tail,3,(0,255,255),2)
##        print 'car_head;',car_head,'car_tail:',car_tail
####        f.write('car_head:'+str(car_head)+'car_tail:'+str(car_tail)+'\n')
##        print 'center_sign:',center_sign
####        f.write('center_sign:'+str(center_sign)+'\n')
##        cv2.circle(image, center_car,radius_car, (0,255,0), 2)
##        cv2.circle(image, center_sign,radius_sign, (0,255,0), 2)
##        cv2.line(image, center_car, center_sign,(255,255,0),3)
##        cv2.circle(image, center_sign, 3,(0,255,255),5)

        '''
            获取 车灯距（第一个是坐标上的车灯距，第二个是经过换算的车灯距）
        '''
        car_signal_distance = ((x_car-x_sign)**2 + (y_car-y_sign)**2)**0.5
        real_car_signal_distance = car_signal_distance / x_size * x_pic_length
##        print 'car_signal_distance:', car_signal_distance, 'real_car_signal_distance:',real_car_signal_distance
##        f.write('car_signal_distance:' + str(car_signal_distance) + 'real_car_signal_distance:' + str(real_car_signal_distance)+'\n')
        
        '''计算车的方向向量'''
        car_direction_x = car_head[0] - car_tail[0]
        car_direction_y = car_head[1] - car_tail[1]
        '''计算车和灯连线的向量'''
        car_signal_x = x_sign - x_car
        car_signal_y = y_sign - y_car
        '''算出角度'''
        cos_angle = (car_direction_x * car_signal_x + car_direction_y * car_signal_y) / ((car_direction_x**2 + car_direction_y**2)**0.5 * (car_signal_x**2 + car_signal_y**2)**0.5)
        angle = math.acos(cos_angle) * 180/math.pi

        print '车灯向量',(car_signal_x,car_signal_y)
        print '车方向向量',(car_direction_x, car_direction_y)

#         # 对叉乘得出的值进行判断，从而确定标志物在哪个方向
#         cross_product = car_direction_x * car_signal_y - car_direction_y * car_signal_x
        
#         if cross_product > 0:
#             sign_car_side = 'right'
#         elif cross_product < 0:
#             sign_car_side = 'left'
#         else:
#             sign_car_side = 'left'
            
# ##        print 'angle:',angle, 'sign_car_side:',sign_car_side
# ##        f.write('angle:'+str(angle) + 'sign_car_side:'+ str(sign_car_side) +'\n')
        
# ##        cv2.imwrite("route.jpg",image)
#         return (4, car_head, car_tail, center_sign, car_signal_distance, real_car_signal_distance, angle, sign_car_side)

#     elif len(contours_OK) == 1:
# ##        print 'Out of range!'
# ##        f.write('Out of range!\n')
#         return (1, None, None, None, None, None, None, None)
    
#     else:
# ##        print 'len(contours_OK): ', len(contours_OK)
# ##        f.write('len(contours_OK): '+str(len(contours_OK))+'\n')
# ##        print 'No target found!'
# ##        f.write('No target found!\n')
#         return (0, None, None, None, None, None, None, None)


# control blocks

def Turn_right_small_near(angle):
    steer = 383 - (round((angle - 24)/2) + 5)
    ser.write('0'+str(int(steer))+'00000150/')
##    print "turn right small near"
##    f.write("turn right small near\n")
   
def Turn_right_mid_near(angle):
    steer = 383 - (round((angle - 60)/2) + 23)
    ser.write('0'+str(int(steer))+'00000150/')
##    print 'turn right mid near'
##    f.write("turn right mid near\n")
    
def Turn_right_big_near():
    ser.write('034200000150/')
##    print "turn right big near"
##    f.write("turn right big near\n")

def Turn_left_small_near(sign_car_side, angle):
    if sign_car_side == 'right':
        steer = 383 + (round((14 - angle)/2) + 5)
    elif sign_car_side == 'left':
        steer = 383 + (round((angle + 14)/2) + 5)
    ser.write('0'+str(int(steer))+'00000150/')
##    print "turn left small near"
##    f.write("turn left small near\n")

def Turn_left_mid_near(angle):
    steer = 383 + (round((angle - 22)/2) + 23)
    ser.write('0'+str(int(steer))+'00000150/')
##    print 'turn left mid near'
##    f.write("turn left mid near\n")
    
def Turn_left_big_near():
    ser.write('042400000150/')
##    print "turn left big near"
##    f.write("turn left big near\n")
 
def Turn_right_small_far(angle):
    steer = 383 - (round((angle - 16)/2) + 5)
    ser.write('0'+str(int(steer))+'00000190/')
##    print "turn right small far"
##    f.write("turn right small far\n")

def Turn_right_mid_far(angle):
    steer = 383 - (round((angle - 52)/ 2 ) + 23)
    ser.write('0'+str(int(steer))+'00000170/')
##    print 'turn right mid far'
##    f.write("turn right mid far\n")

def Turn_right_big_far():
    ser.write('034200000150/')
##    print "turn right big far"
##    f.write("turn right big far\n")
        
def Turn_left_small_far(sign_car_side, angle):
    if sign_car_side == 'right':
        steer = 383 + (round((4 - angle) / 2) + 5)
    elif sign_car_side == 'left':
        steer = 383 + (round((angle + 4) / 2) + 5)
    ser.write('0'+str(int(steer))+'00000190/')
##    print "turn left small far"
##    f.write("turn left small far\n")

def Turn_left_mid_far(angle):
    steer = 383 + (round((angle - 32)/ 2 ) + 23)
    ser.write('0'+str(int(steer))+'00000170/')
##    print 'turn left mid far'
##    f.write("turn left mid far\n")

def Turn_left_big_far():
    ser.write('042400000150/')
##    print "turn left big far"
##    f.write("turn left big far\n")

def Straight_fast():
    ser.write('038300000190/')
##    print "straight fast"
##    f.write("straight fast\n")

def Straight_slow():
    ser.write('038300000150/')
##    print "straight slow"
##    f.write("straight slow\n")

def Stop_control():
    ser.write('038300000000/')
##    print "stop control"
##    f.write("stop control\n")
    
def Back():
    ser.write('038301700000/')
##    print "back"
##    f.write("back\n")
    
def Back_angle(angle):
    steer = 383 + angle
    ser.write('0'+str(int(steer))+'01700000/')
##    print "back angle"
##    f.write("back angle\n")
    
def Turn_angle(angle):
    steer = 383 + angle
    ser.write('0'+str(int(steer))+'00000150/')
##    print "turn angle"
##    f.write("turn angle\n")


# the controlment
    
def Control(angle, sign_car_side, real_car_signal_distance): 
    if real_car_signal_distance < 1.2:
        if (sign_car_side == 'right' and angle < 24 and angle > 14):
            Straight_slow()
        elif (sign_car_side == 'right' and angle >= 24 and angle <= 60):
            Turn_right_small_near(angle)
        elif (sign_car_side == 'right' and angle > 60 and angle <= 96):
            Turn_right_mid_near(angle)
        elif (sign_car_side == 'right' and angle > 96 and angle <= 140):
            Turn_right_big_near()
        elif (sign_car_side == 'right' and angle > 140 and angle <= 165):
            Turn_right_big_near()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle > 165) or (sign_car_side == 'left' and angle > 135):
            Turn_right_big_near()
            sleep(0.4)
        elif (sign_car_side == 'right' and angle <= 14) or (sign_car_side == 'left' and angle <= 22):
            Turn_left_small_near(sign_car_side, angle)
        elif (sign_car_side == 'left' and angle > 22 and angle <= 58):
            Turn_left_mid_near(angle)
        elif (sign_car_side == 'left' and angle > 58 and angle <= 105):
            Turn_left_big_near()
        elif (sign_car_side == 'left' and angle > 105 and angle <= 135):
            Turn_left_big_near()
            sleep(0.2)
        else:
            Stop_control()   
    else:
        if (sign_car_side == 'right' and angle < 16 and angle > 4):
            Straight_fast()
        elif (sign_car_side == 'right' and angle >= 16 and angle <= 52):
            Turn_right_small_far(angle)
        elif (sign_car_side == 'right' and angle > 52 and angle <= 88):
            Turn_right_mid_far(angle)
        elif (sign_car_side == 'right' and angle > 88 and angle <= 140):
            Turn_right_big_far()
        elif (sign_car_side == 'right' and angle > 140 and angle <= 165):
            Turn_right_big_far()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle > 165) or (sign_car_side == 'left' and angle > 135):
            Turn_right_big_far()
            sleep(0.4)
        elif (sign_car_side == 'right' and angle <= 4) or (sign_car_side == 'left' and angle <= 32):
            Turn_left_small_far(sign_car_side, angle)
        elif (sign_car_side == 'left' and angle > 32 and angle <= 68):
            Turn_left_mid_far(angle)
        elif (sign_car_side == 'left' and angle > 68  and angle <= 105):
            Turn_left_big_far()
        elif (sign_car_side == 'left' and angle > 105  and angle <= 135):
            Turn_left_big_far()
            sleep(0.2)
        else:
            Stop_control()


def main():
##    global distance_list, sign_list

    # 相机初始化，串口设置为 USB 往串口写数据
    Initialize()
    
    error_count = 0
    out_count = 0
    sign_list = [(0,False)] * 30 
    distance_list = [random.random()*5] * 20 # 生成 100 以内的随机数
    
    while True:
##        time0 = time.time()
        rangefinder = 3.1 ##TA
        range_pic_ratio = 0.43 ##TA
        threshold = 248  ##TA
        capture = Snapshot() # 就拍了一张照片
        # flag, car_head, car_tail, center_sign, car_signal_distance, real_car_signal_distance, angle, sign_car_side = Calculate(capture, threshold, rangefinder, range_pic_ratio)
        while True:    
            Calculate(capture, threshold, rangefinder, range_pic_ratio)
            a = input('next? >(Y/N)')
            if a == 'N':
                break
main()
