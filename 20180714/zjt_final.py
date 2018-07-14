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

def Variance(l):
    ex = sum(l)/len(l)
    s = 0
    for i in l:
        s = s + (i - ex)**2
    return (s / len(l))**0.5


def Initialize():
    global camera,ser,f
    f = open("log.txt","a")
    camera = PiCamera()
    camera.resolution = (640,544)
    camera.framerate = 60
    print 'Camera initialized...'
    f.write('Camera initialized...\n')
    
    ser = serial.Serial('/dev/ttyUSB0',57600,timeout=1)
    # ser.write('038300000000/')
    print 'Serial initialized...\n'
    f.write('Serial initialized...\n\n')
    

def Snapshot():    
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture,'bgr', use_video_port = True)
    image = rawCapture.array
##    cv2.imwrite("rawcapture.jpg",image) 
    return image


def Calculate(image, threshold, rangefinder, range_pic_ratio):
    x_size = image.shape[1]
    x_pic_length = rangefinder/range_pic_ratio
    
    output_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(output_gray,threshold,255,cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours_OK = []

    for i in range(len(contours)):
        contours_OK.append((cv2.contourArea(contours[i]),contours[i]))
    
    contours_OK = sorted(contours_OK,key = lambda cons_ok:cons_ok[0],reverse = True)
    print len(contours_OK)
    if len(contours_OK) == 4:
        (x_car,y_car),radius_car = (None,None),None
        center_car = None
        center_sign = None
        radius_sign = None
        
        (x_sign,y_sign),radius_sign = cv2.minEnclosingCircle(contours_OK[0][1])
        x_sign = int(x_sign)
        y_sign = int(y_sign)
        center_sign = (int(x_sign),int(y_sign))
        radius_sign = int(radius_sign)
        contours_OK.pop(0)

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

        if min(distance1,distance2,distance3) == distance1:
            car_tail = (int((x1+x2)/2),int((y1+y2)/2))
            car_head = (x3,y3)
            
        elif min(distance1,distance2,distance3) == distance2:
            car_tail = (int((x1+x3)/2),int((y1+y3)/2))
            car_head = (x2,y2)
        else:
            car_tail = (int((x2+x3)/2),int((y2+y3)/2))
            car_head = (x1,y1)

        if car_tail!=None and car_head!=None:
            center_car = (int((car_head[0]+car_tail[0])/2),int((car_head[1]+car_tail[1])/2))
            radius_car = int(0.5*((car_head[0]-car_tail[0])**2+(car_head[1]-car_tail[1])**2)**0.5)

        x_car = center_car[0]
        y_car = center_car[1]
        car_signal_distance = ((x_car-x_sign)**2 + (y_car-y_sign)**2)**0.5
        real_car_signal_distance = car_signal_distance / x_size * x_pic_length
   
        car_direction_x = car_head[0] - car_tail[0]
        car_direction_y = car_head[1] - car_tail[1]
        car_signal_x = x_sign - x_car
        car_signal_y = y_sign - y_car
        cos_angle = (car_direction_x * car_signal_x + car_direction_y * car_signal_y) / ((car_direction_x**2 + car_direction_y**2)**0.5 * (car_signal_x**2 + car_signal_y**2)**0.5)
        angle = math.acos(cos_angle) * 180/math.pi
        
        cross_product = car_direction_x * car_signal_y - car_direction_y * car_signal_x
        
        if cross_product > 0:
            sign_car_side = 'right'
        elif cross_product < 0:
            sign_car_side = 'left'
        else:
            sign_car_side = 'left'
            
        print 'angle:',angle, 'sign_car_side:',sign_car_side,'real_car_signal_distance',real_car_signal_distance
        f.write('angle:'+str(angle) + 'sign_car_side:'+ str(sign_car_side) +'\n'+'real_car_signal_distance'+str(real_car_signal_distance))
        
##        cv2.imwrite("route.jpg",image)
        return (4, car_head, car_tail, center_sign, car_signal_distance, real_car_signal_distance, angle, sign_car_side)

    elif len(contours_OK) == 1:
        print 'Out of range!'
        f.write('Out of range!\n')
        return (1, None, None, None, None, None, None, None)
    
    else:
        print 'len(contours_OK): ', len(contours_OK)
        f.write('len(contours_OK): '+str(len(contours_OK))+'\n')
        print 'No target found!'
        f.write('No target found!\n')
        return (0, None, None, None, None, None, None, None)


def Turn_right_small_near(angle):
    str_angle = str(int(round((angle - 24)/1.6 + 95))).rjust(3,'0')
    s = '090000'+str_angle
    ser.write(s)

    # steer = 383 - (round((angle - 24)/2) + 5)
    # ser.write('0'+str(int(steer))+'00000150/')
    print 'turn right small near'
    f.write("turn right small near\n")
   
def Turn_right_mid_near(angle):

    str_angle = str(int(round((angle - 24)/1 + 103))).rjust(3,'0')
    s = '095000'+str_angle
    ser.write(s)

    # steer = 383 - (round((angle - 60)/2) + 23)
    # ser.write('0'+str(int(steer))+'00000150/')
    print 'turn right mid near'
    f.write("turn right mid near\n")
    
def Turn_right_big_near():

    ser.write('100000140')

    # ser.write('034200000150/')
    print "turn right big near"
    f.write("turn right big near\n")

def Turn_left_small_near(sign_car_side, angle):
    if sign_car_side == 'right':
        str_angle = str(int(round((angle - 24)/2.4 + 85))).rjust(3,'0')
    elif sign_car_side == 'left':
        str_angle = str(int(round(angle/(-2.9) + 90))).rjust(3,'0')

    s = '095000'+str_angle
    ser.write(s)
    print "turn left small near"
    f.write("turn left small near\n")

def Turn_left_mid_near(angle):

    str_angle = str(int(round((angle-24)/(-1) + 90))).rjust(3,'0')
                
    s = '100000'+str_angle
    ser.write(s)
    
    # steer = 383 + (round((angle - 22)/2) + 23)
    # ser.write('0'+str(int(steer))+'00000150/')
    print 'turn left mid near'
    f.write("turn left mid near\n")
    
def Turn_left_big_near():
    ser.write('110000042')
    print "turn left big near"
    f.write("turn left big near\n")
 
def Turn_right_small_far(angle):
    str_angle = str(int(round((angle - 24)/1.6 + 92))).rjust(3,'0')
                
    s = '110000'+str_angle
    ser.write(s)

    # steer = 383 - (round((angle - 16)/2) + 5)
    # ser.write('0'+str(int(steer))+'00000190/')
    print "turn right small far"
    f.write("turn right small far\n")

def Turn_right_mid_far(angle):
    str_angle = str(int(round((angle - 24)/1.6 + 90))).rjust(3,'0')
                
    s = '120000'+str_angle
    ser.write(s)

    # steer = 383 - (round((angle - 52)/ 2 ) + 23)
    # ser.write('0'+str(int(steer))+'00000170/')
    print 'turn right mid far'
    f.write("turn right mid far\n")

def Turn_right_big_far():
    ser.write('113000140')
    print "turn right big far"
    f.write("turn right big far\n")
        
def Turn_left_small_far(sign_car_side, angle):
    if sign_car_side == 'right':
        str_angle = str(int(round((angle - 24)/2 + 85))).rjust(3,'0')  
        # steer = 383 + (round((4 - angle) / 2) + 5)
    elif sign_car_side == 'left':
        str_angle = str(int(round(angle/(-2) + 87))).rjust(3,'0') 
    s = '110000'+str_angle
    ser.write(s)

    print "turn left small far"
    f.write("turn left small far\n")

def Turn_left_mid_far(angle):
    str_angle = str(int(round(angle/(-2) + 95))).rjust(3,'0') 
    # steer = 383 + (round((angle - 32)/ 2 ) + 23)
    s = '110000'+str_angle
    ser.write(s)
    
    print 'turn left mid far'
    f.write("turn left mid far\n")

def Turn_left_big_far():
    ser.write('100000040')

    # ser.write('042400000150/')
    print "turn left big far"
    f.write("turn left big far\n")

def Straight_fast():

    ser.write('105000092')
    print "straight fast"
    f.write("straight fast\n")

def Straight_slow():
    ser.write('090000088')
    print "straight slow"
    f.write("straight slow\n")

def Stop_control():
    ser.write('000000090')
    print "stop control"
    f.write("stop control\n")
    
def Back():
    ser.write('000100088')
    # ser.write('038301700000/')
    print "back"
    f.write("back\n")
    
def Back_angle(angle):
    steer = 90 - angle
    ser.write('000110'+str(int(steer)).rjust(3,'0'))
    print "back angle"
    f.write("back angle\n")
    
def Turn_angle(angle):
    steer = 90 - angle
    ser.write('000100'+str(int(steer)).rjust(3,'0'))
    print "turn angle"
    f.write("turn angle\n")

    
def Control(angle, sign_car_side, real_car_signal_distance):  
    if real_car_signal_distance < 1.2:
        if (sign_car_side == 'right' and angle < 16 and angle > 6):
            Straight_slow()
        elif (sign_car_side == 'right' and angle >= 16 and angle <= 60):
            Turn_right_small_near(angle)
        elif (sign_car_side == 'right' and angle > 60 and angle <= 90):
            Turn_right_mid_near(angle)
        elif (sign_car_side == 'right' and angle > 90 and angle <= 140):
            Turn_right_big_near()
        elif (sign_car_side == 'right' and angle > 140 and angle <= 165) or (sign_car_side == 'left' and angle > 113 and angle <= 121):
            Turn_right_big_near()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle > 165): #or (sign_car_side == 'left' and angle > 135)
            Turn_right_big_near()
            sleep(0.4)
        elif (sign_car_side == 'right' and angle <= 14) or (sign_car_side == 'left' and angle <= 14):
            Turn_left_small_near(sign_car_side, angle)
        elif (sign_car_side == 'left' and angle > 14 and angle <= 24):
            Straight_slow() 
        elif (sign_car_side == 'left' and angle > 24 and angle <= 60):
            Turn_left_mid_near(angle)
        elif (sign_car_side == 'left' and angle > 60 and angle <= 88):
            Turn_left_big_near()
            sleep(0.2)
        elif (sign_car_side == 'left' and angle > 105 and angle <= 135):
            Turn_left_big_near()
            sleep(0.2)
        else:
            Stop_control()   
    else:
        if (sign_car_side == 'right' and angle < 10 and angle > 4):
            Straight_fast()
        elif (sign_car_side == 'right' and 14 <= angle and angle <=20) or (sign_car_side == 'right' and 33 <= angle and angle <= 52):
            Turn_right_small_far(angle)
            # added
        elif (sign_car_side == 'right' and 20 < angle and angle < 33):
            Turn_right_big_far()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle > 52 and angle <= 88):
            Turn_right_mid_far(angle)
        elif (sign_car_side == 'right' and angle > 88 and angle <= 140):
            Turn_right_big_far()
            # add loc
        elif (sign_car_side == 'right' and angle > 140 and angle <= 165) or (sign_car_side == 'left' and angle > 113 and angle <= 123):
            Turn_right_big_far()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle > 165) or (sign_car_side == 'left' and angle > 135):
            Turn_right_big_far()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle <= 4) or (sign_car_side == 'left' and angle <= 35):
            Turn_left_small_far(sign_car_side, angle)
            sleep
        elif (sign_car_side == 'left' and angle > 35 and angle <= 68):
            Turn_left_mid_far(angle)
        elif (sign_car_side == 'left' and angle > 68  and angle <= 105):
            Turn_left_big_far()
        elif (sign_car_side == 'left' and angle > 105 and angle <= 135):
            Turn_left_big_far()
            sleep(0.3)
        else:
            Stop_control()

def main():
##    global distance_list, sign_list
    
    Initialize()
    error_count = 0
    out_count = 0
    sign_list = [(0,False)] * 30
    distance_list = [random.random()*5] * 20
    
    while True:
        sleep(0.13)
##        time0 = time.time()
        rangefinder = 3.1 ##TA
        range_pic_ratio = 0.43 ##TA
        threshold = 248  ##TA
        capture = Snapshot()
##        time1 = time.time()
        flag, car_head, car_tail, center_sign, car_signal_distance, real_car_signal_distance, angle, sign_car_side = Calculate(capture, threshold, rangefinder, range_pic_ratio)     
##        time2 = time.time()
        
        if (flag == 4 and angle!= None and sign_car_side!=None and real_car_signal_distance!= None):
            error_count = 0
            out_count = 0
            sign_list.pop(0)
            sign_list.append((sign_car_side, angle > 45))
            distance_list.pop(0)
            distance_list.append(real_car_signal_distance)

            if Variance(distance_list) < 0.05:
                Back_angle(random.randint(-15,15))
                sleep(1.4)
                distance_list = [random.random()*5] * 20
                print 'CRASH HANDLE...'
                f.write('CRASH HANDLE...\n')
                
            if sign_list.count(('right',True)) >= 27:
                Turn_angle(random.randint(-30,-15))
                sleep(0.1)
                sign_list = [(0,False)] * 30
                print 'ANGLE OPTIMIZED...'
                f.write('ANGLE OPTIMIZED...\n')
                 
            Control(angle,sign_car_side,real_car_signal_distance)
            
            print 'CONTROLLING CAR...'
            f.write('CONTROLLING CAR...\n')
            
        elif flag == 1:
            out_count = out_count + 1
            if out_count > 8:
                # Back()
                sleep(1.5)
                out_count = 0
                print 'BACK...'
                f.write('BACK...\n')
            
        else:
            error_count = error_count + 1
            if error_count > 5:
                Stop_control()
                error_count = 0
                print 'ERROR...'
                f.write('ERROR...\n')
                
##        time3 = time.time()
##        print
##        print time3 - time0        
##        print time3 - time2
##        print time2 - time1
##        print time1 - time0
##        print
##        f.write('\n')


main()
