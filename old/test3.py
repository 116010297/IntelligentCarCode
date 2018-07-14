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
##    f = open("log.txt","a")
    camera = PiCamera()
    camera.resolution = (640,544)
    camera.framerate = 60
    print 'Camera initialized...'
##    f.write('Camera initialized...\n')
    
    # ser = serial.Serial("/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_95635333231351C04281-if00",115200)
    # ser.write('038300000000/')
    # print 'Serial initialized...\n'
##    f.write('Serial initialized...\n\n')
    

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
    print 'photo',output_gray
    ret, thresh = cv2.threshold(output_gray,threshold,255,cv2.THRESH_BINARY)
    print 'thresh:',thresh
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

        print 'car-light vector',(car_signal_x,car_signal_y)
        print 'car vector',(car_direction_x, car_direction_y)


def main():
##    global distance_list, sign_list
    Initialize()
    
    error_count = 0
    out_count = 0
    sign_list = [(0,False)] * 30 
    distance_list = [random.random()*5] * 20 
    
    while True:
##        time0 = time.time()
        rangefinder = 3.1 ##TA
        range_pic_ratio = 0.43 ##TA
        threshold = 248  ##TA
        capture = Snapshot() #
        # flag, car_head, car_tail, center_sign, car_signal_distance, real_car_signal_distance, angle, sign_car_side = Calculate(capture, threshold, rangefinder, range_pic_ratio)   
        Calculate(capture, threshold, rangefinder, range_pic_ratio)
        a = raw_input('next? >(Y/N)')
        if a == 'N':
            break
        else:
            pass

main()
