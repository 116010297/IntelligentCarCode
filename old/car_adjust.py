import serial    #import serial module
ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
print 'success handler'
#try and exceptstructure are exception handler


def Turn_right_small_near(angle):
    str_angle = str(int(round((angle - 24)/2 + 87.5))).rjust(3,'0')
    s = '150000'+str_angle
    ser.write(s)
    print(s)

    # steer = 383 - (round((angle - 24)/2) + 5)
    # ser.write('0'+str(int(steer))+'00000150/')
    

   
def Turn_right_mid_near(angle):
    # steer = 383 - (round((angle - 60)/2) + 23)
    # ser.write('0'+str(int(steer))+'00000150/')
    str_angle = str(int(round((angle - 24)/2 + 87.5))).rjust(3,'0')
    ser.write('150000'+str_angle)
    print(s)
    
##    print 'turn right mid near'
##    f.write("turn right mid near\n")
    
def Turn_right_big_near():
    ser.write(255000130)
##    print "turn right big near"
##    f.write("turn right big near\n")

def Turn_left_small_near(sign_car_side, angle):
    if sign_car_side == 'right':
        steer = str(int(round((angle - 24)/2 + 87.5))).rjust(3,'0')
    elif sign_car_side == 'left':
        steer = str(int(round(angle/(-2) + 95))).rjust(3,'0')
    ser.write('170000'+steer)
##    print "turn left small near"
##    f.write("turn left small near\n")

def Turn_left_mid_near(angle):
    steer = 383 + (round((angle - 22)/2) + 23)
    ser.write('160000'+str(int(round(angle/(-2) + 95))).rjust(3,'0'))
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



    
def Control(angle, sign_car_side, real_car_signal_distance):  
    if real_car_signal_distance < 1.2:
        if (sign_car_side == 'right' and angle < 24 and angle > 14):
            Straight_slow()
        elif (sign_car_side == 'right' and angle >= 24 and angle <= 60):
            Turn_right_small_near(angle)
        elif (sign_car_side == 'right' and angle > 60 and angle <= 90):
            Turn_right_mid_near(angle)
        elif (sign_car_side == 'right' and angle > 90 and angle <= 140):
            Turn_right_big_near()
        elif (sign_car_side == 'right' and angle > 140 and angle <= 165):
            Turn_right_big_near()
            sleep(0.2)
        elif (sign_car_side == 'right' and angle > 165) #or (sign_car_side == 'left' and angle > 135):
            Turn_right_big_near()
            sleep(0.4)
        #elif (sign_car_side == 'right' and angle <= 14) or (sign_car_side == 'left' and angle <= 22):
            #Turn_left_small_near(sign_car_side, angle)
        elif (sign_car_side == 'left' and angle > 14 and angle <= 24):
            Straight_slow() 
        elif (sign_car_side == 'left' and angle > 24 and angle <= 60):
            Turn_left_mid_near(angle)
        elif (sign_car_side == 'left' and angle > 60 and angle <= 90):
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


def angleTest():
    while True:
        angle = int(raw_input('enter your angle here'))
        sign_car_side = raw_input('side')
        real_car_signal_distance = float(raw_input('distance'))
        Control(angle,sign_car_side,real_car_signal_distance)


try:
    angleTest() 
except:
  print 'exc'
  ser.close()

