import serial    #import serial module
import time

ser = serial.Serial('/dev/ttyUSB0',115200)
# a = raw_input()
print 'connected'

try:
  show()
except:
  print 'something wrong,please retry!'
  ser.close()

def read():
  response = ser.readall()
  print response

def stop():
  ser.write('000000000')
  read()

def forward():
  ser.write('120000000')
  read()

def backward():
  ser.write('000120000')
  read()


def turn_small_left():
  ser.write('000000080')
  read()

def turn_big_left():
  ser.write('000000050')
  read()


def turn_small_right():
  ser.write('000000100')
  read()

def turn_big_right():
  ser.write('000000130')
  read()

def show():
  # backforward
  forward()
  time.sleep(2)
  stop()

  time.sleep(1)
  # rudder
  turn_big_left()
  turn_big_right()
  turn_small_left()
  turn_small_right()

  time.sleep(1)

  turn_small_right()
  forward()
  time.sleep(2)
  stop()

  # loop
  while True:
    cmd = raw_input('Enter anycommand you like >> ')
    if cmd == 'Y':
      break
    else:
      serial.write(cmd)
      read()




