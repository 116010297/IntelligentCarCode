import serial    #import serial module
ser = serial.Serial('/dev/ttyACM0', 9600,timeout=1)
print 'success handler'
#try and exceptstructure are exception handler
try:
  while 1:
    print 'runing...'
    ser.write('s') #writ a string to port
    response = ser.readall()#read a string from port
    print response
except:
  print 'exc'
  ser.close()
