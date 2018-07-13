import serial
ser = serial.Serial('/dev/ttyUSB0',57600,timeout=1)
print 'success handler'
#try and exceptstructure are exception handler
try:
  while 1:
    a = raw_input('enter your code here:')
    ser.write(a) #writ a string to port
    response = ser.readall()#read a string from port
    print response
except:
  print 'exc'
  ser.close()