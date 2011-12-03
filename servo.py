import serial

usbport = '/dev/tty.usbserial-A700fkJk'
ser = serial.Serial(usbport, 9600, timeout=1)

def move(servo, angle):

	if (0 <= angle <= 180):
		ser.write(chr(255))
		ser.write(chr(servo))
		ser.write(chr(angle))
	else:
		print "Servo angle must be an integer between 0 and 180.\n"