HOME_FOLDER                 = "/home/rydpiservo/expctl/src/expctl/servers/servoaligner/"
DEVICENAME_LIST             = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2']     # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
SERVO_SPEED = 2000
SERVO_ACC = 90
#
sts3032_dict={0:[3,'1x'], 1:[4,'1y'], 2:[1,'2x'], 3:[2,'2y'], 4:[5,'3x'], 5:[6,'3y'], 6:[7,'4x'],7:[8,'4y'] 
              #,8:[9, 'UV1x'], 9:[10, 'UV1y'], 10:[11, 'UV2x'], 11:[12, 'UV2y']
              } # dict {index:[ID, servo name]}
