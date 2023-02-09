# --Imports--
import RPi.GPIO as GPIO
import time

# --Initialisation--
# Here i set numbers used to represent GPIO number and not Pin number
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --Variables--
Motor_R_EN1 = 6
Motor_R_EN2 = 5
Motor_EN = 13

Right = 1
Left = 0

# --Setup--
# I set GPIO pin 5 and 6 to outputs
GPIO.setup(Motor_R_EN1,GPIO.OUT)
GPIO.setup(Motor_R_EN2,GPIO.OUT)
GPIO.setup(Motor_EN,GPIO.OUT)

# --Motor Control--
def Motor_Control(Direction):
    # Just some simple starting code
    Current_Direction = Direction
    
    # Enable motor
    GPIO.output(Motor_EN,1)
    
    # Turn Right
    if(Current_Direction == Right):
        GPIO.output(Motor_R_EN1,1)
        GPIO.output(Motor_R_EN2,0)
        
    # Turn Left
    elif(Current_Direction == Left):
        GPIO.output(Motor_R_EN1,0)
        GPIO.output(Motor_R_EN2,1)

# --Main--
while(True):
    Motor_Control(Right)
    time.sleep(5)
    GPIO.output(Motor_EN,0)
    time.sleep(5)
    Motor_Control(Left)
    time.sleep(5)
    GPIO.output(Motor_EN,0)
    time.sleep(5)