#import wire
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
from adafruit_servokit import Servokit

#DECLARO VARIABLES GLOBALES
MIN_PULSE_WIDTH=       650
MAX_PULSE_WIDTH=       2350
FREQUENCY      =       50


#Instancio el Driver del controlador de servos
#Adafruit_PWMServorDriver pwm = Adafruit_PWMServorDriver();
pwm = adafruit_pca9685.PCA9685(i2c)
Kit =Servokit(channels=16)



#CONFIGURO EL SETUP
time.sleep(5)   
pwm.frecuency = FRECUENCY
GPIO.setmode(GPIO.BOAARD)
hand = pwm.channels[0]
hand = adafruit_motor.servo.Servo(0)   #cualquiera de las dos, mejor la de abajo
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3)
base = adafruit_motor.servo.Servo(4)
porwrist = adafruit_motor.servo.Servo(5)
potEbow = adafruit_motor.servo.Servo(6)
potshoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.begin()
pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)  
GPIO.setup(7,GPIO.IN)  # channel tiene que ser un pin valido en jetson



def moveMotor(controlIn,motorOut):
    pulse_wide, pulse_width, potVal = -7

    #potval = analogRead(controLIn);
    potVal = GPIO.input(controLIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096); 
    #pwm.setPWM(motorOut, 0, pulse_width);    #lectura en C
    pwm = GPIO.PWM(motorOut, 0, pulse_width)


While(True):
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)
    moveMotor(potShoulder, shoulder)
    moveMotor(potBase, base)
    pushButton = GPIO.input(7)
    if(pushButton == GPIO.LOW):
        pwm.setPWM(hand, 0, 180)                             
        print("Grab")
    else:
        pwm.setPWM(hand, 0, 90)                              
        print("Release")
 
 


GPIO.cleanup()


int potWrist = A3
int potElbow = A2                       
int potShoulder = A1
int potBase = A0
int hand = 11
int wrist = 12
int elbow = 13                        
int shoulder = 14
int base = 15



















