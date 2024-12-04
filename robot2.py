#Aqui poneis el Docstring que querais
"""
  Programación de un brazo robótico realizado en c++ y convertido a python.Este trabajo esta sacado de https://www.youtube.com/watch?v=AIsVlgopqJc&t=794sque es la parte 1 y https://www.youtube.com/watch?v=OiQKw0lZ5Rw&t=18s que es la parte 2.
  Lo primero de todo es pasar el codigo original de brazo robótico de c++ a python debido a que se utilizará una jetson nano.

  Los pasos en orden serian:
  Importar las bibliotecas.
  Definir las variables.
  Asignar los pines de entrada de la Jetson nano.
  Configurar el setup y relacionar los motores con los pines de entrada
  Asignar los motores con el potenciometro.
  Configurar el cierre y la apertura de la pinza.
  
  Por último hacemos una revisión del código para asegurarse de que no tenga fallos.
"""
#import Wire
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL,board.SDA)
from adafruit_servokit import Servokit


#Declaro variables globales
MIN_PULSE_WIDTH  =  650
MAX_PULSE_WIDTH  =  2350
FREQUENCY        =  50


#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm = adafruit_pca9685.PCA9685("i2C")
kit = ServoKit(channels=16)
potWrist = GPIO.input(19)
potElbow = GPIO.input(21)                      #//Assign Potentiometers to pins on Arduino Uno
potShoulder = GPIO.input(23)
potBase = GPIO.input(29)

#Configuro el SetUP
time.sleep(5)                         #<---  So I have time to get controller to starting position
pca.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)

hand = adafruit_motor.servo.Servo(0)  
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)                  #Set Gripper to 90 degrees (Close Gripper) X en Jetson
pwm.begin()
GPIO.setup(13, GPIO.IN)    # channel tiene que ser un pin válido en jetson


def moveMotor(controlIn, motorOut):
  """
    Aqui definimos lo que hace la función.
     
    Args:
      controlIn (int):El pin GPIO recibe el valor del potenciometro y se obtiene el valor mediante el pin.
      motorOut (int):Este es el pin de salida y se utiliza para enviar la señal PWM al motor.
    
    Returns:
      Esta función no tiene valor,solo recibe el valor que le da el potenciometro para colocar el robot en la posición que indique el valor.
      """
  pulse_wide, pulse_width, potVal = -7
  
#  potVal = analogRead(controlIn);                                                   #//Read value of Potentiometer
  potVal = GPIO.input(controlIN)
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                #//Map Potentiometer position to Motor
  
#  pwm.setPWM(motorOut, 0, pulse_width);
  pwm = GPIO.PWM(motorOut, 0, pulse_width)
 
 
while (True):  
  moveMotor(potWrist, wrist)
  moveMotor(potElbow, elbow)                              # //Assign Motors to corresponding Potentiometers
  moveMotor(potShoulder, shoulder)
  moveMotor(potBase, base)
  pushButton = GPIO.input(13)      
  if(pushButton == GPIO.LOW):

    pwm.setPWM(hand, 0, 180);                             #//Keep Gripper closed when button is not pressed
    print("Grab")
  
  else:
  
    pwm.setPWM(hand, 0, 90);                              #//Open Gripper when button is pressed
    print("Release")

GPIO.cleanup()





