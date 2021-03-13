import os
import time
#Libraries
import RPi.GPIO as GPIO
import time
import dht11
import datetime
 
#GPIO Mode (BOARD / BCM)
GPIO.setwarnings(True)
GPIO.setmode(GPIO.BCM)


#Led Pins Configs
gate_red = 2 # RP3
gate_green = 4  # RP7

GPIO.setup(gate_red, GPIO.OUT)
GPIO.setup(gate_green, GPIO.OUT)
GPIO.output(gate_green, GPIO.LOW)
GPIO.output(gate_red, GPIO.LOW)

# Servo Motor Pin Config
servo = 3 #GPIO4
GPIO.setup(servo,GPIO.OUT)
pwm = GPIO.PWM(servo,50)


# Motor Pin
motor = 21
GPIO.setup(motor,GPIO.OUT)
GPIO.output(motor,GPIO.HIGH)

# read data using pin 14
instance = dht11.DHT11(pin=14) #RP8
 
#set GPIO Pins
GPIO_TRIGGER = 18  #RP12
GPIO_ECHO = 24   #RP 18
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 4
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent
 
def setangle(angle):
    duty = angle / 18 + 2
    GPIO.output(servo,True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo,False)
    pwm.ChangeDutyCycle(0)
    
    
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    try:
        while True:
            dist = distance()
            result = instance.read()
            print ("Measured Distance = %.1f cm" % dist)
            print("Measured Temperature = ",result.temperature,'C')
            if dist < 10 and result.is_valid() and result.temperature !=0 and result.temperature < 30:
                GPIO.output(gate_green, GPIO.HIGH)
                GPIO.output(gate_red, GPIO.LOW)
                pwm.start(angle_to_percent(0))
                #Go at 90°
                
                pwm.ChangeDutyCycle(angle_to_percent(90))
                time.sleep(5)
                GPIO.output(motor, GPIO.LOW)
                time.sleep(3)
                GPIO.output(motor, GPIO.HIGH)
                #Finish at 180°
                pwm.ChangeDutyCycle(angle_to_percent(0))
                #time.sleep(1)
            else:
                GPIO.output(gate_green, GPIO.LOW)
                GPIO.output(gate_red, GPIO.HIGH)
                 
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()