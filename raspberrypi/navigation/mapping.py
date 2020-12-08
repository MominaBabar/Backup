
import RPi.GPIO as GPIO          
from time import sleep
import time
import sys
#motor pins for movement
in1 = 22
in2 = 17
in3 = 27
in4 = 25
en4 = 18
en8 = 5

#ultrasonic pins
F_TRIG = 23
F_ECHO = 24
L_TRIG = 26
L_ECHO = 19
R_TRIG = 13  #grey
R_ECHO = 16 # yellow

print("Initializing...")
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

GPIO.setup(en4,GPIO.OUT)
GPIO.setup(en8,GPIO.OUT)
    

GPIO.setup(F_TRIG,GPIO.OUT)
GPIO.setup(F_ECHO,GPIO.IN)
GPIO.setup(L_TRIG,GPIO.OUT)
GPIO.setup(L_ECHO,GPIO.IN)


GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)


GPIO.output(F_TRIG, False)
GPIO.output(L_TRIG, False)

p=GPIO.PWM(en4,1000)
p1=GPIO.PWM(en8,1000)

p.start(25)
p1.start(25)

p.ChangeDutyCycle(100)
p1.ChangeDutyCycle(100)

start = time.time()

PERIOD_OF_TIME = 180 # 5min

class ROBOT():

    def __init__(self,direction):
        self.direction = direction 

    def forward_distance():
        print("Forward Distance Measurement In Progress")
        sleep(2)

        GPIO.output(F_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(F_TRIG, False)

        while GPIO.input(F_ECHO)==0:
            pulse_start = time.time()

        while GPIO.input(F_ECHO)==1:
            pulse_end = time.time()      

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150
        distance = round(distance, 2)
        print("Forward Distance:",distance,"cm")
        #GPIO.cleanup()
        return distance

    def left_distance():
        print("Left Distance Measurement In Progress")
        sleep(2)

        GPIO.output(L_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(L_TRIG, False)

        while GPIO.input(L_ECHO)==0:
            pulse_start = time.time()

        while GPIO.input(L_ECHO)==1:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150
        distance = round(distance, 2)
        print("Left Distance:",distance,"cm")
        #GPIO.cleanup()
        return distance

        
    def forward(sec):
        print("forward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
        sleep(sec)
        #GPIO.cleanup()

    def backward(sec):
        print("backward")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.HIGH)
        sleep(sec)
        #GPIO.cleanup()

    def left(sec):
        print("left")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.HIGH)
        sleep(sec)


    def back_right(sec):
        print("back right")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.HIGH)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.HIGH)
        sleep(sec)
        #GPIO.cleanup()

    def back_left(sec):
        print("back left")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
        sleep(sec)
        #GPIO.cleanup()

    def right(sec):
        print("right")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.HIGH)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
        sleep(sec)
        #GPIO.cleanup()

    def stop(sec):
        print("stop")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.LOW)
        sleep(sec)
        #GPIO.cleanup()
    def cleanup():
        GPIO.cleanup()


if __name__ == '__main__':
    print("Forming map...")
    print("\n")
    print("--------------------------------------------------")
    print("Press: w-forward s-backward a-left d-right x-stop q-quit")
    print("--------------------------------------------------")
    print("\n")    
    robot = Robot("NORTH")
    while(1):
        x=input("Enter direction: ")
        sec = 0.5
        rsec = 1
        lsec = 1.5
        print(x)
        if x=="x":
            print("stop")
            robot.stop(sec)
        elif x=="s":
            print("backward")
            robot.backward(sec)
        elif x=="z":
            print("left")
            robot.left(sec)
        elif x=="c":
            print("back right")
            robot.back_right(rsec)
        elif x=='w':
            print("forward")
            robot.forward(sec)
        elif x=='a':
            print("back left")
            robot.back_left(lsec)
        elif x=='d':
            print("right")
            robot.right(rsec)
        elif x=='q':
            print("ending program...")
            robot.cleanup()
            break
        else:
            print("<<<  wrong data  >>>")
            print("please enter the defined data to continue.....")
            print("\n")
            print("--------------------------------------------------")
            print("Press: w-forward s-backward a-left d-right x-stop q-quit")
            print("--------------------------------------------------")
            print("\n") 
        
        