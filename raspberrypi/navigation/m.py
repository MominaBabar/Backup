#import RPi.GPIO as GPIO          
from time import sleep
import time
import sys
import numpy as np
import turtle
import subprocess
import os
import io
from PIL import Image

# turtle.color('red', 'yellow')
# turtle.begin_fill()
# while True:
#     turtle.forward(200)
#     turtle.left(170)
#     if abs(turtle.pos()) < 1:
#         break
# turtle.end_fill()
# turtle.getscreen().getcanvas().postscript(file='outputname.ps')
# ps = open('outputname.ps')
# turtle.done()
# #ps = self.canvas.postscript(colormode='color')
# img = Image.open(io.BytesIO(ps.encode('utf-8')))
# img.save('test.jpg')

psimage=Image.open('outputname.ps')
psimage.save('outputname.png')
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
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(in1,GPIO.OUT)
# GPIO.setup(in2,GPIO.OUT)
# GPIO.setup(in3,GPIO.OUT)
# GPIO.setup(in4,GPIO.OUT)

# GPIO.setup(en4,GPIO.OUT)
# GPIO.setup(en8,GPIO.OUT)
    

# GPIO.setup(F_TRIG,GPIO.OUT)
# GPIO.setup(F_ECHO,GPIO.IN)
# GPIO.setup(L_TRIG,GPIO.OUT)
# GPIO.setup(L_ECHO,GPIO.IN)


# GPIO.output(in1,GPIO.LOW)
# GPIO.output(in2,GPIO.LOW)
# GPIO.output(in3,GPIO.LOW)
# GPIO.output(in4,GPIO.LOW)


# GPIO.output(F_TRIG, False)
# GPIO.output(L_TRIG, False)

# p=GPIO.PWM(en4,1000)
# p1=GPIO.PWM(en8,1000)

# p.start(25)
# p1.start(25)

# p.ChangeDutyCycle(100)
# p1.ChangeDutyCycle(100)

start = time.time()

PERIOD_OF_TIME = 180 # 5min

class ROBOT():

    def __init__(self,direction,rows,cols):
        self.direction = "North"
        self.rows = rows
        self.cols = cols
        self.map = np.zeros((self.rows,self.cols))
        self.starting_position = [self.rows//2,self.cols//2]
        self.current_position = [self.rows//2,self.cols//2]
        self.map_formed = False
        self.map[self.current_position[0],self.current_position[1]] = 3
        self.save_map()
        print("[+] Starting position: ",self.current_position)
        self.print_map()

    def save_map(self):
        print("Saving Map")
        a_file = open("map.txt", "w")
        for row in self.map:
            np.savetxt(a_file, row)
        a_file.close()
    
    def reset_map(self):
        self.map = np.zeros((self.rows,self.cols))
        self.starting_position = [self.rows//2,self.cols//2]
        self.current_position = [self.rows//2,self.cols//2]
        print("Map Reset.")

    def load_map(self):
        print("Loading Map..")
        original_array = np.loadtxt("map.txt").reshape(self.rows, self.cols)
        self.map = original_array
    
    def aload_map(self):
        print("Loading Map..")
        original_array = np.loadtxt("temp.txt").reshape(self.rows, self.cols)
        self.map = original_array

    def add_padding(self):
        row_to_be_added = np.zeros((1,self.cols))
        result = np.vstack ((row_to_be_added,self.map) )
        result = np.vstack ((result,row_to_be_added) )
        self.map = result
        self.rows +=2
        self.starting_position[0]+=1

    def follow_map(self):
        self.aload_map()
        self.add_padding()
        self.current_position = self.starting_position
        self.direction = "NORTH"
        print("[+] Starting position: ",self.current_position)
        self.make_area()
        self.map[self.current_position[0],self.current_position[1]]=7
        t = 0
        while(self.complete()==False and t<8):
            t+=1
            if(self.direction=="NORTH"):
                if(self.map[self.current_position[0],self.current_position[1]-1]==6):
                    print("left")
                    self.turn_robot("left")
                    self.current_position[1]-=1
                elif(self.map[self.current_position[0]-1,self.current_position[1]]==6):
                    self.current_position[0]-=1
                    print("forward")
                elif(self.map[self.current_position[0],self.current_position[1]+1]==6):
                    self.current_position[1]=+1
                    print("right")
                    self.turn_robot("right")
                self.map[self.current_position[0],self.current_position[1]]=7

            if(self.direction=="EAST"):
                print("kkk")
                # if(self.map[self.current_position[0],self.current_position[1]-1]==6):
                #     print("left")
                #     self.turn_robot("left")
                #     self.current_position[1]-=1
                # elif(self.map[self.current_position[0]-1,self.current_position[1]]==6):
                #     self.current_position[0]-=1
                #     print("forward")
                # elif(self.map[self.current_position[0],self.current_position[1]+1]==6):
                #     self.current_position[1]=+1
                #     print("right")
                #     self.turn_robot("right")

            self.print_map()    
                
            
        #     print("forward")
        # if(self.map[self.current_position[0],self.current_position[1]]==2):
        #     print("left")
        # if(self.map[self.current_position[0],self.current_position[1]]==4):
        #     print("right")
    def complete(self):
        for i in range(0,self.rows):
            for j in range(0,self.cols):
                if(self.map[i,j]==6):
                    return False
        
        return True

    def make_area(self):
        self.print_map()
        #print(self.rows,self.cols)
        for i in range(0,self.rows-1):
            for j in range(0,self.cols):
                if(self.map[i,j] == 0 and self.map[i,j-1]!=0 and self.map[i-1,j]!=0 and self.map[i-1,j-1]!=0):
                    self.map[i,j] = 6 
        
        for i in range(0,self.rows):
            for j in range(0,self.cols):
                if(self.map[i,j]!=0):
                    self.map[i,j] = 6  

    def forward_distance(self):
        print("Forward Distance Measurement In Progress")
        sleep(2)

        # GPIO.output(F_TRIG, True)
        # time.sleep(0.00001)
        # GPIO.output(F_TRIG, False)

        # while GPIO.input(F_ECHO)==0:
        #     pulse_start = time.time()

        # while GPIO.input(F_ECHO)==1:
        #     pulse_end = time.time()

        # pulse_duration = pulse_end - pulse_start

        # distance = pulse_duration * 17150
        # distance = round(distance, 2)
        # print("Forward Distance:",distance,"cm")
        # #GPIO.cleanup()
        # return distance

    def left_distance(self):
        print("Left Distance Measurement In Progress")
        sleep(2)

        # GPIO.output(L_TRIG, True)
        # time.sleep(0.00001)
        # GPIO.output(L_TRIG, False)

        # while GPIO.input(L_ECHO)==0:
        #     pulse_start = time.time()

        # while GPIO.input(L_ECHO)==1:
        #     pulse_end = time.time()
        # pulse_duration = pulse_end - pulse_start

        # distance = pulse_duration * 17150
        # distance = round(distance, 2)
        # print("Left Distance:",distance,"cm")
        # #GPIO.cleanup()
        # return distance
    
    def right_distance(self):
        print("Right Distance Measurement In Progress")
        sleep(2)

        # GPIO.output(R_TRIG, True)
        # time.sleep(0.00001)
        # GPIO.output(R_TRIG, False)

        # while GPIO.input(R_ECHO)==0:
        #     pulse_start = time.time()

        # while GPIO.input(R_ECHO)==1:
        #     pulse_end = time.time()
        # pulse_duration = pulse_end - pulse_start

        # distance = pulse_duration * 17150
        # distance = round(distance, 2)
        # print("Left Distance:",distance,"cm")
        # #GPIO.cleanup()
        # return distance
    
    def update_map(self, move):
        if(move=="forward"):
            if(self.direction=="North"):
                self.current_position[0]-=1
            elif(self.direction=="East"):
                self.current_position[1]+=1
            elif(self.direction=="South"):
                self.current_position[0]+=1
            elif(self.direction=="West"):
                self.current_position[1]-=1
            print(self.map[self.current_position[0],self.current_position[1]])
            if(self.map[self.current_position[0],self.current_position[1]]==3):
                print("Starting point reached. Map finished.")
                self.save_map()
                self.map_formed = True
                return
            self.map[self.current_position[0],self.current_position[1]] = 1
        elif(move=="left"):
            self.map[self.current_position[0],self.current_position[1]] = 4
            self.turn_robot("left")
        elif(move=="right"):
            self.map[self.current_position[0],self.current_position[1]] = 2
            self.turn_robot("right")

    def turn_robot(self,move):
        if(move=="left"):
            if(self.direction=="East"):
                self.direction = "North"
            elif(self.direction=="South"):
                self.direction = "East"
            elif(self.direction=="West"):
                self.direction = "South"
            elif(self.direction=="North"):
                self.direction = "West"
        elif(move=="right"):
            if(self.direction=="East"):
                self.direction = "South"
            elif(self.direction=="South"):
                self.direction = "West"
            elif(self.direction=="West"):
                self.direction = "North"
            elif(self.direction=="North"):
                self.direction = "East"

    def forward(self,sec):
        print("forward")            
        # GPIO.output(in1,GPIO.LOW)
        # GPIO.output(in2,GPIO.HIGH)
        # GPIO.output(in3,GPIO.HIGH)
        # GPIO.output(in4,GPIO.LOW)
        # sleep(sec)
        # self.stop(0.3)
        #GPIO.cleanup()

    def backward(self,sec):
        print("backward")
        # GPIO.output(in1,GPIO.HIGH)
        # GPIO.output(in2,GPIO.LOW)
        # GPIO.output(in3,GPIO.LOW)
        # GPIO.output(in4,GPIO.HIGH)
        # sleep(sec)
        # self.stop(0.3)
        #GPIO.cleanup()

    def left(self,sec):
        print("left")   
        # GPIO.output(in1,GPIO.LOW)
        # GPIO.output(in2,GPIO.HIGH)
        # GPIO.output(in3,GPIO.HIGH)
        # GPIO.output(in4,GPIO.HIGH)
        # sleep(sec)
        # self.stop(0.3)


    def back_right(self,sec):
        print("back right")
        # GPIO.output(in1,GPIO.HIGH)
        # GPIO.output(in2,GPIO.HIGH)
        # GPIO.output(in3,GPIO.LOW)
        # GPIO.output(in4,GPIO.HIGH)
        # sleep(sec)
        # self.stop(0.3)
        #GPIO.cleanup()

    def back_left(self,sec):
        print("back left")
        # GPIO.output(in1,GPIO.HIGH)
        # GPIO.output(in2,GPIO.LOW)
        # GPIO.output(in3,GPIO.HIGH)
        # GPIO.output(in4,GPIO.LOW)
        # sleep(sec)
        # self.stop(0.3)
        #GPIO.cleanup()

    def right(self,sec):
        print("right")
        # GPIO.output(in1,GPIO.HIGH)
        # GPIO.output(in2,GPIO.HIGH)
        # GPIO.output(in3,GPIO.HIGH)
        # GPIO.output(in4,GPIO.LOW)
        # sleep(sec)
        # self.stop(0.3)
        #GPIO.cleanup()

    def stop(self,sec):
        print("stop")
        # GPIO.output(in1,GPIO.LOW)
        # GPIO.output(in2,GPIO.LOW)
        # GPIO.output(in3,GPIO.LOW)
        # GPIO.output(in4,GPIO.LOW)
        # sleep(sec)
        #GPIO.cleanup()

    def cleanup(self):
        print("Cleaning up")
        #GPIO.cleanup()
    
    def print_map(self):
        print("current direction: ", self.direction)
        print(self.map)

if __name__ == '__main__':
    print("Forming map...")
    print("\n")
    print("--------------------------------------------------")
    print("Press: w-forward s-backward a-left d-right x-stop q-quit")
    print("--------------------------------------------------")
    print("\n")
    robot = ROBOT("NORTH",20,20)
    while(1):
        choice = int(input("--------\npress 1 to form map\npress 2 to use map to traverse.\npress 3 to quit\n"))
        if(choice==1):
            robot.reset_map()
            while(1):
                x=input("Enter direction: ")
                sec = 0.5
                rsec = 2
                lsec = 2
                if x=="x":
                    print("stop")
                    robot.stop(sec)
                elif x=="s":
                    print("backward")
                    robot.backward(sec)
                elif x=="a":
                    print("left")
                    robot.left(lsec)
                    robot.update_map("left")
                elif x=="c":
                    print("back right")
                    robot.back_right(rsec)
                elif x=='w':
                    print("forward")
                    robot.forward(sec)
                    robot.update_map("forward")
                elif x=='z':
                    print("back left")
                    robot.back_left(lsec)
                elif x=='d':
                    print("right")
                    robot.right(rsec)
                    robot.update_map("right")
                elif x=='q':
                    print("ending program...")
                    robot.save_map()
                    robot.cleanup()
                    robot.load_map()
                    break
                else:
                    print("<<<  wrong data  >>>")
                    print("please enter the defined data to continue.....")
                    print("\n")
                    print("--------------------------------------------------")
                    print("Press: w-forward s-backward a-left d-right x-stop q-quit")
                    print("--------------------------------------------------")
                    print("\n")
                
                if(robot.map_formed==True):
                    break
                robot.print_map()
        if(choice==2):
            if(robot.map_formed==True):
                robot.follow_map()
            else:
                print("No map exists. first form map then come here")
                robot.follow_map()
        if(choice==3):
            sys.exit(0)


