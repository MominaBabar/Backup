
from flask import Flask,Response,render_template
import json
from flask_socketio import SocketIO, send ,emit
import time
import threading
import argparse
import datetime
import imutils
import RPi.GPIO as GPIO          
from time import sleep
import mapping
import json
import os

outputFrame = None
lock = threading.Lock()
app = Flask(__name__)
app.config['SECRET_KEY'] = 'mysecret'
socketio = SocketIO(app)
   
time.sleep(0.1)

in1 = 22
in2 = 17
in3 = 27
in4 = 25
en4 = 18
en8 = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

GPIO.setup(en4,GPIO.OUT)
GPIO.setup(en8,GPIO.OUT)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)


p=GPIO.PWM(en4,1000)
p1=GPIO.PWM(en8,1000)

p.start(25)
p1.start(25)

p.ChangeDutyCycle(100)
p1.ChangeDutyCycle(100)

cleaning = False

def forward(sec):
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    time.sleep(sec)
    stop(0.3)
    #GPIO.cleanup()

def backward(sec):
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    time.sleep(sec)
    stop(0.3)
    #GPIO.cleanup()

def left(sec):
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.HIGH)
    time.sleep(sec)
    stop(0.3)
    #GPIO.cleanup()

def right(sec):
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    time.sleep(sec)
    stop(0.3)
    #GPIO.cleanup()

def stop(sec):
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    time.sleep(0.3)
    #GPIO.cleanup()



@app.route('/')
def index():
    return render_template('index.html')

############################connect#######
@socketio.on('connect')
def test_connect():
    print("Connecting..")
    emit('connection_response','Connection established')
    @socketio.on ('connection')
    def on_connection(c_res):
        print("connection made")
        #init()
        print(c_res)

@socketio.on('alarm')
def alarm():
    print("Alarm Reached.")
    if(cleaning==True):
        emit('connection_response','Cleaning Already in progress.')
    else:
        cleaning = True
        emit('connection_response','Cleaning started.')
        os.system('python mapping.py')

@socketio.on('start_cleaning')
def alarm():
    print("Starting Cleaning.")
    if(cleaning==True):
        emit('connection_response','Cleaning Already in progress.')
    else:
        cleaning = True
        emit('connection_response','Cleaning started.')
        os.system('python mapping.py')

@socketio.on('finish_cleaning')
def alarm():
    print("Cleaning Finished.")
    if(cleaning==True):
        cleaning = False
        emit('connection_response','Cleaning Finished.')
    else:
        emit('connection_response','Cleaning not started.')    

@socketio.on('cleaning_info')
def info():
    print("Starting Cleaning.")
    with open(os.path.join(os.getcwd(),'values.json') as f:
        data = json.load(f)
        print(data)
    if(cleaning==True):
        data["percent"] = 100
        emit('data',data)
    else:
        emit('data',data)

@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')

##########Controller Control ###############
    
@socketio.on('forward_con')
def on_forward_con(*args):
    print("forward")
    forward(0.5)


@socketio.on('right_con')
def on_right_con(*args):
    print("right")
    right(1.5)


@socketio.on('left_con')
def on_left_con(*args):
    print("left")
    left(1.5)

@socketio.on('back_con')
def on_back_con(*args):
    print("back")
    backward(0.5)

@socketio.on('stop_con')
def on_stop_con(*args):
    print("stop")
    stop(0.5)


if __name__ == '__main__':
    socketio.run(app,host='0.0.0.0',port=5090)
