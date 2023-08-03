# This program runs on the FPV car from a Winlab summer 2022 internship project
# Authors: A. Martin and B. Yu 
# the code that runs on the laptop is at:
# https://github.com/canqda/FPV-Car-Code/blob/main/FPV-Client-Side-Code.py
​
# The final project presentation is at:
# https://docs.google.com/presentation/d/11I4LnqWGcEINTxNEV6wiZwQ__Nb-fthx4U4ywqaa71Q/edit?usp=sharing
​
​
import re
import RPi.GPIO as GPIO  # control motor board through GPIO pins
import socket
import threading
from threading import Thread
import cv2
import time
import numpy as np
​
# If IN1Rear=True and IN2Rear=False right motor move forward, If IN1Rear=False,IN2Rear=True right motor move backward,in other cases right motor stop
IN1Rear = 16  # GPIO23 to IN1 Rear-right wheel direction
IN2Rear = 18  # GPIO24 to IN2 Rear-right wheel direction
​
# If IN3Rear=True and IN3Rear=False left motor move forward, If IN3Rear=False,IN4Rear=True left motor move backward,in other cases left motor stop
IN3Rear = 13  # GPIO27 to IN3 Rear-left wheel direction
IN4Rear = 15  # GPIO22 to IN4 Rear-left wheel direction
​
# ENA/ENB are PWM(analog) signal pin which control the speed of right/left motor through GPIO ChangeDutyCycle(speed) function
ENA = 12  # GPIO18 to ENA PWM SPEED of rear right motor
ENB = 33  # GPIO13 to ENB PWM SPEED of rear left motor
​
# If IN1Front=True and IN2Front=False right motor move forward, If IN1Front=False,IN2Front=True right motor move backward,in other cases right motor stop
IN1Front = 40  # GPIO21 to IN1 Front Model X right wheel direction
IN2Front = 38  # GPIO20 to IN2 Front Model X right wheel direction
​
# If IN3Front=True and IN3Front=False left motor move forward, If IN3Front=False,IN4Front=True left motor move backward,in other cases left motor stop
IN3Front = 36  # GPIO16 to IN3 Front Model X left wheel direction
IN4Front = 32  # GPIO12 to IN4 Front Model X left wheel direction
​
# initialize GPIO pins, tell OS which pins will be used to control Model-Pi L298N board
GPIO.setmode(GPIO.BOARD)
GPIO.setup(IN1Rear, GPIO.OUT)
GPIO.setup(IN2Rear, GPIO.OUT)
GPIO.setup(IN3Rear, GPIO.OUT)
GPIO.setup(IN4Rear, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1Front, GPIO.OUT)
GPIO.setup(IN2Front, GPIO.OUT)
GPIO.setup(IN3Front, GPIO.OUT)
GPIO.setup(IN4Front, GPIO.OUT)
GPIO.output(ENA, True)
GPIO.output(ENB, True)
​
def rr_ahead(speed):
    GPIO.output(IN1Rear, True)
    GPIO.output(IN2Rear, False)
​
def rl_ahead(speed):
    GPIO.output(IN3Rear, True)
    GPIO.output(IN4Rear, False)
​
def rr_back(speed):
    GPIO.output(IN2Rear, True)
    GPIO.output(IN1Rear, False)
​
def rl_back(speed):
    GPIO.output(IN4Rear, True)
    GPIO.output(IN3Rear, False)
​
def fr_ahead(speed):
    GPIO.output(IN1Front, True)
    GPIO.output(IN2Front, False)
​
def fl_ahead(speed):
    GPIO.output(IN3Front, True)
    GPIO.output(IN4Front, False)
​
def fr_back(speed):
    GPIO.output(IN2Front, True)
    GPIO.output(IN1Front, False)
​
def fl_back(speed):
    GPIO.output(IN4Front, True)
    GPIO.output(IN3Front, False)
​
def go_ahead(speed):
    rl_ahead(speed)
    rr_ahead(speed)
    fl_ahead(speed)
    fr_ahead(speed)
​
def go_back(speed):
    rr_back(speed)
    rl_back(speed)
    fr_back(speed)
    fl_back(speed)
​
def turn_right(speed):
    rl_ahead(speed)
    rr_back(speed)
    fl_ahead(speed)
    fr_back(speed)
​
def turn_left(speed):
    rr_ahead(speed)
    rl_back(speed)
    fr_ahead(speed)
    fl_back(speed)
​
def shift_left(speed):
    fr_ahead(speed)
    rr_back(speed)
    rl_ahead(speed)
    fl_back(speed)
​
def shift_right(speed):
    fr_back(speed)
    rr_ahead(speed)
    rl_back(speed)
    fl_ahead(speed)
​
def upper_right(speed):
    rr_ahead(speed)
    fl_ahead(speed)
​
def lower_left(speed):
    rr_back(speed)
    fl_back(speed)
​
def upper_left(speed):
    fr_ahead(speed)
    rl_ahead(speed)
​
def lower_right(speed):
    fr_back(speed)
    rl_back(speed)
​
def stop_car():
    GPIO.output(IN1Rear, False)
    GPIO.output(IN2Rear, False)
    GPIO.output(IN3Rear, False)
    GPIO.output(IN4Rear, False)
    GPIO.output(IN1Front, False)
    GPIO.output(IN2Front, False)
    GPIO.output(IN3Front, False)
    GPIO.output(IN4Front, False)
    
HEADER = 100000
PORT = 5050
#SERVER = socket.gethostbyname(socket.gethostname())
SERVER = "10.61.1.234"
#SERVER = socket.gethostname()
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
​
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#server.connect(("8.8.8.8",5050))
#SERVER = server.getsockname()[0]
server.bind(ADDR)
​
def handle_client(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")
​
    connected = True
    while connected:
        msg_length = conn.recv(HEADER).decode(FORMAT)
        if msg_length:
            print('msg length is',msg_length)
            #if msg_length /= "1":
            #    msg_length = "1"
            msg_length = 1
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)
            print(msg)
            if msg == "s":
                go_ahead(100)
            if msg == "w":
                go_back(100)
            if msg == "d":
                shift_left(100)
            if msg == "a":
                shift_right(100)
            if msg == "e":
                turn_left(1)
            if msg == "q":
                turn_right(1)
            if msg == "g":
                upper_left(100)
            if msg == "f":
                upper_right(100)
            if msg == "t":
                lower_left(100)
            if msg == "r":
                lower_right(100)
            if msg == "i":
                stop_car()
            if msg == "k":
                stop_car()
                connected = False
            print(f"[{addr}] {msg}")
    conn.close()
def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")
​
def sendData():
    print("In send data")
    HEADER = 64
    PORT = 5051
    SERVER = "10.61.2.68"
    ADDR = (SERVER, PORT)
    FORMAT = 'utf-8'
    DISCONNECT_MESSAGE = "!DISCONNECT"
​
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #server.connect(("8.8.8.8",5050))
    #SERVER = server.getsockname()[0]
    client.connect(ADDR)
    
    def send(msg):
        print("In send")
        #print("msg being sent is",msg)
        client.send(msg)
​
​
    cam = cv2.VideoCapture(0)
   # cv2.namedWindow("test")
​
    img_counter = 0
​
    last = time.time() 
    while True:
        # read a frame
        print("camera while loop")
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        img_counter = img_counter + 1 
​
        #endcode the frame to jpeg as bytes 
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        image_bytes = cv2.imencode('.jpg', frame, encode_param)[1].tobytes()
        #imgae_byte = image_bytes
        lenbytes = len(image_bytes)
        print(lenbytes)
        send(image_bytes)
        #client.sent(image_bytes)
​
        
​
        """
        #decode the jpg into an opencv frame 
        img = np.frombuffer(image_bytes,dtype=np.uint8)
        img2 = cv2.imdecode(img,cv2.IMREAD_UNCHANGED)
​
        # show the image for waitkey millisec 
        cv2.imshow("test", img2)
        k = cv2.waitKey(1)
        """
        # throw control back to the operating system
        time.sleep(0.005)
        """
        now = time.time()
        diff = now-last 
        print("Got frame # %i sec = %s len=%s " % (img_counter,diff,lenbytes))
        last = now 
        """
    cam.release()
    
        
​
t1 = Thread(target=start)
t2 = Thread(target=sendData)
​
t1.start()
t2.start()
​
​
print("[STARTING] server is starting... ")
start()
​
GPIO.cleanup()
