import socket
import subprocess
import random
import time
import threading
import pygame
import pickle
s = socket.socket()
port = 6795
try:
    s.connect(('192.168.43.112', port))
    print(s.recv(1024).decode())
except:
    print("Error Establishing Connection")
pygame.init()
win = pygame.display.set_mode((500, 500))
pygame.display.set_caption("Anjel")
flag = False
x = 800
y = 800
width = 10
height = 10
roll = 0
pitch = 0
vel = 1
val = 5
speed = 1000
run = True
HEADERSIZE = 2

def sendingData():
    data = str(int(speed)) + " " + str(int(roll / 100)) + " " + str(int(pitch / 100)) + " \n"
    try:
        s.send(data.encode('utf-8'))
        print(data)
        #print(s.recv(1024).decode())
    except:
        #print(data)
        print("Sending or Receiving Error")

while run:
    # creates time delay of 10ms
    pygame.time.delay(10)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    # stores keys pressed
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT] and roll > -x:
        roll -= val
        sendingData()
    if keys[pygame.K_RIGHT] and roll < x:
        roll += val
        sendingData()
    if keys[pygame.K_UP] and pitch > -x:
        pitch -= val
        sendingData()
    if keys[pygame.K_DOWN] and pitch < x:
        pitch += val
        sendingData()
    if keys[pygame.K_w] and speed < 2000:
        speed += vel
        sendingData()
    if keys[pygame.K_s] and speed > 1000:
        speed -= vel
        sendingData()

    pygame.draw.rect(win, (255, 0, 0), (roll, pitch, width, height))
    pygame.display.update()


# closes the pygame window
pygame.quit()
    #time.sleep(0.2)



