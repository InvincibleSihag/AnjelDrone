import time
import socket
from threading import Thread
import serial
import pickle
global flag
flag = False
flags = [False]
port = 6795
droneControldata = ["","",""]
data = ["",""]

def sendSignal(c,ser,flags):
    while True:
        try:
            if ser.in_waiting > 0:
                print(ser.read(15).decode())
                ser.reset_output_buffer()
                time.sleep(1)
        except:
            #print("Arduino Error")
            continue
        if(flags[0]==False):
            c.close()
            break
class ClientThread(Thread): 
    def __init__(self,c,ser): 
        Thread.__init__(self)
        #print "[+] New server socket thread started for " + ip + ":" + str(port) 
 
    def run(self):
        HEADERSIZE = 4
        newMess = True
        #time.sleep(0.1)
        print("New Thread Started")
        try:
            while True:
                ser.reset_input_buffer()
                temp = ""
                arduinoData = ""
                temp=c.recv(1024).decode()
                #print(temp)
                i=0
                while(temp[i] != "\n"):
                    arduinoData += temp[i]
                    i += 1
                arduinoData += "\n"
                #print(arduinoData)
                #print(len(arduinoData))
                if(len(arduinoData)>7 and len(arduinoData)<12):
                    ser.write(arduinoData.encode('utf-8'))
                
                if(temp==""):
                    c.close()
                    break
        except:
            c.close()
            
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.bind(('',port))
s.listen()
print(socket.gethostname())

while(True):
    if(flags[0]==False):
        print("Ready For new connection")
        c, addr = s.accept()
        print("Got connection from - ", addr)
        c.send(("You are Connected to ANJel").encode('utf-8'))
        ser = serial.Serial('/dev/ttyUSB0', 115200,timeout=0)
        ser.flush()
        newthread = ClientThread(c,ser) 
        newthread.start()
        flags[0] = True
        sendThread = Thread(target=sendSignal,args=(c,ser,flags,))
        sendThread.start()
    #print("Yes")
    
    if(newthread.isAlive() == False or sendThread.isAlive() == False):
        print("Thread Ended")
        c.close()
        flags[0]=False
        newthread.join()
        sendThread.join()
        
c.close()
newthread.join()
print("Bye")