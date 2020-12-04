# AnjelDrone
I made a Quadcopter from very Basic using Arduino , Raspberrypi and Android app.  
I am Photography and Cinematography Enthusiast , i always try to capture beautiful and stunning moments with cameras. I have seen some incredible shots taken in cinema using Drones. But those drones are super Expensive. Being an Engineering enthusiast i love to implement my Ideas practically, So i strarted my New Project - "The Anjel Drone". 
I have this Idea of making a Quadcopter from the very basic level , without using any Flight Controller . I created this whole project building my own software and logics to control the Quadcopter.
I am making this project to record beautiful Videos and in future i will try to thing of different scenerios where it will be useful in my Farms.

Chapter 2: Project Requirements (Software/Hardware requirements)
Coming to Hardware and Softwares, 
choosing hardware for my drone is the very first and complex step.
Basically a Drone (Quadcopter) consist of 4 Brushless Motors (are more efficient that Brushed Motor ) . and these Brushless Motors works on 3 phase AC current . But a drone only has a DC battery. So to convert DC to AC , i used 4 ESC (Electronic Speed Controller) for each Motor. and ESC can be easily controlled by PWM signal.
To Provide the PWM signal to ESC's i couldn't find anything better than Arduino Uno(just Best). So Speed of Each Motor can be controlled easily by changing PWM signal in Arduino. But then comes the main Part - "How to communicate with Arduino ". To communicate with arduino i Used Raspberrypi 4 ("The main Board on top of Everything in my Drone") . Using Raspberrypi gives me opportunity of doing so many things in my Drone. I can attach "Camera" , "can Implement hard Algorithms " ,"Record data Easily", "In future , i might use some Basic Neural Network Model ".
Communication between Arduino and Raspberrypi is done using the USB port (very simple connection but real work depend on software protocols).
Now the most important thing in drone is IMU (Inertial Measurement Unit) basically MPU6050 accelrometer and Gyro Sensor for Motion processing in Drone.Sesor will output the Roll ,pitch and yaw angle of Drone. This was the hardest part , and i have learnt so many things while implementing this eg - "complimentary filter", "noise in sensor due to motor vibrations",etc. 

For Ground Station or transmitter i used Smartphone to send "Thrust, roll and pitch " signal to Drone.

To give power to my "Anjel Drone" i need 2 Lipo Batteries, one for all the Boards and 2nd(big one) for all 4 motors.

Coming to the Software part- 
I program Raspberry Pi with  Python , code on Raspberry pi basically focused on Communication between Arduino and Smartphone. I have to use threads to simuntanously accept and send data on both devices.
And also Raspberrypi also contains the code to capture Beautiful Videos with camera.
Arduino is programmed using C language, which make it faster to implement code , the main work of Arduino software is to get data from MPU6050 sensor and Rspberrypi , the controlling the Motors speed based on that Data. I tried to move most of code on raspberrypi because Arduino only has 1kb of ram, and it will be very difficult for Arduino to process large Strings.

I have  Python program in my Laptop to send signal to Raspberry Pi or you can say Ground Controller. 
Also , i have made very basic android app to controll drone, means i only require a smartphone to controll my drone.

 Chapter 3: Implementation Details (Algorithm, code )
So , until now , i have told you my main idea of Implementaion.
TO let you know , i found no one on the Internet who has made a drone based on this Idea, this is solely my own hard work of many months. Now you will say "TALK IS CHEAP , SHOW ME THE CODE". Everything is below here .

ALGORITHMS - 
As we all know most of good Electronic or even Life work on feedback loop.
There is also a loop, the whole system is running in loop, if loop breaks due to any error , drone will fall down.

Loop is - Smartphone is sending Signal(thrust,pitch,yaw values) to Raspberry pi via WIFI , then raspberrypi is send it further to arduino through USB cable, based on that data ,Ardunio is controlling the speed of Motors using Motor Mixing Algorithm (will come to that later) . 
Also the Senor is sending real (thrust , pitch , yaw values) to arduino , based on the control values (sent by smartphone) and real values (sent by sonsor), arduino will calculate the error and then arduino will change the speed of motors until the error reaches 0(zero). I am using PID control
to control the speed of motors, PID stands for Proportional , Integration and Derivative.  PID is most important part of the Drone for its stable flight. PID control tries to minimise error in better way. 

Proportional means how reactive the motors will be based on Error, it is direct relation between error and Motor Speed . eg- if error is 20 ,means if drone is 20 degree far from its original required angle, the proportional part of control will apply Kp * 20 speed to desired Motors. Here Kp is constant for proportional control , which we have to find based on test flights.
2nd part of PID is Derivative - I found this one super interesting , and it really makes me feel good about maths. Basically we all know derivative tell us the slope  at particular time, means it predicts what will happen in future based on  very very short time interval, In this Drone also , it calculates the change in error over time.If the change in error is large in short amount of time, it will give us a large value , means drone will react faster to compensate and vice-versa.
3rd part is Integral - This one is also good , but i am afraid to use it , because it can cause huge change in speed values of motor over time.
Basic idea behind this is that, based on past errors it adds the previous error and learning from past values it uses that to control speed. I haven't used this one. It is also necessary.

At last the hole PID algorithm (leaving Integeral)looks like
	                         output = Kp*error + Kd*(error/elapsedTime)
where Kp and Kd are constants.

After Error is minimized the Arduino will send real values back to raspberryPi and then to Smartphone. This process will keep on repeating and Drone will keep on Flying (I guess so).

Now Coming to the other important part - Communication Algorithm
You might be thinking how i will send signal , thats also pure Basic.
I am using Sockets in python and Sockets in Java (smartphone) for communication and for RaspberryPi to Arduino , i am using Serial connection. 
So my Drone Act as a server , which requires clients to connect and fly him. Smartphone and Laptop are clients. 
Also there is protocol to send Data , like Data is only  8 - 10 bytes long string. Which contains (  thrust roll pitch ) eg- "1030 7 -5" . Then this data is sent to Raspberry Pi , Which handle it on different thread , then raspberrypi send it to Arduino, Arduino seperate each part of string to desired Datatype eg - thrust will be 1030 , roll will be = 7 ,pitch will be = -5.
I wrote this algorithm in arduino 

	void processSerialData()
	{ 
	  thrust = mainData.substring(0,4).toInt();
	  int i=5,len = mainData.length();
	  if(len>6)
	  {
	    while(mainData.charAt(i)!=' ')
	    {
	      i++;
	    }
	    inputRoll = mainData.substring(5,i).toInt();
	    i++;
	    inputPitch = mainData.substring(i,len).toInt();
	  } 
	}

Now Coming to Android Part - 
Using this Code i am sending data over socket from android to Raspberrypi
This Thread1 class is continousally sendind control data , i have to use thread class because socket can make Main Thread wait for long time , hence the app might crash down. So in seperate thread an infine loop is running and sending the data.
	
	class Thread1 implements Runnable
	{
	    @Override
	    public void run() {
		try {
		    Socket s = new Socket("192.168.43.112",6795); 
	// Ip address of Drone and Port number
		    DataInputStream inputStream = new DataInputStream(s.getInputStream());
		    DataOutputStream outputStream = new DataOutputStream(s.getOutputStream());
		    textView.setText(String.valueOf(inputStream.read()));
		    while (true)
		    {
			outputStream.write((thrust+" "+roll+" "+pitch+"\n").getBytes());
			outputStream.flush();
			try {
			    Thread.sleep(10);
			} catch (InterruptedException e) {
			    e.printStackTrace();
			}
			//System.out.println(inputStream.read());
		    }
		} catch (IOException e) {
		    e.printStackTrace();
		}
	    }
	}

Here is the Ardunio Motor Mixing Algorithm - 
MFR = Motor Front Right, MBL = Motor Back Left and so on.
This Algorithm is based on physics .
Basically what it is doing is , when we increase roll values it will increase speed of Right Motors tilting it to opposite side , means we are rolling it or can say moving in that particular direction.
Similarly it works for Pitch .
But for Yaw it is slightly different - it is based on angular momentum.
In drone 2 motors are spinning in clockwise and 2 Motors are moving in counter clockwise to give resultant Angular momentum = 0.
So if we want to rotate our Drone we have to change value of Angular Momentum. It's very easy to do so, we can  increasing of 2 clockwise motors and decreasing 2 counterclockwise motors, which will cause drone to rotate in  Counter Clockwise Direction.  
or if we do opposite to that, Drone will rotate in Clockwise direction . 
EASY PEASY.
Below is the Simple Code - 

	void MotorMixing()
	{
	  MFRvalue = thrust+yawMotor+pitchMotor+rollMotor;
	  MFLvalue = thrust-yawMotor+pitchMotor-rollMotor;
	  MBRvalue = thrust-yawMotor-pitchMotor+rollMotor;
	  MBLvalue = thrust+yawMotor-pitchMotor-rollMotor;
	 }

Raspberry Pi Code is Below - 

	import time
	import socket
	from threading import Thread
	import serial
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
                    ser.write(arduinoData.encode('utf-8')) # Here i am sending #data to Arduino via USB
                
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
		ser = serial.Serial('/dev/ttyUSB0', 115200,timeout=0) #Connecting #to Arduino 
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


To get the full Code check my Github Repository
Main Super Contains Arduino Code
Python Drone Station contains code to fly drone using PC
