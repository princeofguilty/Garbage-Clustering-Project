import RPi.GPIO as GPIO
import time
import requests
from ultralytics import YOLO
import cv2

model = YOLO("best36.pt")
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
#results = model.predict(source="0")
#results = model.predict(source="folder", show=True) # Display preds. Accepts all YOLO predict arguments

# from PIL
#im1 = Image.open("bus.jpg")
#results = model.predict(source=im1, save=True)  # save plotted images

# from ndarray
#im2 = cv2.imread("bus.jpg")
# cap = cv2.VideoCapture("http://192.168.1.2:8080/video")
# cap = cv2.VideoCapture("20230510_101206.mp4")
names = model.names
print(names)

class_detected = 0


button1=3
button2=5
button3=7

servopin1=12
servopin2=32
servopin3=33

echoPin=16
trigPin=18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servopin1,GPIO.OUT)
GPIO.setup(servopin2,GPIO.OUT)
GPIO.setup(servopin3,GPIO.OUT)
GPIO.setup(echoPin,GPIO.IN)
GPIO.setup(trigPin,GPIO.OUT)

GPIO.setup(button1,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(button2,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(button3,GPIO.IN,pull_up_down=GPIO.PUD_UP)

servo=GPIO.PWM(servopin1,50)
servo1=GPIO.PWM(servopin2,50)
servo2=GPIO.PWM(servopin3,50)

servo.start(0)
servo1.start(0)
servo2.start(0)

# buttonPressed=0

def distance():
    # set Trigger to HIGH
	GPIO.output(trigPin, GPIO.HIGH)
 
    # set Trigger after 0.01ms to LOW
	time.sleep(0.002)
	GPIO.output(trigPin, GPIO.LOW)
 
	StartTime = time.time()
	StopTime = time.time()
 
    # save StartTime
	while GPIO.input(echoPin) == GPIO.LOW:
		StartTime = time.time()
 
    # save time of arrival
	while GPIO.input(echoPin) == GPIO.HIGH:
		StopTime = time.time()
 
    # time difference between start and arrival
	TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
	distance = (TimeElapsed * 34300) / 2
 
	return distance

plasticCounter=0
canCounter=0
paperCounter=0
prevState=0

servos=["not pressed",servo,servo1,servo2]

while True:
	cap = cv2.VideoCapture("http://192.168.1.2:8080/video")
	ret, frame = cap.read()
	cap.release()			
	results = model.predict(source=frame)  # save predictions as labels\
	# print(results)
	# for result in results:
		# print(result.cls)
	for r in results:
		for c in r.boxes.cls:
			name = names[int(c)]
			print(name)
			if(name == "Plastic bottle"):
				class_detected = 1
			elif(name == "Can"):
				class_detected = 2
			elif(name == "Paper"):
				class_detected = 3
			else:
				class_detected = 0


	# print("cat")
	dist=distance()
	# print("dog")
	print("Distance: ",dist,"cm")
	print(class_detected)
	# if GPIO.input(button1)==GPIO.LOW:
	# 	buttonPressed=1
	# elif GPIO.input(button2)==GPIO.LOW:
	# 	buttonPressed=2
	# elif GPIO.input(button3)==GPIO.LOW:
	# 	buttonPressed=3

	print("conditions ", dist, class_detected)

	if dist <= 30 and prevState!=class_detected:
		if class_detected == 1:
			servo.ChangeDutyCycle(12)
			plasticCounter+=1
		elif class_detected==2:
			servo1.ChangeDutyCycle(12)
			canCounter+=1
		elif class_detected==3:
			servo2.ChangeDutyCycle(12)
			paperCounter+=1
		if prevState!=0:
			(servos[prevState]).ChangeDutyCycle(7)

		response = requests.post('http://192.168.1.3:80/data/'+str(plasticCounter)+'/'+str(canCounter)+'/'+str(paperCounter))
		prevState = class_detected

	if dist > 30 and class_detected != 0:
		(servos[class_detected]).ChangeDutyCycle(2)
		class_detected=0
		prevState=0
