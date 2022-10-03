import cv2
import RPi.GPIO as GPIO
import dht11
from mscoco_label_map import category_map
from datetime import datetime
import time
from flask import Flask, render_template

BLOB_SIZE = 300
THRESHOLD = 0.3

now1 = datetime.now()
today10to9am = now1.replace(hour=8, minute=50, second=0, microsecond=0)
today10past10am = now1.replace(hour=11, minute=10, second=0, microsecond=0)
today10to12pm = now1.replace(hour=11, minute=50, second=0, microsecond=0)
today10past1pm = now1.replace(hour=13, minute=10, second=0, microsecond=0)
today10to5pm = now1.replace(hour=15, minute=50, second=0, microsecond=0)
today10past6pm = now1.replace(hour=18, minute=10, second=0, microsecond=0)

referenceUnit = 1
sec = 0

global breakfast
global lunch
global dinner
global weight

app = Flask(__name__)

GPIO.setwarnings(True)
GPIO.setmode(GPIO.BCM)
servo_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(3.0) #서보의 0도 위치(0.6ms)이동:값 3.0은 pwm주기인 20ms의 3%를 의미하므로,0.6ms됨.
timeA = 0.6

# read data using pin 14
instance = dht11.DHT11(pin=14)
result = instance.read()

net = cv2.dnn.readNetFromTensorflow(
    'ssd_mobilenet_v2_frozen_inference_graph.pb', 'ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

if ((today10to9am < now1) & (now1 < today10past10am)) or ((today10to12pm < now1) & (now1 < today10past1pm)) or ((today10to5pm < now1) & (now1 < today10past6pm)):
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, img = cap.read()

        h, w, _ = img.shape

        net.setInput(cv2.dnn.blobFromImage(img, size=(BLOB_SIZE, BLOB_SIZE), swapRB=True))
        out = net.forward()

        boxes, scores, classes = [], [], []

        # 모터2 ON

        for detection in out[0, 0, :, :]:
            score = float(detection[2])

            if score > THRESHOLD:
                idx = int(detection[1])
                x1 = int(detection[3] * w)
                y1 = int(detection[4] * h)
                x2 = int(detection[5] * w)
                y2 = int(detection[6] * h)

                cv2.rectangle(img, (x1, y1), (x2, y2), (23, 230, 210), thickness=5)
                label = '{}: {:.0f}%'.format(category_map[idx], score * 100)
                cv2.putText(img, label, (x1, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                now = datetime.now()
                today9am = now.replace(hour=9, minute=0, second=0, microsecond=0)
                today10am = now.replace(hour=11, minute=0, second=0, microsecond=0)
                today12pm = now.replace(hour=12, minute=0, second=0, microsecond=0)
                today1pm = now.replace(hour=13, minute=0, second=0, microsecond=0)
                today5pm = now.replace(hour=17, minute=0, second=0, microsecond=0)
                today6pm = now.replace(hour=18, minute=0, second=0, microsecond=0)

                if (idx == 17 or idx == 18) & (today9am < now) & (today10am > now):
                    while True:
                        sec = sec + 1
                        time.sleep(1)
                        print(sec)
                        if sec == 5:
                            pwm.ChangeDutyCycle(3.0)  # 서보모터를 0도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(5.5)  # 서보 모터를 45도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(7.5)  # 서보 모터를 90도로 회전(이동)
                            time.sleep(timeA)
                        breakfast = True
                        if sec == 15:
                            pwm.ChangeDutyCycle(7.5)  # 서보 모터를 90도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(5.5)  # 서보 모터를 45도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(3.0)  # 서보모터를 0도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            break

                if (idx == 17 or idx == 18) & (today12pm < now) & (today1pm > now):
                    while True:
                        sec = sec + 1
                        time.sleep(1)
                        if sec == 5:
                            pwm.ChangeDutyCycle(3.0)  # 서보모터를 0도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(5.5)  # 서보 모터를 45도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(7.5)  # 서보 모터를 90도로 회전(이동)
                            time.sleep(timeA)
                            lunch = True
                        if sec == 15:
                            pwm.ChangeDutyCycle(7.5)  # 서보 모터를 90도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(5.5)  # 서보 모터를 45도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(3.0)  # 서보모터를 0도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            break


                if (idx == 17 or idx == 18) & (today5pm < now) & (today6pm > now):
                    while True:
                        sec = sec + 1
                        time.sleep(1)
                        if sec == 5:
                            pwm.ChangeDutyCycle(3.0)  # 서보모터를 0도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(5.5)  # 서보 모터를 45도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(7.5)  # 서보 모터를 90도로 회전(이동)
                            time.sleep(timeA)
                            dinner = True
                        if sec == 15:
                            pwm.ChangeDutyCycle(7.5)  # 서보 모터를 90도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(5.5)  # 서보 모터를 45도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            pwm.ChangeDutyCycle(3.0)  # 서보모터를 0도로 회전(이동)
                            time.sleep(timeA)  # 서보 모터가 이동할 시간을 줌
                            break

        cv2.imshow('result', img)
        if (cv2.waitKey(1) == ord('q')) or (sec == 10):
            # 모터2 OFF
            d = datetime.now().date
            if result.is_valid():
                print("Last valid input: " + str(datetime.datetime.now()))
                print("Temperature: %-3.1f C" % result.temperature)
                print("Humidity: %-3.1f %%" % result.humidity)
            @app.route('/')  # 기본 주소
            def home():
               if(today10to9am < d) & (today10past10am > d) & result.is_valid() & (result.temperature < 35) & (result.humidity < 70):
                   return render_template('index2.html', today=d, breakfast="O", temp1=result.temperature, humid1=result.humidity, OkorNot = "O")
               elif (today10to12pm < d) & (today10past1pm > d) & result.is_valid() & (result.temperature < 35) & (result.humidity < 70):
                   return render_template('index2.html', today=d, breakfast = "O", lunch = "O", temp1=result.temperature, humid1=result.humidity, OkorNot = "O")
               elif (today10to5pm < d) & (today10past6pm > d) & result.is_valid() & (result.temperature < 35) & (result.humidity < 70):
                   return render_template('index2.html', today=d, breakfast = "O", lunch = "O", dinner = "O", temp1=result.temperature, humid1=result.humidity, OkorNot = "O")
               elif ((today10to9am < d) & (today10past10am > d) & result.is_valid()) & (result.temperature >= 35) or (
                       result.humidity >= 70):
                   return render_template('index2.html', today=d, breakfast="O", temp1=result.temperature,
                                          humid1=result.humidity, OkorNot="X")
               elif ((today10to12pm < d) & (today10past1pm > d) & result.is_valid()) & (result.temperature >= 35) or (
                       result.humidity >= 70):
                   return render_template('index2.html', today=d, breakfast="O", lunch="O", temp1=result.temperature,
                                          humid1=result.humidity, OkorNot="X")
               elif ((today10to5pm < d) & (today10past6pm > d) & result.is_valid()) & (result.temperature >= 35) or (
                       result.humidity >= 70):
                   return render_template('index2.html', today=d, breakfast="O", lunch="O", dinner="O",
                                          temp1=result.temperature, humid1=result.humidity, OkorNot="X")

            if __name__ == "__main__":  # 웹사이트를 호스팅하여 접속자에게 보여주기 위한 부분
                app.run(host="0.0.0.0", port="8080")
                # host는 현재 라즈베리파이의 내부 IP, port는 임의로 설정
                 # 해당 내부 IP와 port를 포트포워딩 해두면 외부에서도 접속가능
            sec = 0
            break




