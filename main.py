import cv2
import sys
from mscoco_label_map import category_map
from datetime import datetime, time

BLOB_SIZE = 300
THRESHOLD = 0.3
EMULATE_HX711 = False
sec = 0

now1 = datetime.now()
today10to9am = now1.replace(hour=8, minute=50, second=0, microsecond=0)
today10past10am = now1.replace(hour=10, minute=10, second=0, microsecond=0)
today10to12pm = now1.replace(hour=11, minute=50, second=0, microsecond=0)
today10past1pm = now1.replace(hour=13, minute=10, second=0, microsecond=0)
today10to5pm = now1.replace(hour=16, minute=50, second=0, microsecond=0)
today10past6pm = now1.replace(hour=18, minute=10, second=0, microsecond=0)

referenceUnit = 1

global breakfast
global lunch
global dinner
global weight

if not EMULATE_HX711:
    import RPi.GPIO as GPIO
    from hx711 import HX711
else:
    from emulated_hx711 import HX711


def cleanAndExit():
    print("Cleaning...")

    if not EMULATE_HX711:
        GPIO.cleanup()

    print("Bye!")
    sys.exit()

hx = HX711(20, 16)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(referenceUnit)
hx.reset()
hx.tare()

net = cv2.dnn.readNetFromTensorflow(
    'ssd_mobilenet_v2_frozen_inference_graph.pb', 'ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

if (today10to9am < now1 & today10past10am) or (today10to12pm < now1 & today10past1pm) or (today10to5pm < now1 & today10past6pm):
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, img = cap.read()
        try:
            val = hx.get_weight(5)
            print(val)
            hx.power_down()
            hx.power_up()
            time.sleep(0.1)

        except (KeyboardInterrupt, SystemExit):
            cleanAndExit()

        if not ret:
            break

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
                today10am = now.replace(hour=10, minute=0, second=0, microsecond=0)
                today12pm = now.replace(hour=12, minute=0, second=0, microsecond=0)
                today1pm = now.replace(hour=13, minute=0, second=0, microsecond=0)
                today5pm = now.replace(hour=17, minute=0, second=0, microsecond=0)
                today6pm = now.replace(hour=18, minute=0, second=0, microsecond=0)

                if (idx == 17 or idx == 18) & (today9am < now) & (today10am > now):
                    if not breakfast:
                        while True:
                            sec = sec + 1
                            time.sleep(1)
                            if sec == 5:
                                print("아침")  # 수정해야 할 부분(모터1 On)
                                breakfast = True
                                if val == 100:
                                    print("이제 아침 그만")  # 수정해야 할 부분(모터1 OFF)
                            if sec == 600 & val < 30:
                                print("아침 안 먹을거야?")  # 수정해야 할 부분

                if (idx == 17 or idx == 18) & (today12pm < now) & (today1pm > now):
                    if not lunch:
                        while True:
                            sec = sec + 1
                            time.sleep(1)
                            if sec == 5:
                                print("점심")  # 수정해야 할 부분(모터1 On)
                                lunch = True
                                if val == 100:
                                    print("이제 점심 그만")  # 수정해야 할 부분(모터1 OFF)
                            if sec == 600 & val < 30:
                                print("점심 안 먹을거야?")  # 수정해야 할 부분

                if (idx == 17 or idx == 18) & (today5pm < now) & (today6pm > now):
                    if not dinner:
                        while True:
                            sec = sec + 1
                            time.sleep(1)
                            if sec == 5:
                                print("저녁")  # 수정해야 할 부분(모터1 On)
                                dinner = True
                                if val == 100:
                                    print("이제 저녁 그만")  # 수정해야 할 부분(모터1 OFF)
                            if sec == 600 & val < 30:
                                print("저녁 안 먹을거야?")  # 수정해야 할 부분

        cv2.imshow('result', img)
        if cv2.waitKey(1) == ord('q') & sec == 900:
            # 모터2 OFF
            sec = 0
            break



