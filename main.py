import cv2
from mscoco_label_map import category_map

BLOB_SIZE = 300
THRESHOLD = 0.3

net = cv2.dnn.readNetFromTensorflow(
    'ssd_mobilenet_v2_frozen_inference_graph.pb', 'ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

cap = cv2.VideoCapture('videos/24.mp4')

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    h, w, _ = img.shape

    net.setInput(cv2.dnn.blobFromImage(img, size=(BLOB_SIZE, BLOB_SIZE), swapRB=True))
    out = net.forward()

    boxes, scores, classes = [], [], []

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

    cv2.imshow('result', img)
    if cv2.waitKey(1) == ord('q'):
        break
