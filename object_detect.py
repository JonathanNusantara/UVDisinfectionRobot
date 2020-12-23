import cv2
import time

classNames= []
classFile = 'coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'graph.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def detect(frame,thres,nms,objects=[]):
    classIds, confidences, boxes = net.detect(frame,confThreshold=thres,nmsThreshold=nms)
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(),confidences.flatten(),boxes):
            className = classNames[classId - 1]
            if className in objects:
                cv2.rectangle(frame,box,color=(0,0,255))
                cv2.putText(frame, className.upper(), (box[0]+10,box[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                cv2.putText(frame, str(round(confidence * 100,2)), (box[0]+100,box[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    return frame

if __name__ == "__main__":
    cap = cv2.VideoCapture(-1)
    cap.set(3, 320)
    cap.set(4, 320)
    oldtime = 0
    newtime = 0

    while True:
        newtime = time.time()
        _, frame = cap.read()
        frame = cv2.flip(frame, -1)
        result = detect(frame,0.5,0.2,["mouse", "keyboard"])
        fps = 1 / (newtime - oldtime)
        oldtime = newtime
        print(round(fps,2), "frames per second")
        cv2.imshow('Output', frame)
        cv2.waitKey(1)
