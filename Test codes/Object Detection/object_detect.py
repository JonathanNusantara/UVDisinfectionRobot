import cv2
import time

FOCAL = 220 # camera focal length
LENGTH_MOUSE = 14.3
# dist_calc = WIDTH_BALL * FOCAL / dist_rad

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

def getObjects(frame,thres,nms,draw=True,objects=[]):
    classIds, confs, bbox = net.detect(frame,confThreshold=thres,nmsThreshold=nms)
    found = 0
    if len(objects) == 0: 
        objects = classNames
    objectInfo = []
    
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            
            if className in objects:
                objectInfo.append([box, className])
                found = 1
                if (draw):
                    
                    cv2.rectangle(frame,box,color=(0,0,255))
                    cv2.putText(frame,className.upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                    cv2.putText(frame,str(round(confidence*100,2)),(box[0]+100,box[1]+30), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                    
                    
                    x_mid = box[0] + box[2]/2 # Calculated and verified to be x midpoint
                    # If x_mid < 100, turn one dir, if x_mid > 220, turn another dir
                    if x_mid < 120:
                        print("steer right")
                    elif x_mid > 200:
                        print("steer left")
                    
                    object_ratio = (box[2]*box[3])/(320*320)
                    # if less than 0.4, move forward, else lean arm forward
                    if object_ratio < 0.4:
                        print("forward")
                    else:
                        print("move arm")
                    
                    
                    # Print out distance to mouse
                    #print(int(box[0]),box[1],box[2],box[3]) # x1 y1 x_length y2, top left, bottom roght
                    #print(box[0] + box[2]/2)
                    #print(box[1] + box[3]/2)
                    #print(box[3] - box[1])

                    
                    #dist_calc = LENGTH_MOUSE * FOCAL / max((int(box[2]) - int(box[0])), (int(box[3]) - int(box[1])))
                    #print(dist_calc)
                    #print((int(box[2]) - int(box[0])) * (int(box[3]) - int(box[1])))
                    
                    # dist_calc = WIDTH_BALL * FOCAL / dist_rad

                    
                    
                    
    return frame,objectInfo,found

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
        result,objectInfo,found = getObjects(frame,0.5,0.2,objects=["keyboard"])
        print(found)
        #cv2.rectangle(frame,[10,50,160,320],color=(0,0,255))
        fps = 1 / (newtime - oldtime)
        oldtime = newtime
        #print(round(fps,2), "frames per second")
        cv2.imshow('Output', frame)
        cv2.waitKey(1)
