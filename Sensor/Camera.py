import cv2
import numpy as np
import time
from imutils.video import WebcamVideoStream
from imutils.video import FPS
from ultralytics import YOLO

class Camera:
    def __init__(self):
        # 카메라 설정
        self.cam = WebcamVideoStream(-1).start() # 카메라 오픈
        self.fps = FPS() # 디버깅용 fps, 나중에 off
        shape = (self.height, self.width, _) = self.get_image().shape
        print(shape) # 세로, 가로 출력
        time.sleep(2)
        
        # YOLO 설정
        self.model = YOLO('yolov8n.pt')
    
    # 이미지 공급 쓰레드에서 이미지 하나 get.    
    def get_image(self):
        try:
            print("image get")
            return self.cam.read().copy()
        except AttributeError: # 이미지를 얻지 못할경우 검은화면 반환
            print("Attribute Error")
            return np.zeros(shape=(480, 640, 3), dtype="uint8")
        
    def ball_position(self, img):
        return 0, 0, 0 # cx, cy
    
    def ball_distance(self, angle):
        return 0
    
    def cvCircleDetect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=40, minRadius=40, maxRadius=100)
        
        if np.any(circles == None):
            return False, img, None
        
        for i in circles[0]:
            cv2.circle(img, (i[0], i[1]), i[2], (255,255,255), 3)
        return True, img, (circles[0][0][0],circles[0][0][1])
    
    def yoloDetect(self, img):
        result = self.model(img, classes = 32, conf = 0.8)
        print(result)
        