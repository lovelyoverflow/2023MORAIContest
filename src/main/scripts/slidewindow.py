import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *
from matplotlib.pyplot import *

import rospy
from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import CompressedImage


TOTAL_CNT = 50

class SlideWindow:

    def __init__(self):
        self.current_line = "LEFT"
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None
        self.left_cnt = 25
        self.right_cnt = 25
        self.current_lane = 1

        # good_left_inds = 1
        # good_right_inds = 1
        
    def voidCallback(self, _data):
        self.current_lane = _data.data
        # rospy.loginfo(f"{self.current_lane}")


    # 이미지를 입력으로 받아 차선 감지를 수행하는 메서드
    def slidewindow(self, img):
        rospy.Subscriber("currentLane", Int32, self.voidCallback)

        # 차선 중심 위치
        x_location = 280

        # 이미지 초기화 및 크기 설정       
        out_img = np.dstack((img, img, img)) * 255 # 입력 이미지를 기반으로 출력 이미지 시각화
        height = img.shape[0]
        width = img.shape[1]

        # 슬라이딩 윈도우 관련 설정
        window_height = 7 # 슬라이딩 윈도우 높이
        nwindows = 30     # 슬라이딩 윈도우 개수
        cv2.imshow("TEST", img)
        
        # 이미지에서 픽셀의 위치 찾기
        nonzero = img.nonzero()         # 이미지에서 픽셀 값이 0이 아닌 위치
        nonzeroy = np.array(nonzero[0]) # 픽셀의 y 좌표
        nonzerox = np.array(nonzero[1]) # 픽셀의 x 좌표
        
        # 차선 박스의 초기 위치와 크기 설정
        margin = 20 
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []

        # 윈도우 박스 및 라인 위치 초기화
        win_h1 = 400
        win_h2 = 480
        win_l_w_l = 140
        win_l_w_r = 240
        win_r_w_l = 310
        win_r_w_r = 440
        
        # 왼쪽 차선 박스 그리기
        pts_left = np.array([[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)

        # 오른쪽 차선 박스 그리기
        pts_right = np.array([[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        
        # 가운데 차선 박스 그리기
        pts_catch = np.array([[0, 340], [width, 340]], np.int32)
        cv2.polylines(out_img, [pts_catch], False, (0,120,120), 1)


        # 초기 차선 인덱스 설정
        good_left_inds = ((nonzerox >= win_l_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)).nonzero()[0]
        good_right_inds = ((nonzerox >= win_r_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)).nonzero()[0]

        # 초기 차선 방향 및 위치 설정
        line_exist_flag = None 
        y_current = 0
        x_current = 0
        good_center_inds = None
        p_cut = None



        # 왼쪽 차선이 더 잘 인식될 때
        if self.current_lane == 1:
            # rospy.loginfo("1")
            self.current_line = "LEFT"
            line_flag = 1
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", good_left_inds)
            
            x_current = np.int(np.mean(nonzerox[good_left_inds]))
            y_current = np.int(np.mean(nonzeroy[good_left_inds]))
            print(x_current)
            max_y = y_current
        # 오른쪽 차선이 더 잘 인식될 때
        elif self.current_lane == 2:
            # rospy.loginfo("2")
            if (self.left_cnt + self.right_cnt <= TOTAL_CNT) :
                self.right_cnt += 1
                self.left_cnt -=1
            self.current_line = "RIGHT"
            line_flag = 2
            x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            y_current = np.int(np.max(nonzeroy[good_right_inds]))
        else:
            # rospy.loginfo("3")
            self.current_line = "MID"
            line_flag = 3

        # 양쪽 차선 감지가 비슷한 경우 (가운데 차선)
        if line_flag != 3:
            # 유효한 인덱스에 점 표시
            for i in range(len(good_left_inds)):
                    img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
            # 윈도우 슬라이딩 및 그리기
            for window in range(0, nwindows):
                if line_flag == 1: 
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low + int(width * 0.27), win_y_low), (win_x_high + int(width * 0.27), win_y_high), (255, 0, 0), 1)
                    # indicies of dots in nonzerox in one square
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    # check num of indicies in square and put next location to current 
                    if len(good_left_inds) > len(good_right_inds):
                        x_current = np.int(np.mean(nonzerox[good_left_inds]))
                    
                    elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                        p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                        x_current = np.int(np.polyval(p_left, win_y_high))
                    # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                    if win_y_low >= 338 and win_y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        x_location = x_current + int(width * 0.135) 
                else: # change line from left to right above(if)
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    cv2.rectangle(out_img, (win_x_low - int(width * 0.27), win_y_low), (win_x_high - int(width * 0.27), win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    if len(good_right_inds) > len(good_left_inds):
                        x_current = np.int(np.mean(nonzerox[good_right_inds]))
                    elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
                        x_current = np.int(np.polyval(p_right, win_y_high))
                    if win_y_low >= 338 and win_y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        x_location = x_current - int(width * 0.135) 

                left_lane_inds.extend(good_left_inds)
        
        return out_img, x_location, self.current_line