#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2

class Camtest:
    def __init__(self) -> None:
        self.img_src = "test/scripts/img.jpg"
        self.ros = 0
        self.wait_time = 100
        self._print_img_hsv_isrun = False
        self.keycode = ""

    def lines(self):
        zero = np.zeros((480,640,3), np.uint8)
        y, x, _ = zero.shape
        cv2.line(zero, (0, 0), (int(x/3*2), int(y/3*2)), (0, 0, 255), 3)
        cv2.rectangle(zero, (0, 0), (int(x/3*2), int(y/3*2)), (0, 0, 255), 3)
        cv2.circle(zero, (int(x/3*2), int(y/3*2)), 50, (0, 0, 255), 3)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", zero)
        self._cv2_wait("Image")

    def get_img(self, directsrc = None):
        return cv2.imread(self.img_src, cv2.IMREAD_UNCHANGED) if directsrc is None else directsrc

    def print_img(self, img):
        y, x, _ = img.shape
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", img)
        self._cv2_wait("Image")

    def print_img_hsv(self, img):
        y, x, _ = img.shape
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)

        cv2.namedWindow("h", cv2.WINDOW_NORMAL)
        cv2.namedWindow("s", cv2.WINDOW_NORMAL)
        cv2.namedWindow("v", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

        if not self._print_img_hsv_isrun:
            cv2.resizeWindow("Image", x*2, y*2)
            cv2.resizeWindow("h", x*2, y*2)
            cv2.resizeWindow("s", x*2, y*2)
            cv2.resizeWindow("v", x*2, y*2)
            cv2.moveWindow("Image", 1, 1)
            cv2.moveWindow("h", x*2+150, 1)
            cv2.moveWindow("s", 1, y*2+150)
            cv2.moveWindow("v", x*2+50, y*2+150)
            self._print_img_hsv_isrun = True
            print("finished")

        cv2.imshow("h", h)
        cv2.imshow("s", s)
        cv2.imshow("v", v)
        cv2.imshow("Image", img)

        self._cv2_wait(("Image", "h", "s", "v"))

    def img_experiment(self, img):
        y, x, _ = img.shape
        b, g, r = cv2.split(img)
        cvted = cv2.merge(([b, g, r]))

        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", img)
        cv2.namedWindow("Converted", cv2.WINDOW_NORMAL)
        cv2.imshow("Converted", cvted)
        self._cv2_wait(["Image", "Converted"])

    def _cv2_wait(self, window):
        wait_time = self.wait_time
        running = True
        while running:
            keyCode = cv2.waitKey(wait_time)
            self.keycode = chr(keyCode & 0xFF)
            if (keyCode & 0xFF) == ord("q"):
                running = False
                break
            for i in [window] if isinstance(window, str) else window:
                # print(i) # Debugging
                if not cv2.getWindowProperty(i, cv2.WND_PROP_VISIBLE) >= 1:
                    # cv2.destroyAllWindows()
                    running = False
            if self.ros == True: break
        if self.ros==True and running==False:
            rospy.signal_shutdown("Fuck you. There's no reason.")

    def img(self, directsrc = None):
        self.print_img(self.get_img(directsrc))

    def imghsv(self, directsrc = None):
        self.print_img_hsv(self.get_img(directsrc))



def main():
    try:
        instance = Camtest()
        # instance.imghsv()
        instance.img_experiment(instance.get_img())
        # instance.lines()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()