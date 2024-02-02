#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy

class Class_example:
    def __init__(self) -> None:
        self.datta = 0

    def func(self):
        pass

def main():
    try:
        class_name = Class_example
        class_name.func()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()