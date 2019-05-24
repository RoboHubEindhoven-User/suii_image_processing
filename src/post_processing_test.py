#!/usr/bin/env python
#
# Created by Jeroen Bongers in name of RoboHub Eindhoven
#

from post_processing_v2 import PostProcessing
import cv2


class PostProcessingTest:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.test = PostProcessing()
        self.postprocessing_test()   

    def postprocessing_test(self):
        _, frame = self.cap.read()
        if frame is not None:
            build_center = self.test.build_center("Bolt",(0,0,640,480),frame,True)
            build_view = self.test.build_view()
            reset_view = self.test.reset_view()
            print(build_center)
            print(build_view)
            print(reset_view)
            cv2.waitKey(5000)

if __name__ == '__main__':
    PostProcessingTest()
