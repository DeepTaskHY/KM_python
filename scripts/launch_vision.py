#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import json

import cv2
import face_recognition

import os
import numpy as np
import time


_count = 0

class FaceRecognizer:
    def __init__(self):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.
        self.camera = VideoCamera()

        self.known_face_encodings = []
        self.known_face_names = []

        # Load sample pictures and learn how to recognize it.
        dirname = '../data/known_face'
        files = os.listdir(dirname)
        for filename in files:
            name, ext = os.path.splitext(filename)
            name = name.split('_')[0]
            if ext == '.jpg':
                self.known_face_names.append(name)
                pathname = os.path.join(dirname, filename)
                img = face_recognition.load_image_file(pathname)
                face_encoding = face_recognition.face_encodings(img)[0]
                self.known_face_encodings.append(face_encoding)

        # Initialize some variables
        self.face_locations = []
        self.face_encodings = []
        self.face_names = []
        self.process_this_frame = True

    def __del__(self):
        del self.camera

    def get_frame(self):
        # Grab a single frame of video
        frame = self.camera.get_frame()

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        if self.process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            self.face_locations = face_recognition.face_locations(rgb_small_frame)
            self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)

            self.face_names = []
            for face_encoding in self.face_encodings:
                # See if the face is a match for the known face(s)
                distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                min_value = min(distances)

                # tolerance: How much distance between faces to consider it a match. Lower is more strict.
                # 0.6 is typical best performance.
                name = "Unknown"
                if min_value < 0.6:
                    index = np.argmin(distances)
                    name = self.known_face_names[index]

                self.face_names.append(name)

        self.process_this_frame = not self.process_this_frame

        # Display the results
        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, 'Face ID [{}]'.format(name), (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        return frame, self.face_names

    def get_jpg_bytes(self):
        frame = self.get_frame()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        ret, jpg = cv2.imencode('.jpg', frame)
        return jpg.tobytes()


class VideoCamera(object):
    def __init__(self):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.
        self.video = cv2.VideoCapture(0)
        self.fps = self.video.get(cv2.CAP_PROP_FPS)
        # If you decide to use video.mp4, you must have this file in the folder
        # as the main.py.
        # self.video = cv2.VideoCapture('video.mp4')

    def __del__(self):
        self.video.release()

    def get_frame(self):
        # Grab a single frame of video
        ret, frame = self.video.read()
        return frame


def to_ros_msg(data):
    global _count
    
    json_msg = {
        'header': {
            'source': 'vision',
            'target': ['planning'],
            'content': 'face_recognition',
            'id': _count+1
        },
        'face_recognition': {
            'face_id': data,
            'timestamp': str(time.time())
        }
    }
    ros_msg = json.dumps(json_msg, ensure_ascii=False, indent=4)

    return ros_msg


if __name__ == '__main__':
    rospy.init_node('vision_node')
    rospy.loginfo('Start Vision')
    publisher = rospy.Publisher('/recognition/face_id', String, queue_size=10)
    img_pub = rospy.Publisher("/recognition/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    fr = FaceRecognizer()

    rate = rospy.Rate(fr.camera.fps)
    
    while not rospy.is_shutdown():
        frame, face_names = fr.get_frame()
        cv2.imshow('Frame', frame)
        key = cv2.waitKey(1) & 0xFF
    
        if key == ord('q'):
            break
        
        try:
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            img_pub.publish(img_msg)
        except CvBridgeError as err:
            print(err)
        
        try:
            publisher.publish(to_ros_msg(face_names[0]))
        except:
            pass

        rate.sleep()

    cv2.destroyAllWindows()
    # while True:
    #     frame, face_names = fr.get_frame()
    #
    #     cv2.imshow('Frame', frame)
    #     key = cv2.waitKey(1) & 0xFF
    #
    #     if key == ord('q'):
    #         break
    #
    #     # if len(face_names) != 0:
    #     #     rospy.loginfo(face_names[0])
    #     #     publisher.publish(to_ros_msg(face_names[0]))
    #
    # cv2.destroyAllWindows()


