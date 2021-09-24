#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String

import json
import requests
import time
import queue
import threading
import sounddevice as sd
import soundfile as sf
from pynput.keyboard import Key, Listener
import os

_count = 0

def to_ros_msg(data):
    global _count
    json_msg = {
        'header': {
            'source': 'stt',
            'target': ['planning'],
            'content': 'human_speech',
            'id': _count+1
        },
        'human_speech': {
            'stt': data,
            'timestamp': str(time.time())
        }
    }
    ros_msg = json.dumps(json_msg, ensure_ascii=False, indent=4)

    return ros_msg


def stt(audio_file):

    # 네이버 API 예제
    data = open(audio_file, 'rb')  # STT를 진행하고자 하는 음성 파일
    url = 'https://naveropenapi.apigw.ntruss.com/recog/v1/stt?lang=Kor'

    client_id = 'jqhycv20tf'  # 인증 정보의 Client ID
    client_secret = 'EFh6RhwU8DDuLX3O1qaSCqc2jRgu2P6asUl6wmiR'  # 인증 정보의 Client Secret

    headers = {
        'Content-Type': 'application/octet-stream',  # Fix
        'X-NCP-APIGW-API-KEY-ID': client_id,
        'X-NCP-APIGW-API-KEY': client_secret,
    }

    response = requests.post(url, data=data, headers=headers)
    rescode = response.status_code

    if rescode == 200:
        text_dict = json.loads(response.text)
        text_data = text_dict['text']
        rospy.loginfo(text_data)
    else:
        rospy.loginfo('Error : {}'.format(response.text))
        return ''

    return text_data


class Recorder:
    def __init__(self):
        self.recording = False
        self.q = queue.Queue()
        self.rec = None
        self.save_name = '../data/human_speech.wav'
        self.now_pressed = None
        

    def record_voice(self):
        with sf.SoundFile(self.save_name, mode='w', samplerate=16000, subtype='PCM_16', channels=1) as f:
            with sd.InputStream(samplerate=16000, dtype='int16', channels=1, callback=self.save_voice):
                while self.recording:
                    f.write(self.q.get())

    def save_voice(self, indata, frames, time, status):
        self.q.put(indata.copy())

    def key_press(self, key):
        if key is Key.esc:
            return False

        elif key is not self.now_pressed:
            self.now_pressed = key
            self.recording = True
            self.rec = threading.Thread(target=self.record_voice)
            print('Start REC')
            self.rec.start()
            

    def key_release(self, key):
        if key == self.now_pressed:
            self.now_pressed = None
            self.recording = False
            self.rec.join()
            print('Stop REC')
            text = stt(self.save_name)
            publisher.publish(to_ros_msg(text))
            # temp_publisher.publish(text)

        if key == Key.esc:
            return False
        

    def on(self):
        with Listener(on_press=self.key_press, on_release=self.key_release, suppress=True) as listener:
            listener.join()




if __name__ == '__main__':
    rospy.init_node('stt_node')
    rospy.loginfo('Start STT')
    publisher = rospy.Publisher('/recognition/speech', String, queue_size=10)
    # temp_publisher = rospy.Publisher('/action/speech', String, queue_size=10)
    recorder = Recorder()
    print('Press any key...')
    recorder.on()
    
    # while(True):
    #     try:
    #         if keyboard.is_pressed('q'):
    #             break
    #     except:
    #         print('Press any key before you say something!')
    #         os.system('pause')
    #         rec_file = recorder.record(sec=5)
    #         text = stt(rec_file)
    #         publisher.publish(to_ros_msg(text))
    #         temp_publisher.publish(text)
    #         time.sleep(0.1)
            

        
