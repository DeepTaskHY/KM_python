#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from playsound import playsound

import requests, urllib


def tts(arg):
    json_msg = arg.data
    text = json_msg['robot_speech']['text'] # 이거는 json 포맷 정해지면 수정
    text = json_msg
    rospy.loginfo('[TTS] got a new message : {}'.format(text))

    encoded_text = urllib.parse.quote(text)
    data = 'speaker=mijin&speed=0&text=' + encoded_text

    url = 'https://naveropenapi.apigw.ntruss.com/voice/v1/tts'

    # url = "https://openapi.naver.com/v1/voice/tts.bin"
    # request = urllib.request.Request(url)
    # request.add_header("X-Naver-Client-Id", client_id)
    # request.add_header("X-Naver-Client-Secret", client_secret)
    # response = urllib.request.urlopen(request, data=data.encode('utf-8'), headers)
    # rescode = response.getcode()

    client_id = ''  # 인증 정보의 Client ID
    client_secret = ''  # 인증 정보의 Client Secret

    headers = {
        'Content-Type': 'application/x-www-form-urlencoded',  # Fix
        'X-NCP-APIGW-API-KEY-ID': client_id,
        'X-NCP-APIGW-API-KEY': client_secret,
    }

    response = requests.post(url, data=data, headers=headers)
    rescode = response.status_code

    if rescode == 200:
        # print("TTS mp3 저장")
        # print(response.json())
        response_body = response.content
        speech_file = 'data/robot_speech.mp3'
        with open(speech_file, 'wb') as f:
            f.write(response_body)
        playsound(speech_file)

    else:
        rospy.loginfo('Error Code:{}'.format(rescode))


if __name__ == '__main__':

    rospy.init_node('tts_node')
    rospy.loginfo('Start TTS')
    rospy.Subscriber('/action/speech', String, tts)
    rospy.spin()
