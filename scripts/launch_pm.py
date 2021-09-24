import rospy
from std_msgs.msg import String

import json, time, threading

from rosnode import NodeBase

_human_speech: str = ''
_robot_speech: str = ''
_speech_1: bool = False
_vision_1: bool = False
_face_id: int
_timepoint: float
_name: str
_social_context: dict

_km_id = 0


def callback_speech(message):
    global _speech_1, _human_speech

    received_message = json.loads(message.data)
    rospy.loginfo('Received message: \n{}'.format(json.dumps(message, ensure_ascii=False, indent="\t")))

    header = received_message['header']
    _human_speech = received_message['human_speech']['stt']

    if header['id'] == 1:    
        
        t = threading.Thread(target=scenario_1)
        t.start()

        _speech_1 = True

        return

    if header['id'] == 2:    

        if '약' in _human_speech and '먹' in _human_speech:
            json_file = '../json-example/7-p-k.json'
            msg = json.load(open(json_file, 'r'))
            
            pm_node.publish('/taskExecution', json.dumps(msg, ensure_ascii=False, indent='\t'))
            rospy.loginfo('Published message: \n{}'.format(json.dumps(msg, ensure_ascii=False, indent="\t")))

        return

    if header['id'] == 3:
        json_file = '../json-example/10-p-d.json'
        msg = json.load(open(json_file, 'r'))
        msg['dialog_generation']['human_speech'] = _human_speech
        msg['dialog_generation']['social_context'] = _social_context

        pm_node.publish('/taskExecution', json.dumps(msg, ensure_ascii=False, indent='\t'))
        rospy.loginfo('Published message: \n{}'.format(json.dumps(msg, ensure_ascii=False, indent="\t")))
        return

    return



def callback_vision(message):
    global _vision_1, _face_id, _timepoint
    received_message = json.loads(message.data)
    rospy.loginfo('Received message: \n{}'.format(json.dumps(message, ensure_ascii=False, indent="\t")))

    header = received_message['header']

    if header['id'] > 1:    
        
        _face_id = received_message['face_recognition']['face_id']
        _timepoint = received_message['face_recognition']['timestamp']
        
        _vision_1 = True
        return

    return



def callback_task(message):
    global _km_id, _dm_id, _name, _social_context, _human_speech, _robot_speech

    received_message = json.loads(message.data)
    header = received_message['header']
    source = header['source']
    
    
    if 'planning' not in header['target']:
        return
    
    rospy.loginfo('Received message: \n{}'.format(json.dumps(message, ensure_ascii=False, indent="\t")))

    if source == 'knowledge':
        _km_id = header['id']
        if _km_id == 1:

            _social_context = received_message['knowledge_query']['data']['social_context']
            _name = _social_context['name']

            json_file = '../json-example/4-p-d.json'
            msg = json.load(open(json_file, 'r'))
            msg['dialog_generation']['human_speech'] = _human_speech
            msg['dialog_generation']['social_context'] = _social_context

            pm_node.publish('/taskExecution', json.dumps(msg, ensure_ascii=False, indent='\t'))
            rospy.loginfo('Published message: \n{}'.format(json.dumps(msg, ensure_ascii=False, indent="\t")))
            
            return
        
    if source == 'dialog':
        _dm_id = header['id']
        
        content_key = received_message['header']['content'][0]
        _robot_speech = received_message[content_key]['dialog']

        tar = ['tts']
        con_name = ['robot_speech']
        con = {'text':_robot_speech}
        msg = pm_msg_generator(id, tar, con_name, con)
        
        pm_node.publish('/action/speech', json.dumps(msg, ensure_ascii=False, indent='\t'))
        rospy.loginfo('Published message: \n{}'.format(json.dumps(msg, ensure_ascii=False, indent="\t")))

        return

    return



def scenario_1():
    global _speech_1, _vision_1

    while True:
        if _speech_1 and _vision_1:
            t = threading.Thread(target=scenario_2)
            t.start()
            _speech_1 = False
            break
        else:
            time.sleep(0.1)

    return



def scenario_2():
    global _face_id, _timepoint
    json_file = '../json-example/2-p-k.json'
    msg = json.load(open(json_file, 'r'))
    msg['knowledge_query']['data']['face_id'] = _face_id
    msg['knowledge_query']['data']['timestamp'] = _timepoint
    
    pm_node.publish('/taskExecution', json.dumps(msg, ensure_ascii=False, indent='\t'))
    rospy.loginfo('Published message: \n{}'.format(json.dumps(msg, ensure_ascii=False, indent="\t")))

    return



def pm_msg_generator(id, targets, content_names, contents):

    json_msg = {
        'header': {
                'timestamp': str(time.time()),
                'source': 'plannig',
                'target': targets,
                'content': content_names,
                'id': id,
        }
    }
    json_msg.update(contents)

    return json_msg




if __name__ == '__main__':
    pm_node = NodeBase('pm_node')

    pm_node.add_publisher("/taskExecution", String, queue_size=10)
    pm_node.add_publisher("/action/speech", String, queue_size=10)
    pm_node.add_subscriber("/recognition/speech", String, callback_speech)
    pm_node.add_subscriber("/recognition/face", String, callback_vision)
    pm_node.add_subscriber("/taskCompletion", String, callback_task)

    pm_node.spin()
