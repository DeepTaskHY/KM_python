from abc import *
from typing import Dict, Tuple
import time, json

import rospy
from std_msgs.msg import String

from knowledge_manager import KnowledgeManager
from dtroslib.ros import DTNode


# Node of Knowledge Manager
class KMNode(DTNode):

    def __init__(self,
                 *args, **kwargs):

        super(KMNode, self).__init__(publish_message='/taskCompletion',
                                     subscribe_message='/taskExecution',
                                     *args, **kwargs)

        self.source_name = 'knowledge'
        self.knowledge_manager = KnowledgeManager()
        rospy.loginfo('Start KM')

    def generate_content(self,
                         source: str,
                         content_names: list,
                         contents: Dict[str, dict]) -> Tuple[list, list, Dict[str, dict]]:

        targets = [source]
        generated_content_names = list()
        generated_contents = dict()

        for content_name in content_names:
            generated_content_names.append(content_name)
            content = contents[content_name]

            operator = getattr(self.knowledge_manager, content_name)

            operation_type = content['type']
            operation_data = content['data']
            operation_timestamp = content['timestamp']

            content['data'] = getattr(operator, operation_type)(operation_data, operation_timestamp)
            generated_contents[content_name] = content

        return targets, generated_content_names, generated_contents

