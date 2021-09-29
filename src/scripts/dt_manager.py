from abc import *
from typing import Dict, Tuple

import time, os, copy
from glob import glob
from owlready2 import *
import re
from dtroslib.helpers import get_package_path

test_path = get_package_path('km')
# test_path = '..'


def is_korean(text):
    kor_re = re.compile(r"[ㄱ-ㅣ가-힣]")
    return kor_re.search(text) is not None

class KnowledgeManager:

    def __init__(self,
                 owl_file: str = test_path + '/owl2/hyu_service.owl'):
        self.base_owl = owl_file
        self.base_dir = os.path.dirname(self.base_owl)
        self.owlfile_list = glob(os.path.join(self.base_dir, '*.owl'))

        self.onto_dict = dict()
        self.knowledge_base = self.load_ontology()

        self.knowledge_query = self.knowledge_query(self)
        self.knowledge_request = self.knowledge_request(self)
        self.knowledge_inference = self.knowledge_inference(self)


    def load_ontology(self):
        for _owlfile in self.owlfile_list:
            
            while True:
                try:
                    self.onto_dict[_owlfile] = get_ontology(_owlfile).load()
                    break
                except:
                    time.sleep(0.1)

        knowledge_base = self.onto_dict[self.base_owl]
        knowledge_base.imported_ontologies = [self.onto_dict[owlfile] for owlfile in self.owlfile_list]
        
        for i in knowledge_base.individuals():
            i.get_properties()

        return knowledge_base


    def preprocess_spo(self,
                       spo: dict) -> dict:
        sparql_spo = copy.deepcopy(spo)
        vars = set()

        for k, v in spo.items():
            if "?" in v:
                vars.add(v)
            elif is_korean(v):
                sparql_spo[k] = '<{}>'.format(self.knowledge_base.search(label=v)[0].iri)

        triple = ' '.join([sparql_spo['s'], sparql_spo['p'], sparql_spo['o']])

        return vars, triple


    def postprocess_spo(self,
                        query: list,
                        query_result: list) -> list:
        result_list = list()

        for spo in query:
            for k, v in spo.items():
                if "?" in v:
                    spo[k] = query_result[int(v[1:])]
            result_list.append(spo)

        return result_list
    

    class knowledge_query:


        def __init__(self, knowledge_manager):
            self.km = knowledge_manager


        def simple_query(self, 
                         data: list) -> list:

            vars_list = list()
            where_phrase = list()

            for d in data:

                vars, triple = self.km.preprocess_spo(d)

                for var in vars:
                    if var not in vars_list:
                        vars_list.append(var)

                where_phrase.append(triple)

            query_string = "SELECT " + " ".join(vars_list) + " WHERE { " + ' . '.join(where_phrase) + " }"
            query_result = [entity.label[0] for entity in list(default_world.sparql(query_string))[0]]

            result = self.km.postprocess_spo(data, query_result)

            return result
        

        def face_recognition(self,
                             data: list) -> list:
            for d in data:
                f_id = d['face_id']
                print(self.km.knowledge_base)
                user = self.km.knowledge_base.search(faceID=f_id)[0]


                d['social_context']['name'] = user.fullName[0]
                d['social_context']['age'] = user.isAged[0].label[0]
                d['social_context']['appellation'] = user.hasAppellation[0]
                d['social_context']['disease_name'] = user.getDisease[0].label[0]
                d['social_context']['help_avail'] = user.help_avail[0]

            return data


    class knowledge_request:


        def __init__(self, knowledge_manager):
            self.km = knowledge_manager


        def create(self,
                   data: list):
            for d in data:
                cls = d['class'] # MedicalRecord
                
                for _, ont in self.km.onto_dict.items():
                    if ont[cls] is not None:
                        subj = ont[cls]
                
                new_i = subj(namespace=self.km.knowledge_base)

                for prop in d['property']:
                    
                    o = prop['o']

                    if is_korean(prop['p']):
                        p = self.km.knowledge_base.search(label=prop['p'])[0]


                    if '_' in prop['o']:    
                        c = self.km.knowledge_base.search(iri='*'+prop['o'][1:])[0]
                        o = c(namespace=self.km.knowledge_base, name=prop['o'][1:]+'_'+str(time.time()))
                    elif is_korean(prop['o']):
                        o = self.km.knowledge_base.search(label=prop['o'])[0]
                    
                    getattr(new_i, p.python_name).append(o)
            
            self.km.knowledge_base.save(file=self.km.base_owl, format='rdfxml')

            return None


        def update(self,
                   data: list):

            self.km.knowledge_base.save(file=self.km.base_owl, format='rdfxml')

            return None


        def delete(self,
                   data: list):

            self.km.knowledge_base.save(file=self.km.base_owl, format='rdfxml')

            return None


    class knowledge_inference:


        def __init__(self, knowledge_manager):
            self.km = knowledge_manager


        def entity(self,
                   data: dict) -> list:

            result = list()

            user = self.km.knowledge_base.search(label=data['user'])[0] # 이은수
            event_cls = self.km.knowledge_base.search(label=data['event'])[0] # MedicalRecord
            timepoint = data['timepoint']

            if timepoint == 'previous':
                events = event_cls.instances()
                for event in events:
                    event_time = float(event.startTime[0].name.split('_')[-1])
                    if time.time() - event_time > 0:
                        
                        s = event.name
                        props = event.get_properties()
                        for prop in props:
                            triple = dict()
                            triple['s'] = event.name

                            try:
                                triple['p'] = prop.label[0]
                            except:
                                triple['p'] = prop.name
                            
                            try:
                                triple['o'] = getattr(event, prop.name)[0].label[0]
                            except:
                                triple['o'] = getattr(event, prop.name)[0].name
                        
                            result.append(triple)
            
            elif timepoint == 'next':
                pass
            
            data['result'] = result

            return data


