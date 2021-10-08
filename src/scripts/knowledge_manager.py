import json
from abc import *
from typing import Dict, Tuple

import time, os, copy
from glob import glob
from owlready2 import *
import re
import rospkg

PACKAGE_PATH = rospkg.RosPack().get_path('km')


def is_korean(text):
    kor_re = re.compile(r"[ㄱ-ㅣ가-힣]")
    return kor_re.search(text) is not None


class KnowledgeManager:

    def __init__(self,
                 owl_file: str = PACKAGE_PATH + '\owl_test\hyu_service.owl'):

        self.base_owl = owl_file
        self.base_dir = os.path.dirname(self.base_owl)
        self.prefix = self.getPrefix(self.base_owl)

        self.owlfile_list = glob(os.path.join(self.base_dir, '*.owl'))
        self.prefix_list = [self.getPrefix(o) for o in self.owlfile_list]

        self.onto_dict = dict()
        self.knowledge_base = self.load_ontology()

        self.knowledge_query = self.knowledge_query(self)
        self.knowledge_request = self.knowledge_request(self)
        self.knowledge_inference = self.knowledge_inference(self)

    def load_ontology(self):

        for _owlfile in self.owlfile_list:
            prefix = self.getPrefix(_owlfile)

            while True:
                try:
                    self.onto_dict[prefix] = get_ontology(_owlfile).load()
                    setattr(self, prefix, self.onto_dict[prefix])
                    break
                except:
                    time.sleep(0.1)

        knowledge_base = self.onto_dict[self.prefix]
        knowledge_base.imported_ontologies = [self.onto_dict[self.getPrefix(owlfile)] for owlfile in self.owlfile_list]
        
        for i in knowledge_base.individuals():
            i.get_properties()

        # 추가 namespace
        self.onto_dict['knowrob'] = knowledge_base.get_namespace("http://knowrob.org/kb/knowrob.owl#")
        self.onto_dict['owl'] = knowledge_base.get_namespace("http://www.w3.org/2002/07/owl#")
        self.onto_dict['rdf'] = knowledge_base.get_namespace("http://www.w3.org/1999/02/22-rdf-syntax-ns#")

        return knowledge_base

    def getPrefix(self,
                  owlfile: str) -> str:
        return os.path.splitext(os.path.split(owlfile)[-1])[0]

    def preprocess_spo(self,
                       spo: dict) -> dict:
        sparql_spo = copy.deepcopy(spo)
        vars = set()

        for k, v in spo.items():
            if "?" in v:
                vars.add(v)
            elif is_korean(v):
                sparql_spo[k] = '<{}>'.format(self.knowledge_base.search(label=v)[0].iri)
            else:
                sparql_spo[k] = '<{}>'.format(self.knowledge_base.search(iri="*"+v)[0].iri)

        triple = ' '.join([sparql_spo['s'], sparql_spo['p'], sparql_spo['o']])

        return vars, triple

    def postprocess_spo(self,
                        query: list,
                        query_result: list) -> list:
        result_list = list()

        for spo in query:
            spo_dict = spo.copy()
            for k, v in spo.items():
                if "?" in v:
                    spo_dict[k] = query_result[int(v[1:])]
            result_list.append(spo_dict)

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

            simple_query_result = list()

            query_string = "SELECT " + " ".join(vars_list) + " WHERE { " + ' . '.join(where_phrase) + " }"
            sparql_result_list = list(default_world.sparql(query_string))

            for sparql_result in sparql_result_list:
                for i in range(len(sparql_result)):
                    if isinstance(sparql_result[i], str):
                        continue
                    sparql_result[i] = sparql_result[i].name

                simple_query_result.append(self.km.postprocess_spo(data, sparql_result))
            print(json.dumps(simple_query_result, ensure_ascii=False, indent=4))
            return simple_query_result

        def face_recognition(self,
                             data: list) -> list:
            for d in data:
                f_id = d['face_id']
                # print(self.km.knowledge_base)
                user = self.km.knowledge_base.search(faceID=f_id)[0]
                sc = self.social_context([{"target":user.name}])
                d.update(target=sc[0]['target'])
                d.update(social_context=sc[0]['social_context'])

            return data

        def social_context(self,
                           data: list) -> list:
            for d in data:
                user = d['target']

                if isinstance(user, str):
                    user = getattr(self.km.knowledge_base, user)

                sc = dict()
                sc['name'] = user.fullName[0]
                sc['age'] = user.isAged[0].label[0]
                sc['gender'] = user.gender.label[0]
                sc['appellation'] = user.hasAppellation[0]
                sc['help_avail'] = user.help_avail[0]
                sc['visitFreq'] = user.visitFreq[0]
                sc['sleep_status'] = user.sleepStatus[0]
                sc['meal_menu'] = user.haveAMeal[0]
                ms = user.hasMedicalStatus[0]
                sc['disease_name'] = ms.getDisease[0].label[0]
                sc['medical_checkup'] = self.medical_checkup(user)
                d.update(social_context=sc)

            return data

        def medical_checkup(self,
                            user):

            ms = user.hasMedicalStatus[0]
            bp_systolic = float(ms.systolicBloodPressureLevel[0])
            bp_diastolic = float(ms.diastolicBloodPressureLevel[0])
            bs = float(ms.bloodSugarLevel[0])
            cl = float(ms.cholesterolLevel[0])

            if bp_systolic > float(self.km.onto_dict['isro_medical']._SystolicBloodPressureLevel.upperLimit[0]):
                bp_systolic_range = "high"
            elif bp_systolic < float(self.km.onto_dict['isro_medical']._SystolicBloodPressureLevel.lowerLimit[0]):
                bp_systolic_range = "low"
            else:
                bp_systolic_range = "normal"

            if bp_diastolic > float(self.km.onto_dict['isro_medical']._DiastolicBloodPressureLevel.upperLimit[0]):
                bp_diastolic_range = "high"
            elif bp_diastolic < float(self.km.onto_dict['isro_medical']._DiastolicBloodPressureLevel.lowerLimit[0]):
                bp_diastolic_range = "low"
            else:
                bp_diastolic_range = "normal"

            if cl > float(self.km.onto_dict['isro_medical']._CholesterolLevel.upperLimit[0]):
                cl_range = "high"
            elif cl < float(self.km.onto_dict['isro_medical']._CholesterolLevel.lowerLimit[0]):
                cl_range = "low"
            else:
                cl_range = "normal"

            if bs > float(self.km.onto_dict['isro_medical']._BloodSugarLevel.upperLimit[0]):
                bs_range = "high"
            elif bs < float(self.km.onto_dict['isro_medical']._BloodSugarLevel.lowerLimit[0]):
                bs_range = "low"
            else:
                bs_range = "normal"

            med_check = dict()
            med_check['systolic_blood_pressure'] = {"range":bp_systolic_range}
            med_check['diastolic_blood_pressure'] = {"range": bp_diastolic_range}
            med_check['cholesterol_level'] = {"range": cl_range}
            med_check['blood_sugar_level'] = {"range": bs_range}

            return med_check

    class knowledge_request:

        def __init__(self, knowledge_manager):
            self.km = knowledge_manager

        def create(self,
                   data: list):

            for d in data:
                subj = d['subject']
                
                for _, ont in self.km.onto_dict.items():
                    if ont[subj] is not None:
                        subj = ont[subj]

                # 인디비주얼 생성
                new_i = subj(namespace=self.km.knowledge_base)

                # RecordRelatedEvent의 하위클래스인 경우에는 시간 정보 저장
                if subj in list(self.km.onto_dict['isro'].RecordRelatedEvent.subclasses()):
                    time_point = self.km.onto_dict['knowrob'].TimePoint(namespace=self.km.knowledge_base,
                                                                        name='TimePoint_' + str(time.time()))
                    new_i.startTime.append(time_point)

                # 새로 생성된 인디비주얼에 predicate 추가
                for predicate in d['predicate']:

                    if is_korean(predicate['p']):
                        p = self.km.knowledge_base.search(label=predicate['p'])[0]
                    else:
                        p = self.km.knowledge_base.search(iri="*"+predicate['p'])[0]

                    if type(p) == ObjectPropertyClass:
                        if is_korean(predicate['o']):
                            o = self.km.knowledge_base.search(label=predicate['o'])[0]
                        else:
                            o = self.km.knowledge_base.search(iri="*" + predicate['o'])[0]
                    elif type(p) == DataPropertyClass:
                        o = predicate['o']
                    else:
                        raise ValueError

                    getattr(new_i, p.name).append(o)

            self.km.knowledge_base.save(file=self.km.base_owl, format='rdfxml')

            return None

        def update(self,
                   data: list):
            for d in data:
                subj = d['subject']
                for _, ont in self.km.onto_dict.items():
                    if ont[subj] is not None:
                        old_i = ont[subj]

                for predicate in d['predicate']:

                    if is_korean(predicate['p']):
                        p = self.km.knowledge_base.search(label=predicate['p'])[0]
                    else:
                        p = self.km.knowledge_base.search(iri="*"+predicate['p'])[0]

                    if type(p) == ObjectPropertyClass:
                        if is_korean(predicate['o']):
                            o = self.km.knowledge_base.search(label=predicate['o'])[0]
                        else:
                            o = self.km.knowledge_base.search(iri="*" + predicate['o'])[0]
                    elif type(p) == DataPropertyClass:
                        o = predicate['o']
                    else:
                        raise ValueError

                    if len(getattr(old_i, p.name)) > 0:
                        getattr(old_i, p.name).pop(0)
                    getattr(old_i, p.name).append(o)


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

            user = self.km.knowledge_base.search(label=data['user'])[0]  # 이은수
            event_cls = self.km.knowledge_base.search(label=data['event'])[0]  # MedicalRecord
            time_point = data['timepoint']

            if time_point == 'previous':
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
                            except ValueError:
                                triple['p'] = prop.name
                            
                            try:
                                triple['o'] = getattr(event, prop.name)[0].label[0]
                            except ValueError:
                                triple['o'] = getattr(event, prop.name)[0].name
                        
                            result.append(triple)
            
            elif time_point == 'next':
                pass
            
            data['result'] = result

            return data


