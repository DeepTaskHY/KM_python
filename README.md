# 1. [M2-8] Social Ontology Model

## 2. package summary 

Social ontology model consists of two parts: 1) social ontology, 2) knowledge manager. Social ontology is an ontology that defines social concepts required for intelligent social service robots. Knowledge manager deals with the knowledge of social ontology. It can maintain the knowledge of social ontology consistent and can provide that knowledge to other agents if necessary.

- 2.1 Maintainer status: maintained
- 2.2 Maintainer: Gunhee Cho, [freebeinq@gmail.com]()
- 2.3 Author: Gunhee Cho, [freebeinq@gmail.com]()
- 2.4 License (optional): 
- 2.5 Source git: https://github.com/DeepTaskHY/KM_python

## 3. Overview

In order for a service robot to provide an intelligent social service, it is necessary to be able to plan actions suitable for the social service situation based on knowledge of the social concept. Our social ontology has been expanded by adding social concepts to the overall service robot ontology. This ontology not only includes knowledge required for general services such as user, robot profile, perception, environment, and robot behavior, but also knowledge essential in social robot services such as emotional status, goals and intentions of user speech, and social status of users.

![ontology_structure](./figure/onto_structure.png)

Knowledge manager is an agent that manages the knowledge defined in social ontology and serves as an interface between other agents and social ontology. At this time, the information provided to other agents from social ontology includes not only predefined information, but also newly produced information through knowledge processing by knowledge manager.



## 4. Hardware requirements

None




## 5. Quick start

### 5.1 Environment

- **Ubuntu** 20.04
- **ROS** noetic
- **python** 3.x

### 5.1 Configuration & run

- Run python

  ```bash
  cd /km/scripts
  python launcher.py
  ```

- Roslaunch

  ```bash
  roslaunch km km_launcher.launcher
  ```

  

## 6. Input/Subscribed Topics

### 6.1 Message protocol

#### ROS topic

- /taskExecution (std_msgs/String)

#### Message format

```json
{
    "header": {
        "timestamp": $TimeStamp,
        "source": "planning",
        "target": ["knowledge"],
        "content": [$Operator],
		"id": $ID
    },
    $Operator: {
        "timestamp": $TimeStamp,
        "type": $OperationType,
        "data": $OperationData
    }
}
```

> The basic message follows **json** format. It contains two keys, "header" and *$Operator*. "header" contains the meta information of this ROS message. *$Operator* is a variable that can be determined as one of the following three values according to the operation performed by Knowledge Manager: "knowledge_request", "knowledge_query", and "knowledge_inference".

- **header**
  - **timestamp**: published time
  - **source**: publishing module
  - **target**: receiving module
  - **content**: role of this message
  - **id**: message id
- **$Operator**
  - **knowledge_request**: The knowledge_request operator can be used when requesting to change (write) the information stored in the ontology, and Knowledge manager does not return any message.
  - **knowledge_query**: The knowledge_query operator can be used to request the information itself stored in the ontology, and Knowledge manager searches and returns the query object.
  - **knowledge_inference**: The knowledge_inference operator can be used to infer new information based on the knowledge stored in the ontology, and Knowledge manager stores the new information generated through inference in the ontology and returns the corresponding content.

### 6.2 Message examples

> **knowledge_request**
>
> > **create**
> >
> > ```
> > {
> >     "header":{
> >     	"timestamp": $TimeStamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_request"],
> > 		"id": $ID   
> >     },
> >     "knowledge_request":{
> >         "timestamp": $TimeStamp,
> >         "type": "create",
> >         "data": [
> >             {
> >                 "subject": "SpeechRecord",
> >                 "predicate":[
> >                     {
> >                         "p": "speakTo",
> >                         "o": "Silbot001"
> >                     },
> >                     {
> >                         "p": "hasSpeaker",
> >                         "o": "Person001"
> >                     },
> >                     {
> >                         "p": "hasContents",
> >                         "o": "안녕"
> >                     },
> >                     {
> >                         "p": "containIntention",
> >                         "o": "_GreetingIntention"
> >                     }
> >                 ]
> >             }
> >         ]
> >     }    
> > }
> > ```
> >
> > To create a new individual of a specific type class in the ontology and create a relationship through the predicate in the new instance, send a "knowledge_request" message of "create" type. 
> >
> > - **data**: a list of dictionaries containing "subject" and "predicate".
> >   - **subject** : a name of the ontology class corresponding to the type of individual to be newly created.
> >   - **predicate** : a list of dictionaries containing "p" and "o", which mean a property and an object respectively. 
> >     - If "p" is an object property, "o" must be an individual existing in the ontology. 
> >     - If "p" is a data property, "o" is literal value data
>
> > **update**
> >
> > ```json
> > {
> >     "header":{
> >     	"timestamp": $timestamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_request"],
> > 		"id": $id   
> >     },
> >     "knowledge_request":{
> >         "timestamp": $timestamp,
> >         "type": "update",
> >         "data": [
> >             {
> >                 "subject": "Person001",
> >                 "predicate":[
> >                     {
> >                         "p": "sleepStatus",
> >                         "o": "positive"
> >                     }
> >                 ]
> >             }
> >         ]
> >     }    
> > }
> > ```
> >
> > To update the relationships of an individual that already exist in the ontology, send a "knowledge_request" message of "update" type.
> >
> > - **data**: a list of dictionaries containing "subject" and "predicate".
> >   - **subject**: the target individual to be updated the relationship. It must already exist in the ontology.
> >   - **predicate** : a list of dictionaries containing "p" and "o", which mean a property and an object respectively. 
> >     - If "p" is an object property, "o" must be an individual existing in the ontology. 
> >     - If "p" is a data property, "o" is literal value data.
>
> > **delete**
> >
> > ```json
> > {
> >     "header":{
> >     	"timestamp": $timestamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_request"],
> > 		"id": $id   
> >     },
> >     "knowledge_request":{
> >         "timestamp": $timestamp,
> >         "type": "delete",
> >         "data": [
> >             {
> >                 "subject": $ontology.individual,
> >                 "predicate":[
> >                     {
> >                         "p": $ontology.objectproperty,
> >                         "o": $ontology.individual
> >                     },
> >                     {
> >                         "p": $ontology.dataproperty,
> >                         "o": $ontology.value
> >                     }
> >                 ]
> >             }
> >         ]
> >     }    
> > }
> > ```
> >
> > To delete the relationships of an individual that already exist in the ontology or delete the individual itself, send a "knowledge_request" message of "delete" type.
> >
> > - **data**: a list of dictionaries containing "subject" and "predicate".
> >   - **subject**: the target individual to be deleted the relationship or to be deleted itself. It must already exist in the ontology.
> >   - **predicate** : a list of dictionaries containing "p" and "o", which mean a property and an object respectively. 
> >     - If "p" is an object property, "o" must be an individual existing in the ontology. 
> >     - If "p" is a data property, "o" is literal value data.

> **knowledge_query**
>
> > **simple_query**
> >
> > ```json
> > {
> >     "header": {
> >         "timestamp": $TimeStamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_query"],
> >         "id": $ID
> >     },
> >     "knowledge_query": {
> >         "timestamp": $TimeStamp,
> >         "type": "simple_query",
> >         "data": [
> >             {
> >                 "s":"Person001",
> >                 "p":"hasMedicalStatus",
> >                 "o":"?0"
> >             },
> >             {
> >                 "s":"?0",
> >                 "p":"relatedDisease",
> >                 "o":"?1"
> >             },
> >             {
> >                 "s":"?1",
> >                 "p":"medicalAdvice",
> >                 "o":"?2"
> >             }
> >         ]
> >     }
> > }
> > ```
> >
> > To query the triple-based relationship of individual existing in the ontology, send a "knowledge_query" message of the "simple_query" type.
> >
> > - **data**: a list of dictionaries containing "s", "p", and "o".
> >   - "s", "p", and "o" mean "subject", "property", and "object", respectively, and they are the basic structural units that make up the triple-based representing the relationship in the ontology.
> >   - Among "s", "p", and "o", the target to be obtained through a query should be expressed as an index, and a "?" symbol should be added in front of the index to indicate that it is a variable.
> >   - As in the example message above, queries can be nested.
>
> > **social_context**
> >
> > ```json
> > {
> >     "header": {
> >         "timestamp": $TimeStamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_query"],
> >         "id": $ID
> >     },
> >     "knowledge_query": {
> >         "timestamp": $TimeStamp,
> >         "type": "social_context",
> >         "data": [
> >             {
> >                 "target":"Person001"
> >             } 
> >         ]
> >     }
> > }
> > ```
> >
> > To query all social contexts of a specific user, send a "knowledge_query" message of the "social_context" type.
> >
> > - **data**: a list of dictionaries containing "target".
> >   - **target**: a target of the social context query
>
> > **face_recognition**
> >
> > ```json
> > {
> >     "header": {
> >         "timestamp": $TimeStamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_query"],
> >         "id": $ID
> >     },
> >     "knowledge_query": {
> >         "timestamp": $TimeStamp,
> >         "type": "face_recognition",
> >         "data": [
> >             {
> >                 "face_id": 1
> >             } 
> >         ]
> >     }
> > }
> > ```
> >
> > To query the user's social context through the user id identified as a result of face recognition by the robot, send a "knowledge_query" message of "face_recognition" type.
> >
> > - **data**: a list of dictionaries containing "face_id".
> >   - **face_id**: user id identified by face recognition

> **knowledge_inference**
>
> > **entity**
> >
> > ```json
> > ```
> >
> > (in progress)

## 7. Output/Published Topics

### 7.1 Message protocol

#### ROS topic

- /taskCompletion (std_msgs/String)

#### Message format

```json
{
    "header": {
        "timestamp": $TimeStamp,
        "source": "knowledge",
        "target": ["planning"],
        "content": [$Operator],
		"id": $ID
    },
    $Operator: {
        "timestamp": $TimeStamp,
        "type": $OperationType,
        "data": $OperationData
    }
}
```

> The basic message follows **json** format. It contains two keys, "header" and *$Operator*. "header" contains the meta information of this ROS message. *$Operator* is a variable that can be determined as one of the following three values according to the operation performed by Knowledge Manager: "knowledge_request", "knowledge_query", and "knowledge_inference".

- **header**
  - **timestamp**: published time
  - **source**: publishing module
  - **target**: receiving module
  - **content**: role of this message
  - **id**: message id
- **$Operator**
  - **knowledge_request**: The knowledge_request operator can be used when requesting to change (write) the information stored in the ontology, and Knowledge manager does not return any message.
  - **knowledge_query**: The knowledge_query operator can be used to request the information itself stored in the ontology, and Knowledge manager searches and returns the query object.
  - **knowledge_inference**: The knowledge_inference operator can be used to infer new information based on the knowledge stored in the ontology, and Knowledge manager stores the new information generated through inference in the ontology and returns the corresponding content.

### 7.2 Message examples

> **knowledge_request**
>
> > **create**
> >
> > There is no return message.
>
> > **update**
> >
> > There is no return message.
>
> > **delete**
> >
> > There is no return message.

> **knowledge_query**
>
> > **simple_query**
> >
> > ```json
> > {
> >     "header": {
> >         "timestamp": $TimeStamp,
> >         "source": "planning",
> >         "target": ["knowledge"],
> >         "content": ["knowledge_query"],
> >         "id": $ID
> >     },
> >     "knowledge_query": {
> >         "timestamp": $TimeStamp,
> >         "type": "simple_query",
> >         "data": [
> >             [
> >                 {
> >                     "s": "Person001",
> >                     "p": "hasMedicalStatus",
> >                     "o": "MedicalRecord_1631989900.1036713"
> >                 },
> >                 {
> >                     "s": "MedicalRecord_1631989900.1036713",
> >                     "p": "relatedDisease",
> >                     "o": "_HighBloodPressure"
> >                 },
> >                 {
> >                     "s": "_HighBloodPressure",
> >                     "p": "medicalAdvice",
> >                     "o": "금연"
> >                 }
> >             ],
> >             [
> >                 {
> >                     "s": "Person001",
> >                     "p": "hasMedicalStatus",
> >                     "o": "MedicalRecord_1631989900.1036713"
> >                 },
> >                 {
> >                     "s": "MedicalRecord_1631989900.1036713",
> >                     "p": "relatedDisease",
> >                     "o": "_HighBloodPressure"
> >                 },
> >                 {
> >                     "s": "_HighBloodPressure",
> >                     "p": "medicalAdvice",
> >                     "o": "유산소 운동"
> >                 }
> >             ]
> >         ]
> >     }
> > }
> > ```
>
> > **social_context**
> >
> > ```json
> > {
> >     "header": {
> >         "timestamp": $TimeStamp,
> >         "source": "knowledge",
> >         "target": ["planning"],
> >         "content": ["knowledge_query"],
> >         "id": $ID
> >     },
> >     "knowledge_query": {
> >         "timestamp": $TimeStamp,
> >         "type": "social_context",
> >         "data": [
> >             {
> >                 "target": "Person001",
> >                 "social_context": {
> >                     "name": "이병현",
> >                     "age": "노인",
> >                     "gender": "남성",
> >                     "appellation": "어르신",
> >                     "help_avail": true,
> >                     "visitFreq": 2,
> >                     "sleep_status": "positive",
> >                     "meal_menu": "설렁탕",
> >                     "disease_name": "고혈압",
> >                     "medical_checkup": {
> >                         "systolic_blood_pressure": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         },
> >                         "diastolic_blood_pressure": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         },
> >                         "blood_sugar_level": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         },
> >                         "cholesterol_level": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         }
> >                     }
> >                 }
> >             }
> >         ]
> >     }
> > }
> > ```
>
> > **face_recognition**
> >
> > ```json
> > {
> >     "header": {
> >         "timestamp": $TimeStamp,
> >         "source": "knowledge",
> >         "target": ["planning"],
> >         "content": ["knowledge_query"],
> >         "id": $ID
> >     },
> >     "knowledge_query": {
> >         "timestamp": $TimeStamp,
> >         "type": "face_recognition",
> >         "data": [
> >             {
> >                 "face_id": 1,
> >                 "target": "Person001",
> >                 "social_context": {
> >                     "name": "이병현",
> >                     "age": "노인",
> >                     "gender": "남성",
> >                     "appellation": "어르신",
> >                     "help_avail": true,
> >                     "visitFreq": 2,
> >                     "sleep_status": "positive",
> >                     "meal_menu": "설렁탕",
> >                     "disease_name": "고혈압",
> >                     "medical_checkup": {
> >                         "systolic_blood_pressure": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         },
> >                         "diastolic_blood_pressure": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         },
> >                         "blood_sugar_level": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         },
> >                         "cholesterol_level": {
> >                             "state": "decreasing",
> >                             "range": "normal"
> >                         }
> >                     }
> >                 }
> >             }
> >         ]
> >     }
> > }
> > ```

> **knowledge_inference**
>
> > **entity**
> >
> > ```json
> > 
> > 
> > ```
> >
> > (in progress)

## 8. Parameters

## 9. Related Applications (Optional)

## 10. Related Publications (Optional)

