# Autor: Samuele Sandrini

# This script is usefull for generate the task_properties collection based on 
# slots_distribution and object_distribution yaml file
from json.tool import main

import yaml
from yaml.loader import SafeLoader
import sys

from pymongo import MongoClient

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
END = '\033[0m'
TASK_PROPERTIES_AVAILABLE = ["splitted_pickplace",          #pick_white_box, place_white_box
                             "place_based_on_group_name",   #pick_white_box, place_robot_group 
                             "place_based_on_slots_name"]   #pick_white_box, place_AR1
ERROR_MSG_CONFIG_NOT_PRESENT = RED + "{} not defined in config file" + END
ROBOT_AGENT_NAME = "ur5_on_guide"
HUMAN_AGENT_NAME = "human_right_arm"
ROBOT_SLOTS_GROUP_NAME = "robot_group"
HUMAN_SLOTS_GROUP_NAME = "human_group"

def getAgent(slot_group):
    print(slot_group)
    if slot_group==ROBOT_SLOTS_GROUP_NAME:
        return [ROBOT_AGENT_NAME]
    elif slot_group==HUMAN_SLOTS_GROUP_NAME:
        return [HUMAN_AGENT_NAME]
    else:
        return [ROBOT_AGENT_NAME,HUMAN_AGENT_NAME]    
def getAgentFromInbound(inbound):
    if inbound=="robot_desk1":
        return [ROBOT_AGENT_NAME]
    elif inbound=="human_desk":
        return [HUMAN_AGENT_NAME]
    else:
        return [ROBOT_AGENT_NAME,HUMAN_AGENT_NAME]  
def getSlotsGroupFromAgent(agent):
    slots_group = []
    if ROBOT_AGENT_NAME in agent:
        slots_group.append(ROBOT_SLOTS_GROUP_NAME)
    if HUMAN_AGENT_NAME in agent:
        slots_group.append(HUMAN_SLOTS_GROUP_NAME)
    return slots_group
def main():
    # Type of storage based on available one
    storage_type = TASK_PROPERTIES_AVAILABLE[0]
    
    # File name
    file_name_slot = "slots_distribution.yaml"
    db_name = "agents_synergy"
    coll_task_properties="tasks_properties_test"
    file_name_box = "objects_distribution.yaml"
    
    # Connection to db
    client = MongoClient()
    db = client[db_name]
    coll_skills = db[coll_task_properties]
    coll_skills.delete_many({})
    
    # Slots distribution data file / Object distribution data file
    with open(file_name_slot) as f:
        data_slots_distribution = yaml.load(f, Loader=SafeLoader)
    with open(file_name_box) as f:
        data_objects_distribution = yaml.load(f, Loader=SafeLoader)
    # Retrieve objects distribution
    if "inbound" in data_objects_distribution:
        if "objects" in data_objects_distribution["inbound"]:
            objects_distribution = data_objects_distribution["inbound"]["objects"]
        else:
            print(ERROR_MSG_CONFIG_NOT_PRESENT.format("objects"))
            return 0
    else:
        print(ERROR_MSG_CONFIG_NOT_PRESENT.format("inbound"))
        return 0       
    # Retrieve slots and slots_group        
    if "outbound" in data_slots_distribution:
        if "slots_group" in data_slots_distribution["outbound"]:
            slots_group = data_slots_distribution["outbound"]["slots_group"]
        else:
            print(ERROR_MSG_CONFIG_NOT_PRESENT.format("slots_group"))
            return 0
        if "slots" in data_slots_distribution["outbound"]:
            slots = data_slots_distribution["outbound"]["slots"]
        else:
            print(ERROR_MSG_CONFIG_NOT_PRESENT.format("slots"))
            return 0 
    else:
        print(ERROR_MSG_CONFIG_NOT_PRESENT.format("outbound"))
        return 0             
    
       
    if storage_type == TASK_PROPERTIES_AVAILABLE[0]:
        for object in set([tuple([d["type"],d["inbound"]]) for d in objects_distribution]):   #if i have same object type belong to same inbound only 1 task properties is created
            object_type = object[0]
            object_inbound = object[1]
            print(object)

            agent = getAgentFromInbound(object_inbound)
            coll_skills.insert_one({"name":"pick_"+object_type,"type":"pick","description":"","agent":agent,"goal":[object_type]})
            print(agent)
            print(getSlotsGroupFromAgent(agent))
            if len(agent)>1:
                for single_agent in agent:
                    coll_skills.insert_one({"name":"place_"+object_type+"_"+single_agent,"type":"place","description":"","agent":[single_agent],"goal": getSlotsGroupFromAgent(single_agent)})
            else:    
                coll_skills.insert_one({"name":"place_"+object_type,"type":"place","description":"","agent":agent,"goal": getSlotsGroupFromAgent(agent)})
            input("Giro dopo...")
    elif storage_type == TASK_PROPERTIES_AVAILABLE[1]:
        #Iterate all object type (one for type)
        for object in set([tuple([d["type"],d["inbound"]]) for d in objects_distribution]):   #if i have same object type belong to same inbound only 1 task properties is created
            object_type = object[0]
            object_inbound = object[1]
            print(object)
            agent = getAgentFromInbound(object_inbound)
            coll_skills.insert_one({"name":"pick_"+object_type,"type":"pick","description":"","agent":agent,"goal":[object_type]})
        #Iterate all slots group for all place_*slot_group*
        for slot in slots_group:
            if "name" in slot:
                coll_skills.insert_one({"name":"place_"+slot["name"],"type":"place","description":"","agent":getAgent(slot["name"]),"goal": [slot["name"]]})
            else:
                print(ERROR_MSG_CONFIG_NOT_PRESENT.format("slot name"))
                return 0
            
    elif storage_type == TASK_PROPERTIES_AVAILABLE[2]:
        #Iterate all object type (one for type)
        for object in set([tuple([d["type"],d["inbound"]]) for d in objects_distribution]):   #if i have same object type belong to same inbound only 1 task properties is created
            object_type = object[0]
            object_inbound = object[1]
            print(object)
            agent = getAgentFromInbound(object_inbound)
            coll_skills.insert_one({"name":"pick_"+object_type,"type":"pick","description":"","agent":agent,"goal":[object_type]})
            for slot in data_slots_distribution["outbound"]["slots"]:
                print(slot)
                if "name" in slot and "slots_group" in slot:
                    print("dentro")
                    coll_skills.insert_one({"name":"place_"+slot["name"],"type":"place","description":"","agent":getAgent(slot["slots_group"]),"goal":[slot["slots_group"]]})
                else:
                    print(ERROR_MSG_CONFIG_NOT_PRESENT.format("slot name"))
                    return 0
       
       
       
       
       
    # # Place task properties based on slots group. Ex: place_robot_group, place_human_group with right agent
    # if "outbound" in data_slots_distribution:
    #     if "slots_group" in data_slots_distribution["outbound"]:
    #         for slot in data_slots_distribution["outbound"]["slots_group"]:
    #             #print(slot)
    #             if "name" in slot:
    #                 #print(getAgent(slot))
    #                 #print(slot["name"])
    #                 coll_skills.insert_one({"name":"place_"+slot["name"],"type":"place","description":"","agent":getAgent(slot["name"]),"goal": [slot["name"]]})

    # #Place to single objects . Ex place_AR1 with agent according to slot_group          
    # # if "outbound" in data_slots_distribution:
    # #     print(data_slots_distribution)
    # #     if "slots" in data_slots_distribution["outbound"]:
    # #         print(data_slots_distribution)
    # #         for slot in data_slots_distribution["outbound"]["slots"]:
    # #             print(slot)
    # #             if "name" in slot and "slots_group" in slot:
    # #                 print("dentro")
    # #                 coll_skills.insert_one({"name":"place_"+slot["name"],"type":"place","description":"","agent":getAgent(slot["slots_group"]),"goal":slot["slots_group"]})
    
    # # input("seconda parte")

    
    # if "inbound" in data_objects_distribution:
    #     if "objects" in data_objects_distribution["inbound"]:
    #         for object in set([tuple([d["type"],d["inbound"]]) for d in data_objects_distribution["inbound"]["objects"]]):   #if i have same object type belong to same inbound onli 1 task properties is created
    #             object_type = object[0]
    #             object_inbound = object[1]
    #             print(object)
    #             # print([object[0]])
    #             # print([str(object[0])])
    #             coll_skills.insert_one({"name":"pick_"+object_type,"type":"pick","description":"","agent":getAgentFromInbound(object_inbound),"goal":[object_type]})

    
if __name__ == "__main__":
    main()