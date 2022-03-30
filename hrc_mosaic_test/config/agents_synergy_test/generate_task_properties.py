# Autor: Samuele Sandrini

# This script is usefull for generate the task_properties collection based on 
# slots_distribution and object_distribution yaml file
from json.tool import main

import yaml
from yaml.loader import SafeLoader
import sys

from pymongo import MongoClient

def getAgent(slot_group):
    print(slot_group)
    if slot_group=="robot_group":
        return ["ur5_on_guide"]
    elif slot_group=="human_group":
        return ["human_right_arm"]
    else:
        return ["ur5_on_guide","human_right_arm"]    
def getAgentFromInbound(inbound):
    if inbound=="robot_desk1":
        return ["ur5_on_guide"]
    elif inbound=="human_desk":
        return ["human_right_arm"]
    else:
        return ["ur5_on_guide","human_right_arm"]  
def main():
    file_name_slot = "slots_distribution.yaml"
    db_name = "agents_synergy"
    coll_task_properties="tasks_properties"
    file_name_box = "objects_distribution.yaml"

    client = MongoClient()
    db = client[db_name]
    coll_skills = db[coll_task_properties]
    #coll_skills.delete_many({})
    
    # Place task
    with open(file_name_slot) as f:
        data = yaml.load(f, Loader=SafeLoader)
    
    if "outbound" in data:
        if "slots_group" in data["outbound"]:
            for slot in data["outbound"]["slots_group"]:
                print(slot)
                if "name" in slot:
                    print(getAgent(slot))
                    print(slot["name"])
                    coll_skills.insert_one({"name":"place_"+slot["name"],"type":"place","description":"","agent":getAgent(slot["name"]),"goal": [slot["name"]]})

    #Place to single objects           
    # if "outbound" in data:
    #     print(data)
    #     if "slots" in data["outbound"]:
    #         print(data)
    #         for slot in data["outbound"]["slots"]:
    #             print(slot)
    #             if "name" in slot and "slots_group" in slot:
    #                 print("dentro")
    #                 coll_skills.insert_one({"name":"place_"+slot["name"],"type":"place","description":"","agent":getAgent(slot["slots_group"]),"goal":slot["slots_group"]})
    
    with open(file_name_box) as f:
        data = yaml.load(f, Loader=SafeLoader)
    
    if "inbound" in data:
        if "objects" in data["inbound"]:
            for object in set([tuple([d["type"],d["inbound"]]) for d in data["inbound"]["objects"]]):
                print(object)
                print([object[0]])
                print([str(object[0])])
                coll_skills.insert_one({"name":"pick_"+object[0],"type":"pick","description":"","agent":getAgentFromInbound(object[1]),"goal":[object[0]]})
    
    
if __name__ == "__main__":
    main()