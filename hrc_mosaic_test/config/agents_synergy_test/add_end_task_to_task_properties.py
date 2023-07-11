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

def getHomeLocation():
    # Slots distribution data file / Object distribution data file
    with open("locations_distribution.yaml") as f:
        locations_distribution_file = yaml.load(f, Loader=SafeLoader)

    locations_name_list = []
    # Retrieve objects distribution
    if "go_to_location" in locations_distribution_file:
        locations = locations_distribution_file["go_to_location"]
        for location in locations:
            if "name" in location:
               locations_name_list.append(location["name"])
    return list(filter(lambda location_name_str: "home" in location_name_str, locations_name_list))

def main():
    # File name
    db_name = "agents_synergy"
    coll_task_properties="tasks_properties_test"
    
    # Connection to db
    client = MongoClient()
    db = client[db_name]
    coll_skills = db[coll_task_properties]
    

            

    coll_skills.insert_one({"name":"end","type":"goto","description":"","agent":[ROBOT_AGENT_NAME,HUMAN_AGENT_NAME],"goal":getHomeLocation()})

       
       
       
    
if __name__ == "__main__":
    main()