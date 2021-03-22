#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mosaic_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group_name="ur5_on_guide";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("Node %s has not a parameter named group_name",pnh.getNamespace().c_str());
    return -1;
  }

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac(group_name+"/pick");
  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac(group_name+"/place");

  ROS_INFO("Waiting for pick server");
  pick_ac.waitForServer();
  ROS_INFO("Connection ok");
  ROS_INFO("Waiting for place server");
  place_ac.waitForServer();
  ROS_INFO("Connection ok");

  std::map<std::string,std::string> recipe;
  if (!pnh.getParam("recipe",recipe))
  {
    ROS_ERROR("Node %s has not a parameter named group_name",pnh.getNamespace().c_str());
    return -1;
  }
  std::map<std::string,bool> done_recipe;
  for (const std::pair<std::string,std::string>& object: recipe)
  {
    done_recipe.insert(std::pair<std::string,bool>(object.first,false));
  }

  for (unsigned int pick_itrial=0;pick_itrial<10;pick_itrial++)
  {
    bool all_done=true;
    for (const std::pair<std::string,std::string>& object: recipe)
    {
      if (done_recipe.at(object.first))
        continue;

      manipulation_msgs::PickObjectsGoal pick_goal;
      ROS_INFO("[Group %s] Goal: pick object %s and place it in slot %s",pnh.getNamespace().c_str(),object.second.c_str(),object.first.c_str());

      pick_goal.object_types.push_back(object.second);

      pick_ac.sendGoalAndWait(pick_goal);


      if (pick_ac.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to pick -> object type =%s",pnh.getNamespace().c_str(),object.second.c_str());
        all_done=false;
        continue;
      }
      ROS_INFO("[Group %s] well done! I picked it, id=%s",pnh.getNamespace().c_str(),pick_ac.getResult()->object_id.c_str());
      done_recipe.at(object.first)=true;

      manipulation_msgs::PlaceObjectsGoal place_goal;
      place_goal.object_type=pick_ac.getResult()->object_type;
      place_goal.object_id=pick_ac.getResult()->object_id;

      place_goal.place_id.push_back(object.first);
      place_ac.sendGoalAndWait(place_goal);

      if (place_ac.getResult()->result<0)
      {

        ROS_ERROR("[Group %s] unable to place, stop it",pnh.getNamespace().c_str());
        return 0;


      }


    }
    if (all_done)
      break;
  }


  ROS_INFO("[Group %s] pick client stopped",pnh.getNamespace().c_str());
  return 0;
}
