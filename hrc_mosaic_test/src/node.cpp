#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_place_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac("inbound_pick");
  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac("outbound_pallet/place");

  ROS_INFO("Waiting for pick server");
  pick_ac.waitForServer();
  ROS_INFO("Connection ok");
  ROS_INFO("Waiting for place server");
  place_ac.waitForServer();
  ROS_INFO("Connection ok");

  std::vector<std::string> objects;
  objects.push_back("blue_box");
  objects.push_back("blue_box");
  objects.push_back("orange_box");
  objects.push_back("blue_box");
  objects.push_back("orange_box");
  objects.push_back("blue_box");
  objects.push_back("blue_box");
  objects.push_back("orange_box");
  objects.push_back("blue_box");
  objects.push_back("orange_box");

  for (unsigned int ipick=0;ipick<objects.size();ipick++)
  {
    manipulation_msgs::PickObjectsGoal pick_goal;
    pick_goal.object_types.push_back(objects.at(ipick));

    pick_ac.sendGoalAndWait(pick_goal);


    if (pick_ac.getResult()->result<0)
    {
      ROS_ERROR("unable to pick -> object type =%s",objects.at(ipick).c_str());
      continue;
    }
    ROS_INFO("well done! I picked it, id=%s",pick_ac.getResult()->object_id.c_str());

    manipulation_msgs::PlaceObjectsGoal place_goal;
    place_goal.object_type=pick_ac.getResult()->object_type;
    place_goal.object_id=pick_ac.getResult()->object_id;
    place_ac.sendGoalAndWait(place_goal);

    if (place_ac.getResult()->result<0)
    {
      ROS_ERROR("unable to place");
      return 0;
    }

  }
  ROS_INFO("pick client stopped");
  return 0;
}
