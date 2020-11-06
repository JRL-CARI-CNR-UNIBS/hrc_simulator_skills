#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <random>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mosaic_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group1="ur5_on_guide";
  std::string group2="human_right_arm";

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac1(group1+"/pick");
  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac1(group1+"/place");

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac2(group2+"/pick");
  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac2(group2+"/place");



  ros::ServiceClient resetClient = nh.serviceClient<std_srvs::Trigger>("/reset_scene");
  ros::ServiceClient resetBoxClient = nh.serviceClient<std_srvs::SetBool>("/inbound/reset_box");
  ros::ServiceClient resetOutboundClient = nh.serviceClient<std_srvs::SetBool>("/outbound/reset");
  ros::ServiceClient addObjsClient = nh.serviceClient<std_srvs::SetBool>("/inbound/add_objects");
  std_srvs::Trigger trigger_srv;
  std_srvs::SetBool bool_srv;
  bool_srv.request.data = true;

  ROS_INFO("Waiting for pick server");
  pick_ac1.waitForServer();
  pick_ac2.waitForServer();
  ROS_INFO("Connection ok");
  ROS_INFO("Waiting for place server");
  place_ac1.waitForServer();
  place_ac2.waitForServer();
  ROS_INFO("Connection ok");

  ros::Time t0=ros::Time::now();
  unsigned int trial=0;
  while (ros::ok())
  {
    ROS_INFO("Trial %u, time from start = %f seconds",trial++,(ros::Time::now()-t0).toSec());

    std::map<std::string,std::string> recipe1;
    if (!nh.getParam(group1+"/recipe",recipe1))
    {
      ROS_ERROR("Node %s has not a parameter named recipe",group1.c_str());
      return -1;
    }

    std::map<std::string,std::string> recipe2;
    if (!nh.getParam(group2+"/recipe",recipe2))
    {
      ROS_ERROR("Node %s has not a parameter named recipe",group2.c_str());
      return -1;
    }

    unsigned int nr=std::min(recipe1.size(),recipe2.size());
    for (unsigned int ir=0;ir<nr;ir++)
    {

      std::map<std::string,std::string>::iterator item1 = recipe1.begin();
      std::advance( item1,  rand() % recipe1.size() );
      manipulation_msgs::PickObjectsGoal pick_goal1;
      ROS_DEBUG("[Group %s] Goal: pick object %s and place it in slot %s",group1.c_str(),item1->second.c_str(),item1->first.c_str());
      pick_goal1.object_types.push_back(item1->second);
      pick_ac1.sendGoal(pick_goal1);


      std::map<std::string,std::string>::iterator item2 = recipe2.begin();
      std::advance( item2,  rand() % recipe2.size() );
      manipulation_msgs::PickObjectsGoal pick_goal2;
      ROS_DEBUG("[Group %s] Goal: pick object %s and place it in slot %s",group2.c_str(),item2->second.c_str(),item2->first.c_str());
      pick_goal2.object_types.push_back(item2->second);
      pick_ac2.sendGoal(pick_goal2);

      pick_ac1.waitForResult();
      pick_ac2.waitForResult();

      if (pick_ac1.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to pick -> object type =%s",group1.c_str(),item1->second.c_str());
      }

      if (pick_ac2.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to pick -> object type =%s",group2.c_str(),item2->second.c_str());
      }
      ROS_DEBUG("[Group %s] well done! I picked it, id=%s",pnh.getNamespace().c_str(),pick_ac1.getResult()->object_id.c_str());

      manipulation_msgs::PlaceObjectsGoal place_goal1;
      place_goal1.object_type=pick_ac1.getResult()->object_type;
      place_goal1.object_id=pick_ac1.getResult()->object_id;
      place_goal1.place_id=item1->first;
      place_ac1.sendGoal(place_goal1);

      manipulation_msgs::PlaceObjectsGoal place_goal2;
      place_goal2.object_type=pick_ac2.getResult()->object_type;
      place_goal2.object_id=pick_ac2.getResult()->object_id;
      place_goal2.place_id=item2->first;
      place_ac2.sendGoal(place_goal2);

      place_ac1.waitForResult();
      place_ac2.waitForResult();

      if (place_ac1.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to place, stop it",group1.c_str());
      }
      if (place_ac2.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to place, stop it",group2.c_str());
      }

      recipe1.erase(item1);
      recipe2.erase(item2);

    }
    resetClient.call(trigger_srv);
    resetBoxClient.call(bool_srv);
    resetOutboundClient.call(bool_srv);
    addObjsClient.call(bool_srv);
    ROS_INFO("RELOADED!");
  }

  ROS_INFO("[Group %s] pick client stopped",group1.c_str());
  return 0;
}
