#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <random>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <thread>


class Executer
{
public:
  Executer(std::string group_name):
    group_name_(group_name)
  {
    pnh=ros::NodeHandle(group_name);
  }
  void executeRecipe()
  {
    actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac1(group_name_+"/pick");
    actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac1(group_name_+"/place");

    ROS_INFO("Waiting for pick server");
    pick_ac1.waitForServer();
    ROS_INFO("Connection ok");
    ROS_INFO("Waiting for place server");
    place_ac1.waitForServer();
    ROS_INFO("Connection ok");


    std::map<std::string,std::string> recipe1;
    if (!pnh.getParam("recipe",recipe1))
    {
      ROS_ERROR("Node %s has not a parameter named recipe",group_name_.c_str());
      return;
    }

    unsigned int nr=recipe1.size();
    for (unsigned int ir=0;ir<nr;ir++)
    {
      if (!ros::ok())
        return;

      std::map<std::string,std::string>::iterator item1 = recipe1.begin();
      std::advance( item1,  rand() % recipe1.size() );
      manipulation_msgs::PickObjectsGoal pick_goal1;
      ROS_DEBUG("[Group %s] Goal: pick object %s and place it in slot %s",group_name_.c_str(),item1->second.c_str(),item1->first.c_str());
      pick_goal1.object_types.push_back(item1->second);
      pick_ac1.sendGoal(pick_goal1);


      pick_ac1.waitForResult();

      if (pick_ac1.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to pick -> object type =%s",group_name_.c_str(),item1->second.c_str());
      }
      ROS_DEBUG("[Group %s] well done! I picked it, id=%s",pnh.getNamespace().c_str(),pick_ac1.getResult()->object_id.c_str());

      manipulation_msgs::PlaceObjectsGoal place_goal1;
      place_goal1.object_type=pick_ac1.getResult()->object_type;
      place_goal1.object_id=pick_ac1.getResult()->object_id;
      place_goal1.place_id=item1->first;
      place_ac1.sendGoal(place_goal1);

      place_ac1.waitForResult();

      if (place_ac1.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to place, stop it",group_name_.c_str());
      }

      recipe1.erase(item1);

    }

  }
protected:
  std::string group_name_;
  ros::NodeHandle pnh;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mosaic_test");
  ros::NodeHandle nh;
   ros::AsyncSpinner spinner(4);
  spinner.start();

  Executer r1("ur5_on_guide");
  Executer r2("human_right_arm");


  ros::ServiceClient resetClient = nh.serviceClient<std_srvs::Trigger>("/reset_scene");
  ros::ServiceClient resetBoxClient = nh.serviceClient<std_srvs::SetBool>("/inbound/reset_box");
  ros::ServiceClient resetOutboundClient = nh.serviceClient<std_srvs::SetBool>("/outbound/reset");
  ros::ServiceClient addObjsClient = nh.serviceClient<std_srvs::SetBool>("/inbound/add_objects");
  std_srvs::Trigger trigger_srv;
  std_srvs::SetBool bool_srv;
  bool_srv.request.data = true;

  ros::Time t0=ros::Time::now();
  unsigned int trial=0;
  while (ros::ok())
  {
    ROS_INFO("Trial %u, time from start = %f seconds",trial++,(ros::Time::now()-t0).toSec());


    std::thread thread_1 = std::thread(&Executer::executeRecipe, &r1);
    std::thread thread_2 = std::thread(&Executer::executeRecipe, &r2);

    // do other stuff
    if (thread_2.joinable())
      thread_2.join();
    if (thread_1.joinable())
      thread_1.join();

    resetClient.call(trigger_srv);
    resetBoxClient.call(bool_srv);
    resetOutboundClient.call(bool_srv);
    addObjsClient.call(bool_srv);
    ROS_INFO("RELOADED!");
  }

  ROS_INFO("asincro execution stopped");
  return 0;
}
