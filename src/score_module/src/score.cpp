#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandVtolTransition.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <gazebo_msgs/ModelStates.h>

class Score
{
private:
    ros::NodeHandle nh;
    ros::Subscriber arrive_sub;
    ros::Subscriber ship_sub;
    ros::Subscriber thunder_sub;
    ros::Subscriber position_sub;
    ros::Subscriber target_sub;
    geometry_msgs::Pose true_target;

    std_msgs::Bool arrive_flag;
    geometry_msgs::Pose position;

public:
    Score()
    {
    arrive_sub = nh.subscribe<std_msgs::Bool>("/arrive",1,&Score::arrive_callback,this);
    ship_sub = nh.subscribe<std_msgs::Bool>("/ship",1,&Score::ship_callback,this);
    thunder_sub = nh.subscribe<geometry_msgs::Pose>("/zhihang/thunderstorm",1,&Score::thunder_callback,this);
    position_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,&Score::position_callback,this);
    target_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,&Score::target_callback,this);
    }

    void position_callback(gazebo_msgs::ModelStates msg)
    {

        position.position.x = msg.pose[13].position.x;
        position.position.y = msg.pose[13].position.y;

    }

    void target_callback(gazebo_msgs::ModelStates msg)
    {
        true_target.position.x = msg.pose[2].position.x;
        true_target.position.y = msg.pose[2].position.y;
    }

    void thunder_callback(geometry_msgs::Pose msg)
    {
        if (pow(position.position.x - msg.position.x,2)+pow(position.position.y - msg.position.y,2) <= 1000*1000)
        {
            ROS_ERROR("Thunder_Mission_Failed");
        }
    }

    void arrive_callback(std_msgs::Bool msg)
    {

        if (pow(position.position.x + 1200,2)+pow(position.position.y + 1200,2) >=9 )
        {
            ROS_ERROR("Arrive_Mission_Failed!");
        }
        else
        {
            ROS_INFO("Arrive_Mission_Success!");
        }



    }

    void ship_callback(std_msgs::Bool msg)
    {

        if (pow(position.position.x - true_target.position.x,2)+pow(position.position.y - true_target.position.y,2) >=100 )
        {
            ROS_ERROR("Ship_Mission_Failed!");
        }
        else
        {
            ROS_INFO("Ship_Misssion_Success!");
        }


    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "score_start");
    Score score;
    ros::spin();
    return 0;
}
