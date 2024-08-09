#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "../include/ship_detector/ship_land_helper.hpp"
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ship_detector/circle_m.h>
#include <ship_detector/circle_part_m.h>
#include <unistd.h>
#include <std_msgs/Bool.h>


class  Vision_tracker
{
public:

     Vision_tracker()
     {
        //创建订阅者
        sleep(4);
        yolo_sub = nh.subscribe("/H_block_center",1,&Vision_tracker::vision_callback,this);
        start_tracking_sub = nh.subscribe("/ship",1,&Vision_tracker::start_tracking_callback,this);
        local_position_sub = nh.subscribe<geometry_msgs::Pose>("/gazebo/zhihang/standard_vtol/local_position",1,&Vision_tracker::local_position_callback,this);
        //创建发布者
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/xtdrone/standard_vtol_0/cmd_vel_flu",1);
        position_pub = nh.advertise<geometry_msgs::PoseStamped> ("/xtdrone/standard_vtol_0/mavros/setpoint_position/local", 10);

        Start_time = Time_l.toSec();  


        // ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("/xtdrone/standard_vtol_/cmd",1);
        //调用服务
        set_mode_client=nh.serviceClient<mavros_msgs::SetMode>("/standard_vtol_0/mavros/set_mode");
        vision_find_ship = false; 
        // flght_mode.request.custom_mode ="OFFBOARD";//一定要用双引号 //为什么切换会往下掉落
        // set_mode_client.call(flght_mode);//调用
        // flght_mode.response.mode_sent;
        //相关系数初始化
        //     u_center=1280/2 
//     v_center=720/2
//     yolo_find_ship = False
//     square_center = 1000
//     Kp_x = 0.5
//     Kp_y = 0.5
//     Kp_z = 1
     }

     void start_tracking_callback(std_msgs::Bool msg)
     {
        start_tracking = msg;
     }
    void local_position_callback(geometry_msgs::Pose msg)
    {
        local_station=msg;
        
    }


     void vision_callback(ship_detector::circle_part_m data)
    {

        // ros::ROS_INFO('here');//不能在汉书内部使用？
        if (start_tracking.data)
        {
        float delta_x = (data.x-image_center_x);
        float delta_y = (data.y-image_center_y);

        float x_velocity = delta_y * k_x;
        float y_velocity = delta_x * k_y;
        // twist.linear.x = 0;
        // twist.linear.y = 0;
        // twist.linear.z = 0;

        if(delta_y*delta_y >= delta_x*delta_x)
        {
            twist.linear.x = 0;
            twist.linear.y = x_velocity;
            twist.linear.z = 0;
        }
        else
        {
            twist.linear.x = -y_velocity;
            twist.linear.y = 0;
            twist.linear.z = 0;
        }
        // if(delta_x*delta_x + delta_y*delta_y <=2000)
        // {
        //     twist.linear.x = 0;
        //     twist.linear.y = 0;
        //     twist.linear.z = -0.2;
        // }
        // std::cout << local_station.position.z << std::endl;
        if(local_station.position.z >=4.8)
        {
            if(delta_x*delta_x + delta_y*delta_y <=2000)
            {
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = -0.5;
            }

        }
        else
        {
            if(delta_x*delta_x + delta_y*delta_y <=500)
            {
                while (ros::ok())
                {
                flght_mode.request.custom_mode ="AUTO.LAND";//一定要用双引号 //为什么切换会往下掉落
                set_mode_client.call(flght_mode);//调用
                flght_mode.response.mode_sent;
                }

                // while(local_station.position.z >=2.5)
                // {
                //     twist.linear.x = 0;
                //     twist.linear.y = 0;
                //     twist.linear.z = 0;
                // }
            }
        }
        if(local_station.position.z <= 10)
        {
            cmd_vel_pub.publish(twist);
            std::cout << local_station.position.z << std::endl;
            ROS_INFO("Vision_Is_Tracking");
        }
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        std::cout << flght_mode.request.custom_mode << std::endl;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber yolo_sub ;
    ros::Subscriber state_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber start_tracking_sub;
    ros::Publisher position_pub;
    ros::Publisher cmd_vel_pub ;
    ros::Publisher cmd_pub ;
    ros::ServiceClient set_mode_client;
    geometry_msgs::Twist twist;
    mavros_msgs::SetMode flght_mode;
    geometry_msgs::Pose local_station;
    ros::Time Time_l;
    std_msgs::Bool start_tracking;
    double Off_board_time;
    double Start_time;
    double land_cal_time;
    std::vector<double> land_times;
    bool vision_find_ship_1;
    //相关系数初始化
    //  u_center=1280/2 
//     v_center=720/2
//     yolo_find_ship = False
//     square_center = 1000
//     Kp_x = 0.5
//     Kp_y = 0.5
//     Kp_z = 1
    
};
int main(int argc , char **argv)
{
    //初始化
    ros::init(argc , argv , "ship_land_helper");
    Vision_tracker vision_tracker;
    ros::Rate rate_10hz(20);
    ros::spin();
}

