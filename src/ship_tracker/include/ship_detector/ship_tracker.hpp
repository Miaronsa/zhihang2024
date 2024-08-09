#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/String.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

bool vision_find_ship;
float image_center_x = 500;
float image_center_y = 360;
float image_square = 90000;
float k_x = 0.008;                // while(local_station.position.z >=2.5)
                // {
                //     twist.linear.x = 0;
                //     twist.linear.y = 0;
                //     twist.linear.z = 0;
                // }
float k_y = 0.004;
float k_s = 0.0005;//0.00001;
int Num_boxes_now;
int Num_boxes_past;



