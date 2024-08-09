#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "../include/ship_detector/ship_tracker.hpp"
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ship_detector/circle_m.h>
#include <ship_detector/circle_part_m.h>
#include <std_msgs/Bool.h>


class  Vision_tracker
{
public:

     Vision_tracker()
     {
        //创建订阅者
        sleep(4);
        yolo_sub = nh.subscribe("/color_block_center",1,&Vision_tracker::vision_callback,this);
        local_position_sub = nh.subscribe<geometry_msgs::Pose>("/gazebo/zhihang/standard_vtol/local_position",1,&Vision_tracker::local_position_callback,this);
        start_tracking_sub = nh.subscribe("/ship",1,&Vision_tracker::start_tracking_callback,this);
        //创建发布者
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/xtdrone/standard_vtol_0/cmd_vel_flu",1);
        position_pub = nh.advertise<geometry_msgs::PoseStamped> ("/xtdrone/standard_vtol_0/mavros/setpoint_position/local", 10);


        // ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("/xtdrone/standard_vtol_/cmd",1);
        //调用服务
        set_mode_client=nh.serviceClient<mavros_msgs::SetMode>("/standard_vtol_0/mavros/set_mode");
        vision_find_ship = false; 
        Off_board_time = Time.toSec();  
        // flght_mode.request.custom_mode ="OFFBOARD";//一定要用双引号
        // set_mode_client.call(flght_mode);//调用
        // flght_mode.response.mode_sent;//回应
        // vision_find_ship_1 = true;

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
        start_tracking= msg;
    }

    void local_position_callback(geometry_msgs::Pose msg)
    {
        local_station=msg;
        
    }


     void vision_callback(ship_detector::circle_part_m data)
    {
        if(start_tracking.data)
        {
            // ros::ROS_INFO('here');//不能在汉书内部使用？
            float delta_x = (data.x-image_center_x); //delta_x控制左右 正左
            float delta_y = (data.y-image_center_y); //delta_y控制前后 正后
            float delta_S = (image_square-data.square);//delta_S控制上下 正下
            // std::cout <<delta_y << "  "<<delta_x<< std::endl;
            float x_velocity = delta_y * k_x;
            float y_velocity = delta_x * k_y;
            float z_velocity = - delta_S * k_s;
            // int time_num = land_times.size();

            // if(time_num <= 2 or (land_times[time_num-1]-land_times[time_num-4])>=0.6)
            // {
            if(data.square == 4)
            {
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
            if(delta_x*delta_x + delta_y*delta_y <=4000)
            {
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = -1.0;
                land_cal_time=Time.toSec();
                land_times.push_back(land_cal_time);
            }
            }
            else
            {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            }
            //     else
            //     {
            //         send_positon.pose.position.x=local_station.position.x;
            //         send_positon.pose.position.y=local_station.position.y;
            //         send_positon.pose.position.z=15;
            //         // position_pub.publish(send_positon);
            //     }   
            // else
            // {
            //     Stable_time = Time.toSec();  
            //     if (Stable_time-Off_board_time >= 3)
            //     {
            //         twist.linear.x = 0;
            //         twist.linear.y = 0;
            //         twist.linear.z = 0.0;
            //         flght_mode.request.custom_mode ="STABILZED";//逆天，原来这就是自动稳定
            //         set_mode_client.call(flght_mode);
            //         flght_mode.response.mode_sent;
            //         ROS_INFO("Not Found , STABILIZED");
            //     }
                        
            // }
                //为什么此处不能使用
                // std::cout << flght_mode.request.custom_mode << std::endl;
            if( local_station.position.z >= 10 and local_station.position.z <=16)
            {
                cmd_vel_pub.publish(twist);
                std::cout << local_station.position.z << std::endl;
                ROS_INFO("Vision_1_Is_Tracking");
            }
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
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
    ros::Time Time=ros::Time::now();
    std_msgs::Bool start_tracking;
    double Off_board_time;
    double Stable_time;
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
    ros::init(argc , argv , "ship_tracker");
    Vision_tracker vision_tracker;
    ros::Rate rate_10hz(20);
    while (ros::ok())
    {
        ros::spinOnce();
        rate_10hz.sleep();
    }
}



// // // void yolo_callback(yolov8_ros_msgs::BoundingBoxes data)
// // // {
// // //     yolov8_ros_msgs::BoundingBox target;
// // //     // ros::ROS_INFO('here');//不能在汉书内部使用？
// // //     target = data.bounding_boxes.back();
// // //     int Num_boxes_now=data.bounding_boxes.size();
// // //     if( Num_boxes_now != Num_boxes_past)
// // //     {
// // //         if(target.probability >= 80)
// // //         { 
// // //             mavros_msgs::SetMode flght_mode;
// // //             flght_mode.request.custom_mode = 'Hover';
// // //             set_mode_client.call(flght_mode);
// // //             flght_mode.response.mode_sent;
// // //             yolo_find_ship = true;
// // //             float delta_x = (image_center_x-(target.xmax+target.xmin)/2); //delta_x控制左右 正左
// // //             float delta_y = (image_center_y-(target.ymax+target.ymin)/2); //delta_y控制前后 正后
// // //             float delta_S = (image_square-((target.xmax-target.xmin)*(target.ymax-target.ymin)));//delta_S控制上下 正下
// // //             float x_velocity = -delta_y * k_x;
// // //             float y_velocity = delta_x * k_y;
// // //             float z_velocity = - delta_S * k_s;
// // //             twist.linear.x = x_velocity;
// // //             twist.linear.y = y_velocity;
// // //             twist.linear.z = z_velocity;
// // //             std::cout << 'here' << std::endl;
// // //             std::string cmd = "";
// // //             int Num_boxes_past = data.bounding_boxes.size();
// // //             ROS_INFO("is traccking"); //一定要用上双引号         
// // //         }
// // //     }
// // //     else
// // //     {
// // //         twist.linear.x = 0.0;
// // //         twist.linear.y = 0.0;
// // //         twist.linear.z = 0.0;
        
// // //         ROS_INFO("ENTER OFFBOARD_MODE");        
// // //     }
// // // }


// // //
// // //             print('find ship')
// // //             yolo_find_ship = True 
// // //             u = (target.xmax+target.xmin)/2
// // //             v = (target.ymax+target.ymin)/2
// // //             square= (target.xmax - target.xmin) * (target.ymax - target.ymin)
// // //             u_ = u-u_center
// // //             v_ = v-v_center
// // //             x_velocity = Kp_x * u_
// // //             y_velocity = Kp_y * v_
// // //             z_velocity = Kp_z * square
// // //             twist.linear.x = x_velocity
// // //             twist.linear.y = y_velocity
// // //             twist.linear.z = z_velocity
// // //             cmd = ''

    
        
// // // # def local_pose_callback(data):
// // // #     global height, target_height, target_set
// // // #     height = data.pose.position.z    
// // // #     if not target_set:
// // // #         target_height = height     
// // // #         target_set = True    

// // // # def cam_pose_callback(data):
// // // #     global theta
// // // #     q = Quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
// // // #     theta = q.yaw_pitch_roll[1]
            
// // int main(int argc , char **argv)
// // {
// //     //初始化
// //     ros::init(argc , argv , "Yolo_tracking");
// //     ros::Time::init();
// //     // flght_mode.request.custom_mode = "OFFBOARD";

// // //     //创建服务调用
// // //     ros::ServiceClient set_mode_client=nh.serviceClient<mavros_msgs::SetMode>("/standard_vtol_0/mavros/set_mode");

// // //     //创建订阅者
// // //     ros::Subscriber yolo_sub = nh.subscribe("/yolov8/BoundingBoxes",1,yolo_callback);

// // //     //创建发布者
// // //     ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/xtdrone/standard_vtol_0/cmd_vel_flu",1);
// // //     ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("/xtdrone/standard_vtol_/cmd",1);
// // // //     cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd', String, queue_size=1)


// // //     //飞行模式初始化
// // //     set_mode_client.call(flght_mode);
// // //     flght_mode.response.mode_sent;//client 端发送请求后会阻塞，直到 server 端返回结果才会继续执行。
// //     // ROS_INFO("ENTER OFFBOARD_MODE");
// //     // ROS_INFO("START YOLO_TRACKING!");
// //     Yolo_sub_pub Yolo_tracker;
// //     while (ros::ok())
// //     {
// //         ros::spinOnce();
        
// //         /* code */
// //     }
    

// //     //相关数据初始化
// //     // geometry_msgs::Twist twist;
// //     // twist.linear.x = 0;
// //     // twist.linear.y = 0;
// //     // twist.linear.z = 0;
// //     // bool yolo_find_ship = false;
// //         // ros::spinOnce();
// //     // ros::spin();
// //     // return 0;
// // }
// // //     # height = 0  
// // //     # target_height = 0
// // //     # target_set = False
// // //     # find_cnt = 0
// // //     # find_cnt_last = 0
// // //     # not_find_time = 0
// // //     twist = Twist()
// // //     cmd = String()
// // //     # theta = 0
// // //     u_center=1280/2 
// // //     v_center=720/2
// // //     yolo_find_ship = False
// // //     square_center = 1000
// // //     Kp_x = 0.5
// // //     Kp_y = 0.5
// // //     Kp_z = 1
// // //     vehicle_type = sys.argv[1]
// // //     vehicle_id = sys.argv[2]
// // //     rospy.init_node('yolo_human_tracking')
// // //     rospy.Subscriber("/yolov8/BoundingBoxes", BoundingBoxes, yolo_callback,queue_size=1)
// // //     # rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)
// // //     # rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cam_pose', PoseStamped, cam_pose_callback,queue_size=1)
// // //     
// // //     rate = rospy.Rate(60) 
// // //     while not rospy.is_shutdown():
// // //         rate.sleep()
// // //         cmd_vel_pub.publish(twist)
// // //         cmd_pub.publish(cmd)
// // //         # if find_cnt - find_cnt_last == 0:
// // //         #     if not get_time:
// // //         #         not_find_time = rospy.get_time()
// // //         #         get_time = True
// // //         #     if rospy.get_time() - not_find_time > 2.0:
// // //         #         twist.linear.x = 0.0
// // //         #         twist.linear.y = 0.0
// // //         #         twist.linear.z = 0.0
// // //         #         cmd = 'HOVER'
// // //         #         print(cmd)
// // //         #         get_time = False
// // //         # find_cnt_last = find_cnt
// // class SubscribeAndPublish  
// //     {  
// //     public:  
// //       SubscribeAndPublish()  
// //       {  
// //         //Topic you want to publish  
// //         ros::Publisher cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("/xtdrone/standard_vtol_0/cmd_vel_flu",1);
      
// //         //Topic you want to subscribe  
// //         ros::Subscriber yolo_sub = n_.subscribe("/zhihang/standard_vtol/angel",1,&SubscribeAndPublish::callback,this);  //注意这里，和平时使用回调函数不一样了。
// //       }  
      
// //       void callback(const geometry_msgs::Pose& input)  
// //       {  
// //         geometry_msgs::Twist output;
// //         std::cout << 1 << std::endl;  
// //         //.... do something with the input and generate the output...  
// //       }  
      
// //     private:  
// //       ros::NodeHandle n_;   
// //       ros::Publisher pub_;  
// //       ros::Subscriber sub_;  
      
// //     };//End of class SubscribeAndPublish  

    
// // }
// void yolo_callback(const yolov8_ros_msgs::BoundingBoxes& data)
//     {
//         yolov8_ros_msgs::BoundingBox target;
//         // ros::ROS_INFO('here');//不能在汉书内部使用？
//         target = data.bounding_boxes.back();
//         int Num_boxes_now=data.bounding_boxes.size();
//         std::cout << 'here' << std::endl;
//         if( Num_boxes_now != Num_boxes_past)//如果vector的数量没有发生变化，则保持悬停
//         {
//             if(target.probability >= 80)
//             { 
//                 flght_mode.request.custom_mode = 'OFFBOARD';
//                 set_mode_client.call(flght_mode);//调用
//                 flght_mode.response.mode_sent;//回应
//                 yolo_find_ship = true;
//                 float delta_x = (image_center_x-(target.xmax+target.xmin)/2); //delta_x控制左右 正左
//                 float delta_y = (image_center_y-(target.ymax+target.ymin)/2); //delta_y控制前后 正后
//                 float delta_S = (image_square-((target.xmax-target.xmin)*(target.ymax-target.ymin)));//delta_S控制上下 正下
//                 float x_velocity = -delta_y * k_x;
//                 float y_velocity = delta_x * k_y;
//                 float z_velocity = - delta_S * k_s;
//                 twist.linear.x = x_velocity;
//                 twist.linear.y = y_velocity;
//                 twist.linear.z = z_velocity;
//                 ROS_INFO("Is Traccking"); //一定要用上双引号         
//             }
//         }
//         else
//         {
//             twist.linear.x = 0.0;
//             twist.linear.y = 0.0;
//             twist.linear.z = 0.0;
//             flght_mode.request.custom_mode = 'HOVER';
//             set_mode_client.call(flght_mode);
//             flght_mode.response.mode_sent;
//             ROS_INFO("Not Found , Hover");        
//         }
//         //为什么此处不能使用
//         std::cout << "flght_mode" << std::endl;
//         cmd_vel_pub.publish(twist);
//     }
// int main(int argc, char **argv)  
// {  
//     //Initiate ROS  
//     ros::init(argc, argv, "subscribe_and_publish");  
//     ros::NodeHandle nh;
//     static mavros_msgs::SetMode flght_mode;
//     static geometry_msgs::Twist twist;

//     static bool yolo_find_ship = false;
    
//     //创建订阅者
//     ros::Subscriber yolo_sub = nh.subscribe("/yolov8/BoundingBoxes",1,yolo_callback);

//     //创建发布者
//     static ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/xtdrone/standard_vtol_0/cmd_vel_flu",1);

//     //调用服务
//     static ros::ServiceClient set_mode_client=nh.serviceClient<mavros_msgs::SetMode>("/standard_vtol_0/mavros/set_mode");

//     while(ros::ok)
//     {
//         ros::spinOnce();
//         cmd_vel_pub.publish(twist);
//     }


    
//     ros::spin();  
    
//     return 0;  
// }
// class Sub_pub 
// {
// public:
//     void yolo_callback(const yolov8_ros_msgs::BoundingBoxes msg)
//     {
//         std::cout << 1 << std::endl;
//     }
//     Sub_pub()
//     {
//         ros::NodeHandle nh;
//         yolo_sub = nh.subscribe("/yolov8/BoundingBoxes",1,&Sub_pub::yolo_callback,this); ////就是这里
//     }
    
// private:
//     ros::NodeHandle nh;
//     ros::Subscriber yolo_sub;
// }; 
// int main(int argc ,  char **argv)
// {
//     ros::init(argc, argv, "subscribe_and_publish"); 
//     Sub_pub sub;
//     ros::spin(); 
//     return 0;
// }

// class Sub_pub 
// {
// public:
//     void yolo_callback(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg)
//     {
//         std::cout << 1 << std::endl;
//     }
//     Sub_pub()
//     {
//         ros::NodeHandle nh;
//         yolo_sub = nh.subscribe("/yolov8/BoundingBoxes", 1, &Sub_pub::yolo_callback, this);
//     }
    
// private:
//     ros::NodeHandle nh;
//     ros::Subscriber yolo_sub;
// }; 

// int main(int argc ,  char **argv)
// {
//     ros::init(argc, argv, "subscribe_and_publish");
//     Sub_pub sub;
//     ros::spin();
//     return 0;
// }
