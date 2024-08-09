#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandVtolTransition.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
class UAVController {
private:
    ros::NodeHandle nh;

    ros::Subscriber angle_sub;
    ros::Subscriber angle_sub2;
    ros::Subscriber state_sub;
    ros::Subscriber position_sub;

    ros::Publisher setpoint_pub;
    ros::Publisher TARGET_pub;

    ros::ServiceClient client_set_mode;
    ros::ServiceClient client_transition;

    mavros_msgs::State current_state;
    double need_angle;
    double need_angle2;
    bool angle_received = false;

    const double distance_parameter = 20.0;

    geometry_msgs::Pose uav_position;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose x;
    geometry_msgs::Pose target;
    double current_yaw = 0.0;
    bool Rflag ;

public:
    UAVController() {
        angle_sub = nh.subscribe("/need_angle", 10, &UAVController::angleCallback, this);
        angle_sub2 = nh.subscribe("/zhihang/standard_vtol/angel", 10, &UAVController::angleCallback2, this);

        state_sub = nh.subscribe("/standard_vtol_0/mavros/state", 10, &UAVController::stateCallback, this);
        position_sub = nh.subscribe("/gazebo/zhihang/standard_vtol/local_position", 10, &UAVController::positionCallback, this);
        setpoint_pub = nh.advertise<geometry_msgs::Pose>("/xtdrone/standard_vtol_0/cmd_pose_enu", 10);
        TARGET_pub = nh.advertise<geometry_msgs::Pose>("//target_final", 10);

        client_set_mode = nh.serviceClient<mavros_msgs::SetMode>("/standard_vtol_0/mavros/set_mode");
        client_transition = nh.serviceClient<mavros_msgs::CommandVtolTransition>("/standard_vtol_0/mavros/cmd/vtol_transition");
    }

    void positionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        uav_position = *msg;
        tf::Quaternion quat(uav_position.orientation.x, uav_position.orientation.y, uav_position.orientation.z, uav_position.orientation.w);
        double roll, pitch;
        tf::Matrix3x3(quat).getRPY(roll, pitch, current_yaw);
    }

    void angleCallback(const std_msgs::Float64::ConstPtr& msg) {
        //need_angle = msg->data;
        angle_received = true;
    }

    void angleCallback2(const geometry_msgs::Pose::ConstPtr& msg) {
        x = *msg;
        need_angle2=x.position.x;
        
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    void waitForAngle() {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok() && !angle_received) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Angle received: %f", need_angle);
    }

    void ensureQuadMode() {
        if (current_state.mode != "OFFBOARD") {
            mavros_msgs::CommandVtolTransition transition_srv;
            transition_srv.request.state = mavros_msgs::CommandVtolTransition::Request::STATE_MC;
            if (client_transition.call(transition_srv) && transition_srv.response.success) {
                ROS_INFO("Transition to MC mode successful");
            } else {
                ROS_ERROR("Failed to transition to quad mode");
            }
        }
    }

    void publishTargetPose() {
    target_pose.position.x = -3500;
    target_pose.position.y = -3500;
    target_pose.position.z = 20.0;  // 固定高度


    static double yaw_angle = 0.0;  // 初始yaw角度
    double yaw_increment = 0.03;  // 每次发布的角度增量（弧度）
 

    
      tf::Quaternion quat;
    quat.setRPY(0, 0, yaw_angle);
    target_pose.orientation.x = quat.x();
    target_pose.orientation.y = quat.y();
    target_pose.orientation.z = quat.z();
    target_pose.orientation.w = quat.w();

    setpoint_pub.publish(target_pose);

    // 增加yaw角度
    yaw_angle += yaw_increment;
    if (yaw_angle >= 2 * M_PI) {
        yaw_angle -= 2 * M_PI;  // 确保角度在0到2*pi之间
    }

    setpoint_pub.publish(target_pose);


 }

    void rotateAndCheckAngle() {
        ros::Rate rate(10);  // 10 Hz

        while (ros::ok()) {
            ros::spinOnce();

            double error = fabs(need_angle2);
            //ROS_INFO("Angle error: %f,Current yaw: %f", need_angle2,current_yaw);
            if (error < 0.02&&error > 0) {  // 判断 need_angle 是否接近 0
                ROS_INFO("Angle is close to zero. Stopping rotation. Current yaw: %f", current_yaw);
                if(current_yaw>0){
                    if(current_yaw<1.57){
                    ROS_INFO("1st");
                    target.position.x=-1000-2312;
                    target.position.y=-900-2309;
                    }else if(current_yaw>1.57){
                    ROS_INFO("2ed");
                    target.position.x=-1400-2312;
                    target.position.y=-900-2309;
                    }


                }else if (current_yaw<0){
                    if(current_yaw>-1.57){
                    ROS_INFO("3rd");
                    target.position.x=-1000-2312;
                    target.position.y=-1500-2309;
                    }else if(current_yaw<-1.57){
                    ROS_INFO("4th");
                    target.position.x=-1400-2312;
                    target.position.y=-1500-2309;
                    }
                }
                
                TARGET_pub.publish(target);
                ROS_INFO("TARGET (%f,%f) has been published", target.position.x,target.position.y);
                Rflag = 1 ;
            }else{

            publishTargetPose();  // 继续发布旋转目标

            if(Rflag){break;}

            rate.sleep();
            }
        }
    }

    void setOffboardMode() {
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "OFFBOARD";
        if (client_set_mode.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
            ROS_INFO("Offboard mode set successfully");
        } else {
            ROS_ERROR("Failed to set offboard mode");
        }
    }

    void run() {
if (!setArm()) ;
        waitForAngle();
        ensureQuadMode();
        setOffboardMode();
        rotateAndCheckAngle();
    }

    bool setArm() {

    ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandBool>("/standard_vtol_0/mavros/cmd/arming");

    // 等待服务启动
    if (!ros::service::waitForService("/standard_vtol_0/mavros/cmd/arming", ros::Duration(5.0))) {
        ROS_ERROR("Service /standard_vtol_0/mavros/cmd/arming not available");
        return false;
    }

    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;

    if (client.call(arm_srv)) {
        if (arm_srv.response.success) {
            ROS_INFO("Vehicle armed successfully");
            return true;
        } else {
            ROS_WARN("Failed to arm vehicle");
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service /standard_vtol_0/mavros/cmd/arming");
        return false;
    }
}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uva");
    UAVController uav_controller;
    uav_controller.run();
    ros::spin();
    return 0;
}