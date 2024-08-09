#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <ship_detector/circle_m.h>
#include <ship_detector/circle_part_m.h>

class ColorDetector
{
public:
    ColorDetector()
    {
        image_transport::ImageTransport it(nh);
        // 初始化ROS节点

        // 创建图像传输对象
        // image_transport::ImageTransport it(nh);

        // 订阅图像话题
        image_sub = it.subscribe("/standard_vtol_0/camera/image_raw", 1, &ColorDetector::imageCallback, this);

        // 创建发布器，发布色块中心位置
        center_pub = nh.advertise<ship_detector::circle_part_m>("color_block_center", 1);
        processed_image_pub = it.advertise("/color_blocks_image", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        // 处理图像，识别特定颜色
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义颜色范围（这里以黄色为例）
        cv::Scalar lower_yellow(26, 43, 46);//46
        cv::Scalar upper_yellow(34, 255, 255);//如何调节

        //定义白色
        // cv::Scalar lower_white(150, 150, 150); // 阈值可以调整
        // cv::Scalar upper_white(255, 255, 255);

        // 创建掩码
        cv::Mat mask;

        cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);
        // if (cv::countNonZero(mask_yellow) == 0) {
        // std::cerr << "Error: Mask is empty." << std::endl;
        // } 
        // else 
        // {
        // std::cout << "Mask is not empty." << std::endl;
        // }

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));//牛比 5

        //进行形态学操作
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
        cv::morphologyEx(mask, mask , cv::MORPH_CLOSE, element);  


        // 找到色块的轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat result(image.size(), image.type(), cv::Scalar::all(0));
        ship_detector::circle_part_m target_position;
        cv::drawContours(result, contours, 0, cv::Scalar(255, 0, 0), 10);

        if(contours.size()==4)
        {
            for (size_t i = 0; i < contours.size(); i++)
            {
                // 每次进入循环，初始化
                ship_detector::circle_part_m center_msg;
                cv::Point center_point;

                //计算单个色块面积
                double area = cv::contourArea(contours[i]);

                //计算中心
                cv::Moments mu = moments(contours[i]);
                center_point.x = mu.m10 / mu.m00;
                center_point.y = mu.m01 / mu.m00;
                cv::Point2f contours_center(mu.m10 / mu.m00, mu.m01 / mu.m00);

                //对单个色块属性进行赋值
                center_msg.x = center_point.x;
                center_msg.y = center_point.y;
                center_msg.square = area;

                centers.object.push_back(center_msg);
                cv::Moments moments = cv::moments(contours[i]);
                cv::drawContours(result, contours, i, cv::Scalar(255, 0, 0), 10);


                
            }
        //通过每个轮廓的中心计算出整个的中心
        target_position.x=(centers.object[0].x+centers.object[1].x+centers.object[3].x+centers.object[4].x)/4;
        target_position.y=(centers.object[0].y+centers.object[1].y+centers.object[2].y+centers.object[3].y)/4;
        target_position.square=centers.object.size();
        centers.object.clear();
        }

        // 计算所有色块区域总和的重心坐标


        // 将处理后的图像转换为ROS图像消息
        // 绘制轮廓
        cv::Point point_circle;
        cv::Scalar scarlar(0,0,255);
        point_circle.x=640;
        point_circle.y=360;
        cv::circle(result,point_circle,10,scarlar,1);
        cv::circle(result,point_circle,10,scarlar,1);
        cv::circle(result,point_circle,10,scarlar,1);
        processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
        center_pub.publish(target_position);
        
        processed_image_pub.publish(processed_image_msg);
        }
    // void publishCenter(const cv::Point2f& center)
    // {
    //     // 创建一个几何消息，包含色块中心的坐标
    //     geometry_msgs::Point center_msg;
    //     center_msg.x = center.x;
    //     center_msg.y = center.y;
    //     center_msg.z = 0; // 假设色块在图像平面上

    //     // 发布色块中心位置
    //     center_pub.publish(center_msg);

private:
    ros::NodeHandle nh;
    image_transport::Subscriber image_sub;
    ros::Publisher center_pub;
    ros::Publisher h_pub;
    image_transport::Publisher h_image;
    image_transport::Publisher processed_image_pub;
    ship_detector::circle_m centers;
    sensor_msgs::ImagePtr processed_image_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ship_detector");
    ColorDetector color_detector;
    ros::spin();
    return 0;
}




























// #include <opencv4/opencv2/opencv.hpp>
// #include <ros/ros.h>
// #include <opencv4/opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <std_msgs/Float64.h>
// #include <sensor_msgs/Image.h>
// #include <ship_detector/circle_m.h>
// #include <ship_detector/circle_part_m.h>
// #include <image_transport/image_transport.h>

// using namespace std;

// class Color_detector
// {
// public:
//     Color_detector(ros::NodeHandle& nh)
//     {
//         it = new image_transport::ImageTransport(nh);
//         image_sub = it->subscribe("/standard_vtol_0/camera/image_raw", 1, &Color_detector::imageCallback, this);

//         circle_center_pub = nh.advertise<ship_detector::circle_part_m>("/standard_vtol/circle_detector", 1);
//         circle_image_pub = nh.advertise<sensor_msgs::Image>("/standard_vtol_0/circle_compressed_image", 1);
//     }

//     ~Color_detector()
//     {
//         delete it;
//     }

//     cv::Mat rgb_to_bgr(cv::Mat frame)
//     {
//         cv::cvtColor(frame, frame_bgr1, cv::COLOR_RGB2BGR);
//         return frame_bgr1;
//     }

//     void detect_yellow(cv::Mat bgr_img)
//     {
//         int yellow_low_h = 240;
//         int yellow_high_h = 255;
        
//         int yellow_low_s = 230;
//         int yellow_high_s = 240;

//         int yellow_low_v = 0;
//         int yellow_high_v = 10;
        
//         cv::inRange(bgr_img, cv::Scalar(yellow_low_h, yellow_low_s, yellow_low_v), cv::Scalar(yellow_high_h, yellow_high_s, yellow_high_v), img_Threshold_1);

//         // // 开操作，去除噪点
//         // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//         // cv::morphologyEx(img_Threshold_1, img_Threshold_1, cv::MORPH_OPEN, element);

//         // // 闭操作，联通一些联通域
//         // cv::morphologyEx(img_Threshold_1, img_Threshold_1, cv::MORPH_CLOSE, element);
//         vector<vector<cv::Point>> contours;
//         vector<cv::Vec4i> hierarchy;
//         cv::findContours(img_Threshold_1, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//         std::cout << contours.size() << std::endl;
//         cv::drawContours(img_Threshold_1,contours,-1,(255,0,0),0.5);

//         for (size_t i = 0; i < contours.size(); i++)
//         {
//             cv::Moments mu = moments(contours[i]);
//             cv::Point center_point;
//             center_point.x = mu.m10 / mu.m00;
//             center_point.y = mu.m01 / mu.m00;
//             cv::Point2f contours_center(mu.m10 / mu.m00, mu.m01 / mu.m00);
//             cv::circle(img_Threshold_1, center_point, 1, (255, 0, 0), 3);
//             ship_detector::circle_part_m circle_part;
//             circle_part.x = center_point.x;
//             circle_part.y = center_point.y;
//             circle_part.square = cv::contourArea(contours[i]);
//             circle.object.push_back(circle_part);
//         }

//         if (circle.object.size() != 4)
//         {
//             ROS_INFO("The num is not fixed up");
//             circle.object.clear();
//         }
//         else
//         {
//             center.x = (circle.object[0].x + circle.object[1].x + circle.object[2].x + circle.object[3].x) / 4;
//             center.y = (circle.object[0].y + circle.object[1].y + circle.object[2].y + circle.object[3].y) / 4;
//             center.square = cv::contourArea(contours[0]) + cv::contourArea(contours[1]) + cv::contourArea(contours[2]) + cv::contourArea(contours[3]);
//         }
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr & msg)
//     {
//         cv_bridge::CvImageConstPtr cv_ptr;
//         cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
//         cv::Mat image = cv_ptr->image;
//         cv::Mat image_1 = rgb_to_bgr(image);
//         detect_yellow(image_1);
//         circle_center_pub.publish(center);
//         processed_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_Threshold_1).toImageMsg();
//         circle_image_pub.publish(processed_image);
//         // circle_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_Threshold_1).toImageMsg());
//         ROS_INFO("receive image");
//     }

// private:
//     image_transport::ImageTransport* it;
//     image_transport::Subscriber image_sub;
//     ros::Publisher circle_center_pub;
//     ros::Publisher circle_image_pub;
//     ship_detector::circle_m circle;
//     ship_detector::circle_part_m center;
//     sensor_msgs::ImagePtr processed_image;
//     cv::Mat frame_bgr1;
//     cv::Mat img_Threshold_1;
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "vision_detector_node");
//     ros::NodeHandle nh;
//     Color_detector color_detector(nh);
//     ros::spin();
//     return 0;
// }


















// // #include <opencv4/opencv2/opencv.hpp>
// // #include <ros/ros.h>
// // #include <opencv4/opencv2/highgui/highgui.hpp>
// // #include <cv_bridge/cv_bridge.h>
// // #include <std_msgs/Float64.h>
// // #include <sensor_msgs/Image.h>
// // #include <ship_detector/circle_m.h>
// // #include <ship_detector/circle_part_m.h>
// // #include <image_transport/image_transport.h>
// // #include "../include/ship_detector/ship_detector.hpp"

// // using namespace std;      
// // //     // imshow("Theresholded image H",img_Threshold);
// // //     // imshow("raw_img H",bgr_img);
    

// // class Color_detector
// // {
// // public:
// //     Color_detector()
// //     {
// //         image_transport::ImageTransport it(nh);//为什么不能放在

// //         //创建订阅器
// //         // circle_pub=nh.advertise<ship_detector::circle_m>("/standard_vtol/circle_detector",1);
// //         image_sub=it.subscribe("/standard_vtol_0/camera/image_raw",1,&Color_detector::imageCallback,this);


// //         //创建发布器
// //         // circle_pub=nh.advertise<ship_detector::circle_m>("/standard_vtol/circle_detector",1);
// //         circle_center_pub=nh.advertise<ship_detector::circle_part_m> ("/standard_vtol/circle_detector",1);
// //         circle_image=nh.advertise<sensor_msgs::Image>("/standard_vtol_0/circle_compressed_image",1);
// //     }

// //     cv::Mat rgb_to_bgr(cv::Mat frame)
// //     {
// //         cv::cvtColor(frame,frame_bgr1,cv::COLOR_RGB2BGR);
// //         // vector<cv::Mat> bgr_split;
// //         // cv::Mat frame_hsv_split;
// //         // cv::split(frame_hsv,frame_hsv_split);
// //         return frame_bgr1;
// //     } 

// //     void detect__yellow(cv::Mat bgr_img)
// //     {  
// //         int yellow_low_h = 250;
// //         int yellow_high_h = 255;
        
// //         int yello_low_s = 235;
// //         int yello_high_s = 240;

// //         int yello_low_v = 0;
// //         int yello_high_v = 10;
        
// //         cv::inRange(bgr_img,cv::Scalar(yellow_low_h,yello_low_s,yello_low_v),cv::Scalar(yellow_high_h,yello_high_s,yello_high_v),img_Threshold_1);

// //         //开操作，去除噪点
// //         cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
// //         cv::morphologyEx(img_Threshold_1,img_Threshold_1, cv::MORPH_OPEN, element); 


// //         //闭操作，联通一些联通域
// //         cv::morphologyEx(img_Threshold_1,img_Threshold_1, cv::MORPH_CLOSE, element);
// //         vector<vector<cv::Point>> contours;
// //         vector<cv::Vec4i> hierarchy;
// //         cv::findContours(img_Threshold_1,contours, hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
// //         // cv::drawContours(bgr_img,contours,-1,(255,0,0),0.5);
// //         for( size_t i ; i < contours.size() ; i++)
// //         {
// //             cv::Moments mu = moments(contours[i]);
// //             cv::Point center_point;
// //             center_point.x=mu.m10 / mu.m00;
// //             center_point.y=mu.m01 / mu.m00;
// //             cv::Point2f contours_center(mu.m10 / mu.m00 , mu.m01 / mu.m00);
// //             cv::circle(bgr_img,center_point,1,(255,0,0),3);
// //             cv::circle(img_Threshold_1,center_point,1,(255,0,0),3);
// //             vector <float> point_circle;
// //             point_circle.push_back(center_point.x);
// //             point_circle.push_back(center_point.y);
// //             ship_detector::circle_part_m circle_part;
// //             circle_part.x=center_point.x;
// //             circle_part.y=center_point.y;
// //             circle_part.square=cv::contourArea(contours[i]);
// //             circle.object.push_back(circle_part);
// //         }
// //         if (circle.object.size() != 4 )
// //         {
// //             std::cout << circle.object.size()<<std::endl;
// //             circle.object.clear();
// //             ROS_INFO("The num is not fixed up");
// //         }
// //         else
// //         {
// //             center.x = (circle.object[1].x+circle.object[2].x+circle.object[3].x+circle.object[4].x)/4;
// //             center.y = (circle.object[1].y+circle.object[2].y+circle.object[3].y+circle.object[4].y)/4;
// //             center.square = cv::contourArea(contours[1])+cv::contourArea(contours[2])+cv::contourArea(contours[3])+cv::contourArea(contours[4]);
// //         }
// //     }
// //     // point_circle
// //     // std::cout << "Point_circle"  << point_circle
// //     // cv::imshow("Theresholded image circle",img_Threshold);
// //     // cv::imshow("raw_img circle",bgr_img);
// //     void imageCallback(const sensor_msgs::ImageConstPtr & msg)
// //     {
// //         cv_bridge::CvImageConstPtr cv_ptr;
// //         cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
// //         cv::Mat image = cv_ptr->image;
// //         cv::Mat image_1 = rgb_to_bgr(image);
// //         detect__yellow(image_1);
// //         circle_center_pub.publish(center);
// //         circle_image.publish(img_Threshold_1);
// //         // detect_color_yellow(image);
// //         // detect_color_white(image);
// //         ROS_INFO( "receive image" );
// //     }

// // private:
// //     ros::NodeHandle nh;
// //     image_transport::Subscriber image_sub;
// //     ros::Publisher circle_pub;
// //     ros::Publisher circle_image;
// //     ros::Publisher circle_center_pub;
// //     ship_detector::circle_m circle;
// //     ship_detector::circle_part_m circle_part;
// //     // ship_detector::circle_part_m h_flag;
// //     cv::Mat img_Threshold_1;
// //     cv::Mat img_Threshold_2;
// //     cv::Mat frame_bgr1;//用于hsv空间转换
// //     ship_detector::circle_part_m center;
    
// // };



// // int main(int argc , char **argv)
// // {
// //     //初始化类

// //     // cv::Mat frame_bgr;
// //     // frame_bgr = rgb_to_bgr();
// //     // detect_color_yellow(frame_bgr);
// //     // detect_color_white(frame_bgr);
// //     ros::init(argc,argv,"vision_detector_node" );
// //     Color_detector color_detector;
// //     while(ros::ok())
// //     {
// //         ros::spinOnce();
// //     }
// //     // ros::NodeHandle nh;
// //     // ros::Rate rate(1);

// //     // image_transport::ImageTransport it(nh);


// //     // // //创建订阅者
// //     // image_transport::Subscriber image_sub=it.subscribe("/standard_vtol_0/camera/image_raw",1,imageCallback);


// //     // // // 创建发布器
// //     // ros::Publisher circle_pub=nh.advertise<ship_detector::circle_m>("/standard_vtol/circle_detector",1);
// //     // ros::Publisher h_pub=nh.advertise<ship_detector::circle_part_m> ("/standard_vtol/h_detector",1);
// //     // ros::Publisher circle_image=nh.advertise<sensor_msgs::CompressedImage>("/standard_vtol_0/circle",1);
// //     // ros::Publisher h_image=nh.advertise<sensor_msgs::CompressedImage>("/standard_vtol_0/H",1);

// //     //问题是在函数内无法调用pub，函数外无法使用内部的数据

// //     // while(nh.ok())
// //     // {
// //     //     ros::spinOnce();
// //     //     circle_pub.publish(circle);
// //     //     h_pub.publish(h_flag);
// //     //     // circle_pub.publish(img_Threshold_1);
// //     //     // circle_pub.publish(img_Threshold_2);
// //     //     rate.sleep();
// //     // };
// // }









































// // // #include <opencv4/opencv2/opencv.hpp>
// // // #include <ros/ros.h>
// // // #include <opencv4/opencv2/highgui/highgui.hpp>
// // // #include <cv_bridge/cv_bridge.h>
// // // #include <std_msgs/Float64.h>
// // // #include <ship_detector/circle_m.h>
// // // #include <ship_detector/circle_part_m.h>
// // // #include <image_transport/image_transport.h>
// // // #include "../include/ship_detector/ship_detector.hpp"

// // // using namespace std;

// // // cv::Mat rgb_to_bgr(cv::Mat frame)
// // // {
// // //     cv::Mat frame_bgr1;
// // //     cv::cvtColor(frame,frame_bgr1,cv::COLOR_RGB2BGR);
// // //     vector<cv::Mat> bgr_split;
// // //     cv::Mat frame_hsv_split;
// // //     // cv::split(frame_hsv,frame_hsv_split);
// // //     return frame_bgr1;
// // // }


// // // void detect_color_yellow(cv::Mat bgr_img)
// // // {  
// // //     int yellow_low_h = 250;
// // //     int yellow_high_h = 255;
    
// // //     int yello_low_s = 235;
// // //     int yello_high_s = 240;

// // //     int yello_low_v = 0;
// // //     int yello_high_v = 10;
    
// // //     cv::Mat img_Threshold_1;
// // //     cv::inRange(bgr_img,cv::Scalar(yellow_low_h,yello_low_s,yello_low_v),cv::Scalar(yellow_high_h,yello_high_s,yello_high_v),img_Threshold_1);

// // //     //开操作，去除噪点
// // //     cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
// // //     cv::morphologyEx(img_Threshold_1,img_Threshold_1, cv::MORPH_OPEN, element); 


// // //     //闭操作，联通一些联通域
// // //     cv::morphologyEx(img_Threshold_1,img_Threshold_1, cv::MORPH_CLOSE, element);
// // //     cout << 1 << endl;
// // //     // cv::imshow("i",img_Threshold_1);
// // //     vector<vector<cv::Point>> contours;
// // //     vector<cv::Vec4i> hierarchy;
// // //     cv::findContours(img_Threshold_1,contours, hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
// // //     // cv::drawContours(bgr_img,contours,-1,(255,0,0),0.5);
// // //     ship_detector::circle_m circle;
// // //     for( size_t i ; i < contours.size() ; i++)
// // //     {
// // //         cv::Moments mu = moments(contours[i]);
// // //         cv::Point center_point;
// // //         center_point.x=mu.m10 / mu.m00;
// // //         center_point.y=mu.m01 / mu.m00;
// // //         cv::Point2f contours_center(mu.m10 / mu.m00 , mu.m01 / mu.m00);
// // //         cv::circle(bgr_img,center_point,1,(255,0,0),3);
// // //         cv::circle(img_Threshold_1,center_point,1,(255,0,0),3);
// // //         vector <float> point_circle;
// // //         point_circle.push_back(center_point.x);
// // //         point_circle.push_back(center_point.y);
// // //         ship_detector::circle_part_m circle_part;
// // //         circle_part.x=center_point.x;
// // //         circle_part.y=center_point.y;
// // //         circle_part.square=cv::contourArea(contours[i]);
// // //         circle.object.push_back(circle_part);
// // //     }
// // //     if (circle.object.size() != 4 )
// // //     {
// // //         circle.object.clear();
// // //         ROS_INFO("The num is not fixed up");
// // //     }
// // //     // point_circle
// // //     // std::cout << "Point_circle"  << point_circle
// // //     // cv::imshow("Theresholded image circle",img_Threshold);
// // //     // cv::imshow("raw_img circle",bgr_img);
    
    

// // // }

// // // void detect_color_white(cv::Mat bgr_img)
// // // {  
// // //     int white_low_h = 245;
// // //     int white_high_h = 255;
    
// // //     int white_low_s = 245;
// // //     int white_high_s = 255;

// // //     int white_low_v = 245;
// // //     int white_high_v = 255;
    
// // //     cv::Mat img_Threshold_2;
// // //     cv::inRange(bgr_img,cv::Scalar(white_low_h,white_low_s,white_low_v),cv::Scalar(white_high_h,white_high_s,white_high_v),img_Threshold_2);

// // //     //开操作，去除噪点
// // //     cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
// // //     cv::morphologyEx(img_Threshold_2,img_Threshold_2, cv::MORPH_OPEN, element); 


// // //     //闭操作，联通一些联通域
// // //     cv::morphologyEx(img_Threshold_2,img_Threshold_2, cv::MORPH_CLOSE, element);
// // //     vector<vector<cv::Point>> contours;
// // //     vector<cv::Vec4i> hierarchy;
// // //     cv::findContours(img_Threshold_2,contours, hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
// // //     cv::drawContours(img_Threshold_2,contours,-1,(255,0,0),0.5);
// // //     ship_detector::circle_part_m h_flag;    

// // //     for( size_t i ; i < contours.size() ; i++)
// // //     {
// // //         cv::Moments mu = moments(contours[i]);
// // //         cv::Point center_point;
// // //         center_point.x=mu.m10 / mu.m00;
// // //         center_point.y=mu.m01 / mu.m00;
// // //         cv::Point2f contours_center(mu.m10 / mu.m00 , mu.m01 / mu.m00);
// // //         cv::circle(bgr_img,center_point,1,(255,0,0),1);
// // //         cv::circle(img_Threshold_2,center_point,1,(255,0,0),1);
// // //         h_flag.x = center_point.x;
// // //         h_flag.y = center_point.y;
// // //         h_flag.square = cv::contourArea(contours[i]); //为什么在外面访问不料
    
// // //     };
// // // }

    

// // // void imageCallback(const sensor_msgs::ImageConstPtr & msg)
// // // {
// // //     cv_bridge::CvImageConstPtr cv_ptr;
// // //     cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
// // //     cv::Mat image = cv_ptr->image;
// // //     image = rgb_to_bgr(image);
// // //     detect_color_yellow(image);
// // //     // detect_color_white(image);
// // //     ROS_INFO( "receive image" );
// // // }

    

    

// // // //     // imshow("Theresholded image H",img_Threshold);
// // // //     // imshow("raw_img H",bgr_img);
    




// // // int main(int argc , char **argv)
// // // {
// // //     //初始化类
// // //     ship_detector::circle_m circle;
// // //     ship_detector::circle_part_m circle_part;
// // //     ship_detector::circle_part_m h_flag;
// // //     cv::Mat img_Threshold_1;
// // //     cv::Mat img_Threshold_2;

// // //     // cv::Mat frame_bgr;
// // //     // frame_bgr = rgb_to_bgr();
// // //     // detect_color_yellow(frame_bgr);
// // //     // detect_color_white(frame_bgr);
// // //     ros::init(argc,argv,"vision_detector_node" );
// // //     ros::NodeHandle nh;
// // //     ros::Rate rate(1);

// // //     image_transport::ImageTransport it(nh);


// // //     // //创建订阅者
// // //     image_transport::Subscriber image_sub=it.subscribe("/standard_vtol_0/camera/image_raw",1,imageCallback);


// // //     // // 创建发布器
// // //     ros::Publisher circle_pub=nh.advertise<ship_detector::circle_m>("/standard_vtol/circle_detector",1);
// // //     // ros::Publisher h_pub=nh.advertise<ship_detector::circle_part_m> ("/standard_vtol/h_detector",1);
// // //     ros::Publisher circle_image=nh.advertise<sensor_msgs::CompressedImage>("/standard_vtol_0/circle",1);
// // //     ros::Publisher h_image=nh.advertise<sensor_msgs::CompressedImage>("/standard_vtol_0/H",1);

// // //     //问题是在函数内无法调用pub，函数外无法使用内部的数据

// // //     while(nh.ok())
// // //     {
// // //         ros::spinOnce();
// // //         circle_pub.publish(circle);
// // //         // h_pub.publish(h_flag);
// // //         circle_pub.publish(img_Threshold_1);
// // //         circle_pub.publish(img_Threshold_2);
// // //         rate.sleep();
// // //     };
// // // }


