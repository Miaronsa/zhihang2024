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

        // 订阅图像话题
        image_sub = it.subscribe("/standard_vtol_0/camera/image_raw", 1, &ColorDetector::imageCallback, this);

        // 创建发布器，发布色块中心位置
        center_pub = nh.advertise<ship_detector::circle_part_m>("H_block_center", 1);
        processed_image_pub = it.advertise("/H_blocks_image", 1);
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
        cv::Scalar lower_yellow_1(-5, -5, 250);//46
        cv::Scalar upper_yellow_1(5, 5, 255);//如何调节

        cv::Scalar lower_yellow_2(90 ,0 , 0);
        cv::Scalar upper_yellow_2(200 ,40 , 150);

        // 创建掩码
        cv::Mat mask_1;
        cv::Mat mask_2;
        cv::Mat mask;

        cv::inRange(hsv_image, lower_yellow_1, upper_yellow_1, mask);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));//牛比 6

        //进行形态学操作
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);  



        // 找到色块的轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::GaussianBlur(mask, mask, cv::Size(9, 9), 0);
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        auto maxContourIt = std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        }
        );
         if(contours.size()!=0)
        {
        std::vector<cv::Point> largestContour = *maxContourIt;
        cv::Mat result(image.size(), image.type(), cv::Scalar::all(0));
        ship_detector::circle_part_m target_position;
        // for (int num=0; num < contours.size();num++)
        // {
        //     cv::drawContours(result, contours, num, cv::Scalar(255, 0, 0), 10);
        // }
        ship_detector::circle_part_m center_msg;
        cv::Point center_point;

        //计算单个色块面积
        double area = cv::contourArea(largestContour);
        std::cout << area << std::endl;

        //计算中心
        cv::Moments mu = moments(largestContour);
        center_point.x = mu.m10 / mu.m00;
        center_point.y = mu.m01 / mu.m00;
        cv::Point2f contours_center(mu.m10 / mu.m00, mu.m01 / mu.m00);

        //对单个色块属性进行赋值
        center_msg.x = center_point.x;
        center_msg.y = center_point.y;
        center_msg.square = area;

        centers.object.push_back(center_msg);
        cv::Moments moments = cv::moments(largestContour);

        std::vector<std::vector<cv::Point>>::iterator it = std::find(contours.begin(), contours.end(), largestContour);
        size_t index = std::distance(contours.begin(), it);
        cv::drawContours(result, contours, index, cv::Scalar(0,255, 0), 3);


                
        for (size_t i = 0; i < centers.object.size(); i++)
        {
            //通过每个轮廓的中心计算出整个的中心
            target_position.x+=centers.object[i].x;
            target_position.y+=centers.object[i].y;
            target_position.square=centers.object.size();
        }
        target_position.x=target_position.x/target_position.square;
        target_position.y=target_position.y/target_position.square;
        centers.object.clear();
        

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
    }

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
    ros::init(argc, argv, "ship_lander");
    ColorDetector color_detector;
    ros::spin();
    return 0;
}




















