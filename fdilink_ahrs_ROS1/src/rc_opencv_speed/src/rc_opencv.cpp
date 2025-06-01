#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tracking.cpp"

class ImageConverter {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    shared_ptr<Tracking> tracking = make_shared<Tracking>();
public:
    ImageConverter() : it_(nh_) {

        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this);
        
        // 创建OpenCV窗口（可选，无GUI环境需注释掉）
        cv::namedWindow("ROS Image Subscriber");
    }

    ~ImageConverter() {
        // 关闭窗口
        cv::destroyWindow("ROS Image Subscriber");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 将ROS图像消息转换为OpenCV的Mat格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            // 打印图像信息（可选）
            ROS_INFO("Received image: width=%d, height=%d", image.cols, image.rows);


            // 在此处添加图像处理代码（例如边缘检测）
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
            cv::Canny(gray_image, gray_image, 100, 200);
            // threshold(gray_image, gray_image, 0, 255, cv::THRESH_OTSU); // OTSU二值化方法

            // 显示图像（无GUI环境需注释掉）
            cv::imshow("ROS Image Subscriber", gray_image);
            tracking->trackRecognition(gray_image);
            tracking->drawImage(image);
            imshow("to", image);
            cv::waitKey(1);
            // static int counter = 1;
            // string name = ".jpg";
            // string img_path = "/home/ubuntu/smart-car/opencv/res/train/";
            // name = img_path + to_string(counter) + ".jpg";
            // // 保存图像到文件（可选）
            // cv::imwrite(name, gray_image);
            // counter++;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber");
    ImageConverter ic;
    ros::spin();
    return 0;
}