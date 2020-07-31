#include <iostream>
#include <chrono>
#include <memory>
#include <numeric>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "yaml-cpp/yaml.h"

#include <popl.hpp>

using namespace std;

class stereo_rectifier {
    public:
        //! Constructor
        explicit stereo_rectifier(const YAML::Node& yaml_node) {
            const unsigned int rows = yaml_node["Camera.rows"].as<unsigned int>();
            const unsigned int cols = yaml_node["Camera.cols"].as<unsigned int>();
            const cv::Size img_size(cols, rows);
            K_l = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.K_left"].as<std::vector<double>>());
            K_r = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.K_right"].as<std::vector<double>>());
            R_l = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.R_left"].as<std::vector<double>>());
            R_r = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.R_right"].as<std::vector<double>>());
            const auto D_l_vec = yaml_node["StereoRectifier.D_left"].as<std::vector<double>>();
            const auto D_r_vec = yaml_node["StereoRectifier.D_right"].as<std::vector<double>>();
            D_l = parse_vector_as_mat(cv::Size(1, D_l_vec.size()), D_l_vec);
            D_r = parse_vector_as_mat(cv::Size(1, D_r_vec.size()), D_r_vec);
            const auto fx = yaml_node["Camera.fx"].as<double>();
            const auto fy = yaml_node["Camera.fy"].as<double>();
            const auto cx = yaml_node["Camera.cx"].as<double>();
            const auto cy = yaml_node["Camera.cy"].as<double>();
            K_rect = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            cv::initUndistortRectifyMap(K_l, D_l, R_l, K_rect, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
            cv::initUndistortRectifyMap(K_r, D_r, R_r, K_rect, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
        
        };

        //! Apply stereo-rectification
        void rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r, cv::Mat& out_img_l, cv::Mat& out_img_r) const {
            cv::remap(in_img_l, out_img_l, undist_map_x_l_, undist_map_y_l_, cv::INTER_LINEAR);
            cv::remap(in_img_r, out_img_r, undist_map_x_r_, undist_map_y_r_, cv::INTER_LINEAR);
        };

        void get_calib_matrixes(cv::Mat& out_K_left, cv::Mat& out_K_right, cv::Mat& out_R_left, cv::Mat& out_R_right, cv::Mat& out_D_left, 
                                          cv::Mat& out_D_right, cv::Mat& out_K_rect, double& out_focal_x_baseline) {  
            out_K_left = K_l;
            out_K_right = K_r;
            out_R_left = R_l;
            out_R_right = R_r;
            out_D_left = D_l;
            out_D_right = D_r;
            out_K_rect = K_rect;
            out_focal_x_baseline = focal_x_baseline;
        }

    private:
        //! Parse std::vector as cv::Mat
        static cv::Mat parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec) {
            cv::Mat mat(shape, CV_64F);
            std::memcpy(mat.data, vec.data(), shape.height * shape.width * sizeof(double));
            return mat;
        };

        //matrixes for rectification
        //cv::Size *img_size;
        cv::Mat K_l; 
        cv::Mat K_r; 
        cv::Mat R_l; 
        cv::Mat R_r; 
        cv::Mat D_l; 
        cv::Mat D_r;
        cv::Mat K_rect;
        double focal_x_baseline;

        //! undistortion map for x-axis in left image
        cv::Mat undist_map_x_l_;
        //! undistortion map for y-axis in left image
        cv::Mat undist_map_y_l_;
        //! undistortion map for x-axis in right image
        cv::Mat undist_map_x_r_;
        //! undistortion map for y-axis in right image
        cv::Mat undist_map_y_r_;
};

void callback_rectify_and_publish(const sensor_msgs::ImageConstPtr& input_left_msg, const sensor_msgs::ImageConstPtr& input_right_msg, 
                                  image_transport::Publisher& publisher_left, image_transport::Publisher& publisher_right,
                                  ros::Publisher pub_info_left, ros::Publisher pub_info_right, sensor_msgs::CameraInfo info_left,
                                  sensor_msgs::CameraInfo info_right, stereo_rectifier& stereo_rectifier) {
    
    cv::Mat input_left_image = cv_bridge::toCvShare(input_left_msg, "bgr8")->image;
    cv::Mat input_right_image = cv_bridge::toCvShare(input_right_msg, "bgr8")->image;
    cv::Mat left_image_rectified, right_image_rectified;
    
    stereo_rectifier.rectify(input_left_image, input_right_image, left_image_rectified, right_image_rectified);  
    
    cv_bridge::CvImagePtr rect_left_frame, rect_right_frame;
    rect_left_frame = boost::make_shared<cv_bridge::CvImage>();
    rect_right_frame = boost::make_shared<cv_bridge::CvImage>();
    rect_left_frame->encoding = sensor_msgs::image_encodings::BGR8;
    rect_right_frame->encoding = sensor_msgs::image_encodings::BGR8;
    rect_left_frame->image = left_image_rectified;
    rect_right_frame->image = right_image_rectified;
    rect_left_frame->header.stamp = input_left_msg->header.stamp;
    rect_right_frame->header.stamp = input_right_msg->header.stamp;

    info_left.header.stamp = input_left_msg->header.stamp;
    info_right.header.stamp = input_right_msg->header.stamp;

    publisher_left.publish(rect_left_frame->toImageMsg());
    publisher_right.publish(rect_right_frame->toImageMsg());

    pub_info_left.publish(info_left);
    pub_info_right.publish(info_right);

}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "rectify_and_publish");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto left_input_topic = op.add<popl::Value<std::string>>("", "input_topic_left", "topic of left raw images", "/stereo/left/image_raw");
    auto right_input_topic = op.add<popl::Value<std::string>>("", "input_topic_right", "topic of right raw images", "/stereo/right/image_raw");
    auto left_output_topic = op.add<popl::Value<std::string>>("", "output_topic_left", "topic of left rectified images that will be used for publishing", "/stereo/left/image_rect");
    auto right_output_topic = op.add<popl::Value<std::string>>("", "output_topic_right", "topic of right rectified images that will be used for publishing", "/stereo/right/image_rect");
    auto left_camera_info = op.add<popl::Value<std::string>>("", "left_camera_info", "topic of left camera info that will be used for publishing", "/stereo/left/camera_info");
    auto right_camera_info = op.add<popl::Value<std::string>>("", "right_camera_info", "topic of right camera info that will be used for publishing", "/stereo/right/camera_info");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    const YAML::Node yaml_node = YAML::LoadFile(config_file_path->value());
    stereo_rectifier stereo_rectifier(yaml_node);
    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    sensor_msgs::CameraInfo info_left, info_right;

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_left_sub(nh, left_input_topic->value(), 1);
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub(nh, right_input_topic->value(), 1);

    ros::Publisher pub_info_left = nh.advertise<sensor_msgs::CameraInfo>(left_camera_info->value(), 1);
    ros::Publisher pub_info_right = nh.advertise<sensor_msgs::CameraInfo>(right_camera_info->value(), 1);

    image_transport::ImageTransport it_left(nh);
    image_transport::ImageTransport it_right(nh);
    image_transport::Publisher publisher_left = it_left.advertise(left_output_topic->value(), 1);
    image_transport::Publisher publisher_right = it_right.advertise(right_output_topic->value(), 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    MySyncPolicy sync_policy(10);
    message_filters::Synchronizer<MySyncPolicy> sync_stereo(static_cast<const MySyncPolicy>(sync_policy), image_left_sub, image_right_sub);
    
    sync_stereo.registerCallback(boost::bind(&callback_rectify_and_publish, _1, _2, publisher_left, publisher_right, pub_info_left, pub_info_right, info_left, info_right, stereo_rectifier));
    
    ros::spin();

    return EXIT_SUCCESS;
}
