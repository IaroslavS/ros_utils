#include <iostream>
#include <chrono>
#include <numeric>
#include <string>

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

using namespace std;

int main(int argc, char* argv[]) {
    
    ros::init(argc, argv, "listen_and_save");
    ros::NodeHandle nh;
    std::string topic, path_string, encoding;
    nh.param("/stereo/listen_and_save/topic", topic, std::string(""));
    nh.param("/stereo/listen_and_save/path", path_string, std::string(""));
    nh.param("/stereo/listen_and_save/encoding", encoding, std::string(""));

    std::string extension;
    if (encoding == "32FC1") {
        extension = ".exr";
    }
    else {
        extension = ".png";
    }
    path_string.erase(path_string.find_last_not_of("/ ") + 1);

    int num_frame = 0;
    image_transport::ImageTransport it(nh);
    FILE *file_timestamps;
    file_timestamps = fopen((path_string+ "/timestamps_depth.txt").c_str(),"w");
    fclose (file_timestamps);
    image_transport::Subscriber sub = it.subscribe(topic, 1, [&](const sensor_msgs::ImageConstPtr& msg) {
        // std::cout << ros::Time::now().toSec() - 1603273847.0 << std::endl;
        // std::cout << "Time above is for ts " << msg->header.stamp.nsec/1000000000.0 << std::endl << std::endl;
        file_timestamps = fopen((path_string+"/timestamps_depth.txt").c_str(),"a");
        cv::Mat depth_image = cv_bridge::toCvShare(msg, encoding)->image;
        std::stringstream filename_string_stream;
        filename_string_stream << std::setfill('0') << std::setw(6) << num_frame;
        cv::imwrite(path_string+"/images/"+filename_string_stream.str()+extension, depth_image);
        const double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec/1000000000.0; 
        fprintf (file_timestamps, "%f\n", timestamp);
        num_frame++;
        fclose(file_timestamps);
    });
        
    ros::spin();

    return EXIT_SUCCESS;
}
