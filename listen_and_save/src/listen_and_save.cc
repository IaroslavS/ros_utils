#include <iostream>
#include <chrono>
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

#include <popl.hpp>

using namespace std;

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "listen_and_save");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto topic = op.add<popl::Value<std::string>>("", "topic", "topic, to which listen. By default - /stereo/depth", "/stereo/depth");
    auto path = op.add<popl::Value<std::string>>("", "path", "path, where iamges should be saved", "");    
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;

    int num_frame = 0;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    image_transport::Subscriber sub = it.subscribe(topic, 1, [&](const sensor_msgs::ImageConstPtr& msg) {
        std::stringstream filename_string_stream;
        filename_string_stream << std::setfill('0') << std::setw(6) << num_frame;
        cv::imwrite("/media/cds-s/data3/Datasets/Husky-NKBVS/00_map_half_2020-03-17-14-21-57/RGBD_run_slam_ros/rgb/"+filename_string_stream.str()+".png", left_image);
        num_frame++;
    });

    return EXIT_SUCCESS;
}
