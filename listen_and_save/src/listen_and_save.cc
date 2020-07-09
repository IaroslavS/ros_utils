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
    auto topic = op.add<popl::Value<std::string>>("t", "topic", "topic, to which listen. By default - /stereo/depth", "/stereo/depth");
    auto path = op.add<popl::Value<std::string>>("p", "path", "path, where images should be saved. give path without last '/'", "");
    auto encoding = op.add<popl::Value<std::string>>("e", "encoding","encoding with which transformation should be, default - '32FC1'", "32FC1");
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
    }

    int num_frame = 0;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    image_transport::Subscriber sub = it.subscribe(topic->value(), 1, [&](const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat depth_image = cv_bridge::toCvShare(msg, encoding->value())->image;
        std::stringstream filename_string_stream;
        filename_string_stream << std::setfill('0') << std::setw(6) << num_frame;
        cv::imwrite(path->value()+"/"+filename_string_stream.str()+".exr", depth_image);
        num_frame++;
    });
    
    ros::spin();

    return EXIT_SUCCESS;
}
