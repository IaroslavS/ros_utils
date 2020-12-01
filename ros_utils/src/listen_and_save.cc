#include <iostream>
#include <chrono>
#include <numeric>
#include <string>
#include <boost/filesystem.hpp>


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

extern int num_frame=0;

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "listen_and_save");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto topic = op.add<popl::Value<std::string>>("t", "topic", "topic, to which listen", "/stereo/depth");
    auto path = op.add<popl::Value<std::string>>("p", "path", "path, where images should be saved", "");
    auto encoding = op.add<popl::Value<std::string>>("e", "encoding","encoding with which transformation should be", "32FC1");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    std::string path_string = path->value();
    path_string.erase(path_string.find_last_not_of("/ ") + 1);
    boost::filesystem::create_directories(path_string+"/images");

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    int num_frame = 0;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    FILE *file_timestamps;
    file_timestamps = fopen((path->value()+ "/timestamps_depth.txt").c_str(),"w");
    fclose (file_timestamps);
    image_transport::Subscriber sub = it.subscribe(topic->value(), 1, [&](const sensor_msgs::ImageConstPtr& msg) {
        // std::cout << ros::Time::now().toSec() - 1603273847.0 << std::endl;
        // std::cout << "Time above is for ts " << msg->header.stamp.nsec/1000000000.0 << std::endl << std::endl;
        file_timestamps = fopen((path_string+"/timestamps_depth.txt").c_str(),"a");
        cv::Mat depth_image = cv_bridge::toCvShare(msg, encoding->value())->image;
        std::stringstream filename_string_stream;
        filename_string_stream << std::setfill('0') << std::setw(6) << num_frame;
        cv::imwrite(path_string+"/images/"+filename_string_stream.str()+".exr", depth_image);
        const double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec/1000000000.0; 
        fprintf (file_timestamps, "%f\n", timestamp);
        num_frame++;
        fclose(file_timestamps);
    });
        
    ros::spin();

    return EXIT_SUCCESS;
}
