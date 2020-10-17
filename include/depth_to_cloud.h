#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_traits.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <popl.hpp>

using namespace message_filters::sync_policies;
using namespace depth_image_proc;
namespace enc = sensor_msgs::image_encodings;

typedef sensor_msgs::PointCloud2 PointCloud;

namespace depth_image_proc {

class PointCloudXyzI 
{
  public: 
    PointCloudXyzI(const std::string& depth_topic, const std::string& left_topic, 
                   const std::string& left_camera_info_topic,  const std::string& output_topic_cloud); // Constructor
    // ~PointCloudXyzI(); // Destructor
  
  private:
    // types
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    // Variables
    ros::NodeHandle nh;
    ros::NodeHandlePtr intensity_nh_;
    ros::NodeHandlePtr depth_nh;
    boost::shared_ptr<image_transport::ImageTransport> intensity_it_, depth_it_;
    image_geometry::PinholeCameraModel model_;

    // Variables, Subscriptions
    image_transport::SubscriberFilter sub_depth_, sub_intensity_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    boost::shared_ptr<Synchronizer> sync_;

    // Variables, Publications
    boost::mutex connect_mutex_;
    ros::Publisher pub_point_cloud_;

    // Methods

    template<typename T, typename T2>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& intensity_msg,
                 const PointCloud::Ptr& cloud_msg);

    // Handles (un)subscribing when clients (un)subscribe
    void connectCb();
    void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& intensity_msg_in,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);
}; // class
} //namespace depth_image_proc
