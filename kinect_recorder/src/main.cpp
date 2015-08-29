#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

class KinectRecorder{
private:
  typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image,
  sensor_msgs::Image
  > image_sync_policy;

  ros::NodeHandle * nh_;
  std::string dir_;
  int count_;

  std::string rgb_topic_name_;
  std::string depth_topic_name_;

  message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;

  boost::shared_ptr<message_filters::Synchronizer<image_sync_policy> > sync_;

  void sensor_callback( const sensor_msgs::ImageConstPtr & rgb_image_msg,
                        const sensor_msgs::ImageConstPtr & depth_image_msg );

public:
  KinectRecorder( ros::NodeHandle & nh, std::string dir );

  ~KinectRecorder();

};

KinectRecorder::KinectRecorder(ros::NodeHandle &nh, std::string dir) {
  nh_  = & nh;
  dir_ = dir;
  count_ = 1;

  cv::namedWindow( "rgb_image" );
  cv::moveWindow( "rgb_image", 0, 0 );
  cv::namedWindow( "depth_image" );
  cv::moveWindow( "depth_image", 0, 520 );

  rgb_topic_name_   = "/camera/rgb/image_color";
  depth_topic_name_ = "/camera/depth_registered/image_raw";

  rgb_image_sub_.subscribe( nh, rgb_topic_name_, 1 );
  depth_image_sub_.subscribe( nh, depth_topic_name_, 1 );

  sync_.reset( new message_filters::Synchronizer<image_sync_policy>(image_sync_policy(100), rgb_image_sub_, depth_image_sub_) );
  sync_->registerCallback( boost::bind(&KinectRecorder::sensor_callback, this, _1, _2) );

  ros::Rate loop(1);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop.sleep();
  }
}


KinectRecorder::~KinectRecorder(){

}


void KinectRecorder::sensor_callback(const sensor_msgs::ImageConstPtr &rgb_image_msg, const sensor_msgs::ImageConstPtr &depth_image_msg) {
  ROS_INFO_ONCE( "Image information available" );

  cv::Mat rgb_image   = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat depth_image = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  double min, max;
  cv::minMaxIdx( depth_image, &min, &max );
  cv::Mat vis_depth_image;    // image only for display
  cv::convertScaleAbs( depth_image, vis_depth_image, 255./max );

  cv::imshow( "rgb_image", rgb_image );
  cv::imshow( "depth_image", vis_depth_image );

  char k = cv::waitKey(5);
  if ( k == 's' ) {
    std::string rgb_fn = dir_ + "/rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
    std::string depth_fn = dir_ + "/depth_" + boost::lexical_cast<std::string>(count_) + ".png";
    cv::imwrite( rgb_fn, rgb_image );
    cv::imwrite( depth_fn, depth_image );
    ROS_INFO( "Save frame %d", count_ );
    count_ ++;
  }
}


int main( int argc, char ** argv ) {
  std::cout << "Press s to save images\n";
  if ( argc != 2 ) {
    std::cout << "Please input command: ./kinect_recorder <folder>\n";
    return -1;
  }
  ros::init( argc, argv, "kinect_recorder" );
  ros::NodeHandle nh("~");
  KinectRecorder kr( nh, argv[1] );
  return 1;
}


