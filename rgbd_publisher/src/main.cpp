/**
 * RGBD publisher from images on disk
 * @author: Kanzhi Wu
 */


#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>


using namespace std;

class RGBDPublisher{
private:
  ros::NodeHandle * nh_;
  string dir_;
  int n_;

  string rgb_topic_;
  string depth_topic_;
  string camera_info_topic_;

  ros::Publisher rgb_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher camera_info_pub_;

  sensor_msgs::Image rgb_msg_;
  sensor_msgs::Image depth_msg_;
  sensor_msgs::CameraInfo camera_info_msg_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> rgbd_policy;
  boost::shared_ptr<message_filters::Synchronizer<rgbd_policy> > sync_;



public:
  RGBDPublisher( ros::NodeHandle & nh, string dir, int n ) {
    nh_ = & nh;
    dir_ = dir;
    n_ = n;

    rgb_topic_ = "/camera/rgb_image";
    depth_topic_ = "/camera/depth_image";
    camera_info_topic_ = "/camera_info";

    rgb_pub_ = nh.advertise<sensor_msgs::Image>(rgb_topic_, 1);
    depth_pub_ = nh.advertise<sensor_msgs::Image>(depth_topic_, 1);
    camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic_, 1);


    while ( nh_->ok() ) {
      cv_bridge::CvImage rgb_cv;
      cv_bridge::CvImage depth_cv;
      for ( int i = 1; i <= n_; ++ i ) {
        string rgbpath = dir_+"/rgb_"+boost::lexical_cast<string>(i)+".png";
        rgb_cv.image = cv::imread( rgbpath, CV_LOAD_IMAGE_COLOR );
        rgb_cv.encoding = sensor_msgs::image_encodings::BGRA8;
        rgb_cv.toImageMsg( rgb_msg_ );

        string depthpath = dir_+"/depth_"+boost::lexical_cast<string>(i)+".png";
        depth_cv.image  = cv::imread( depthpath, CV_LOAD_IMAGE_ANYDEPTH );
        depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
        depth_cv.toImageMsg( depth_msg_ );


        camera_info_msg_.height = rgb_cv.image.rows;
        camera_info_msg_.width  = rgb_cv.image.cols;
        camera_info_msg_.P[0] = 525.0; camera_info_msg_.P[1] = 0.0; camera_info_msg_.P[2] = 319.5; camera_info_msg_.P[3] = 0.0;
        camera_info_msg_.P[4] = 0.0; camera_info_msg_.P[5] = 525.0; camera_info_msg_.P[6] = 239.5; camera_info_msg_.P[7] = 0.0;
        camera_info_msg_.P[8] = 0.0; camera_info_msg_.P[9] = 525.0; camera_info_msg_.P[10] = 1.0; camera_info_msg_.P[11] = 0.0;
        camera_info_msg_.D.resize(5);
        camera_info_msg_.D[0] = 0.0;
        camera_info_msg_.D[1] = 0.0;
        camera_info_msg_.D[2] = 0.0;
        camera_info_msg_.D[3] = 0.0;
        camera_info_msg_.D[4] = 0.0;

        rgb_pub_.publish( rgb_msg_ );
        depth_pub_.publish( depth_msg_ );
        camera_info_pub_.publish( camera_info_msg_ );

        cv::imshow( "rgbimage", rgb_cv.image );
        cv::waitKey(0);

      }
    }
  }

};


int main( int argc, char ** argv ) {
    ros::init( argc, argv, "data_publisher" );
    ros::NodeHandle nh("~");
    RGBDPublisher publisher( nh, argv[1], 10 );
    ros::spin();
    return 1;
}
