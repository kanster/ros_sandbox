/**
 * extract laser scan and odometry from load data bag
 * @author Kanzhi Wu
 */


#include <ros/ros.h>
#include <ros/common.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <vector>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans;
std::vector<Eigen::Matrix4f> odoms;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    message_filters::SimpleFilter<M>::signalMessage(msg);
  }
};

/// callback
void callback( const sensor_msgs::PointCloud2ConstPtr & scan_msg,
               const nav_msgs::OdometryConstPtr & odom_msg)
{
  pcl::PCLPointCloud2 scan;
  pcl_conversions::toPCL( *scan_msg, scan );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::fromPCLPointCloud2( scan, *cloud );
  scans.push_back( cloud );

  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf quat;
  quat.x() = odom_msg->pose.pose.orientation.x;
  quat.y() = odom_msg->pose.pose.orientation.y;
  quat.z() = odom_msg->pose.pose.orientation.z;
  quat.w() = odom_msg->pose.pose.orientation.w;
  Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();
  odom.block<3,3>(0,0) = rotation_matrix;
  odom(0,3) = odom_msg->pose.pose.position.x;
  odom(1,3) = odom_msg->pose.pose.position.y;
  odom(2,3) = odom_msg->pose.pose.position.z;
  odoms.push_back( odom );

}

/// main func
int main( int argc, char ** argv )
{
  if ( argc != 2 )
  {
    printf( "Command: %s <bagfile>", argv[0] );
    return -1;
  }
  ros::init( argc, argv, "loam_data_retrieve" );
  ros::NodeHandle nh("~");

  rosbag::Bag bag;
  bag.open( argv[1], rosbag::bagmode::Read );
  printf("Load bag file done\n");


  std::string scan_topic_name = "/laser_cloud_surround";
  std::string odom_topic_name = "/integrated_to_init";

  std::vector<std::string> topics;
  topics.push_back( scan_topic_name );
  topics.push_back( odom_topic_name );

  rosbag::View view( bag, rosbag::TopicQuery( topics ) );

  BagSubscriber<sensor_msgs::PointCloud2> scan_sub;
  BagSubscriber<nav_msgs::Odometry> odom_sub;

  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync( scan_sub, odom_sub, 10 );
  sync.registerCallback( boost::bind( &callback, _1, _2 ) );

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
//    printf( "%s\n", m.getTopic().c_str() );
//    nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
//    if (s != NULL)
//    {
//      printf("%f\n", s->pose.pose.position.x);
//    }
    if (m.getTopic() == scan_topic_name || ("/" + m.getTopic() == scan_topic_name))
    {
      sensor_msgs::PointCloud2::ConstPtr scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (scan != NULL)
        scan_sub.newMessage(scan);
    }

    if (m.getTopic() == odom_topic_name || ("/" + m.getTopic() == odom_topic_name))
    {
      nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
      if (odom != NULL)
        odom_sub.newMessage(odom);
    }

  }
  bag.close();
  printf( "Number of scans: %d", (int)scans.size() );
  std::ofstream posesout( "poses.txt" );
  for ( size_t i = 0; i < scans.size(); i ++ )
  {
    std::string pcdpath = "cloud_"+boost::lexical_cast<std::string>(i)+".ply";
    pcl::io::savePLYFileASCII( pcdpath, *scans[i] );
    posesout << odoms[i](0,0) << " " << odoms[i](0,1) << " " << odoms[i](0,2) << " " << odoms[i](0,3) << " " <<
                odoms[i](1,0) << " " << odoms[i](1,1) << " " << odoms[i](1,2) << " " << odoms[i](1,3) << " " <<
                odoms[i](2,0) << " " << odoms[i](2,1) << " " << odoms[i](2,2) << " " << odoms[i](2,3) << " " <<
                odoms[i](3,0) << " " << odoms[i](3,1) << " " << odoms[i](3,2) << " " << odoms[i](3,3) << "\n";
  }
  posesout.close();

  return 0;
}
