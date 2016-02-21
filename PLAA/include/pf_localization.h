// @author: Kanzhi Wu
// @contact:kanzhi.wu@gmail.com
// @data:   22/10/2015
// Particle Filter based Localization under ROS


#ifndef PF_LOCALIZATION_H
#define PF_LOCALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
#include <ros/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
using namespace std;


struct Particle{
    double x;
    double y;
    double o;
    double weight;
};

class PFLocalization{
    ros::NodeHandle * nh_;
    ros::Subscriber scan_sub_;

    message_filters::Subscriber<sensor_msgs::LaserScan> robot_laser_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>     robot_pose_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> robot_policy;
    boost::shared_ptr<message_filters::Synchronizer<robot_policy> > robot_sync_;

    cv::Mat map_img_;
    string scan_topic_name_;
    string pose_topic_name_;
    vector<double> scan_data_;
    double dist_noise_;
    double ori_noise_;
    bool start_step_;

    // particles
    int n_particles_;
    vector<Particle> particles_;
    bool scan_empty_;
    boost::mutex scan_mutex_;
    boost::condition_variable scan_cond_;

    // robot pose
    double prev_pos_[3];
    double current_pos_[3];
    double robot_odom_[2];
    double esti_pos_[3];

    // publisher
    ros::Publisher particles_pub_;
    ros::Publisher esti_pos_pub_;

    // signal to astar planner

    ros::ServiceClient astar_client_;
    // step count for resampling
    int step_count_;

    // cmd mutex
    ros::Subscriber cmd_sub_;
    ros::Subscriber map_sub_;

    bool exit_flag_;

private:
    void robot_callback( const sensor_msgs::LaserScanConstPtr & scan_msg,
                         const nav_msgs::OdometryConstPtr & base_pose_msg );

    void cmd_callback( const geometry_msgs::TwistConstPtr & cmd_msg );

    void init_particles();
    double sense( double sigma, double x, double y, double theta );
    void motion( double dist, double ori );
    void calc_estimate();
    void resampling();
    double calc_meanerror();

    // main process function
    void process();
    boost::thread process_thread_;


private:
    // uniform and gaussian sampling
    double uniform_sampling( double min, double max );
    double gaussian_sampling( double mean, double sigma );

public:
    PFLocalization( ros::NodeHandle & nh, int n_particles );

};




#endif
