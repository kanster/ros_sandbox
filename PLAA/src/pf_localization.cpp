// @author: Kanzhi Wu
// @contact:kanzhi.wu@gmail.com
// @data:   22/10/2015
// Particle Filter based Localization under ROS

#include "pf_localization.h"
#include "PLAA/pose.h"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>

// opencv
#include <opencv2/opencv.hpp>

// boost
#include <boost/random.hpp>

// system
#include <cmath>

// global const variables
const double map_width  = 6.44;
const double map_height = 3.33;
const double nbeams = 5;  // no. of laser beams

// callback function for scan and base pose ground truth
void PFLocalization::robot_callback(
    const sensor_msgs::LaserScanConstPtr & scan_msg,
    const nav_msgs::OdometryConstPtr & base_pose_msg ) {
  // discretize the scan from 361 -> 5
  int step = (scan_msg->ranges.size()-1)/(nbeams-1);
  for ( int i = 0; i < nbeams; ++ i ) {
    scan_data_[i] = scan_msg->ranges[0+step*i];
  }

  // obtain the current pose to quaternion
  tf::Quaternion q;
  q.setX( base_pose_msg->pose.pose.orientation.x );
  q.setY( base_pose_msg->pose.pose.orientation.y );
  q.setZ( base_pose_msg->pose.pose.orientation.z );
  q.setW( base_pose_msg->pose.pose.orientation.w );

  // convert quaternion to rotation matrix
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY( roll, pitch, yaw );
  yaw = yaw < 0.0? 2*M_PI+yaw: yaw;

  if ( start_step_ == true ) {
    prev_pos_[0] = base_pose_msg->pose.pose.position.x; // x
    prev_pos_[1] = base_pose_msg->pose.pose.position.y; // y
    prev_pos_[2] = yaw;
    current_pos_[0] = base_pose_msg->pose.pose.position.x; // x
    current_pos_[1] = base_pose_msg->pose.pose.position.y; // y
    current_pos_[2] = yaw;
    robot_odom_[0] = 0.0;
    robot_odom_[1] = 0.0;
  }
  else {
    prev_pos_[0] = current_pos_[0];
    prev_pos_[1] = current_pos_[1];
    prev_pos_[2] = current_pos_[2];
    current_pos_[0] = base_pose_msg->pose.pose.position.x; // x
    current_pos_[1] = base_pose_msg->pose.pose.position.y; // y
    current_pos_[2] = yaw;
    robot_odom_[0] = sqrt( pow(current_pos_[0]-prev_pos_[0], 2)+
                           pow(current_pos_[1]-prev_pos_[1], 2) );
    robot_odom_[1] = atan2( sin(current_pos_[2]-prev_pos_[2]),
                            cos(current_pos_[2]-prev_pos_[2]));
  }

  {
    boost::mutex::scoped_lock lock( scan_mutex_ );
    scan_empty_ = false;
  }
  scan_cond_.notify_one();
}



// main process function for localization
void PFLocalization::process() {
  while ( !exit_flag_ ) {
    ROS_INFO_ONCE( "Start PF based localization" );
    {
      // check scan data availability
      boost::mutex::scoped_lock lock( scan_mutex_ );
      while ( scan_empty_ )
        scan_cond_.wait( lock );
    }

    {
      if ( start_step_ == true ) {
        // init particles
        init_particles();
        start_step_ = false;
      }
      else {
        // compute odometry and check weather robot moves
        if ( (robot_odom_[0] != 0.0 || robot_odom_[1] != 0.0) ) {
          ROS_INFO("Robot odom: distance = %f and orientation = %f", robot_odom_[0], robot_odom_[1]);
          motion( robot_odom_[0], robot_odom_[1] );

          // updated weights for each particle
          double normaliser = 0.0;
          vector<double> tmp_weights;
          for ( int i = 0; i < (int)particles_.size(); ++ i ) {
            tmp_weights.push_back( sense( 0.5, particles_[i].x, particles_[i].y, particles_[i].o ) );
            normaliser += tmp_weights[i]*particles_[i].weight;
          }
          for ( int i = 0; i < (int)particles_.size(); ++ i ) {
            particles_[i].weight = tmp_weights[i]*particles_[i].weight/normaliser;
          }

          // calculate estimate robot pose
          calc_estimate();

          double error = calc_meanerror();
          ROS_INFO("Estimation error: %f", error);
          double err_thresh = 0.1;
          if ( error < err_thresh ) {
            ROS_INFO("Localization correct!");
            robot_laser_sub_.unsubscribe();
            robot_pose_sub_.unsubscribe();
            PLAA::pose pose_srv;
            pose_srv.request.Enable = true;
            pose_srv.request.x = esti_pos_[0];
            pose_srv.request.y = esti_pos_[1];
            pose_srv.request.o = esti_pos_[2];
            if ( astar_client_.call( pose_srv ) ) {
              ROS_INFO( "return status: %s", pose_srv.response.Start? "true" : "false" );
            }
            else {
              ROS_ERROR( "Failed to call service astar_pose" );
            }
          }

          // resampling particles every 5 steps
          if ( step_count_ == 5 ) {
            resampling();
            step_count_ = 0;
          }
          step_count_ ++;
        }

        // publish particles and estimated robot pose
        geometry_msgs::PoseStamped est_pos_msg;
        est_pos_msg.header.stamp = ros::Time::now();
        est_pos_msg.header.frame_id = "/map";
        tf::poseTFToMsg( tf::Pose(tf::createQuaternionFromYaw(esti_pos_[2]),
                                  tf::Vector3(esti_pos_[0], esti_pos_[1], 0.0)),
                         est_pos_msg.pose);
        esti_pos_pub_.publish( est_pos_msg );

        geometry_msgs::PoseArray particles_msg;
        particles_msg.header.stamp = ros::Time::now();
        particles_msg.header.frame_id = "/map";
        particles_msg.poses.resize( particles_.size() );
        for ( int i = 0; i < (int)particles_.size(); ++ i ) {
          Particle & p = particles_[i];
          if ( p.x < map_width/2.0 && p.x > -map_width/2.0 &&
               p.y < map_height/2.0 && p.y > -map_height/2.0)  {
            tf::poseTFToMsg( tf::Pose(tf::createQuaternionFromYaw(particles_[i].o),
                                      tf::Vector3( particles_[i].x, particles_[i].y, 0.0 )),
                             particles_msg.poses[i]);
          }
        }
        particles_pub_.publish( particles_msg );
      }

      scan_mutex_.lock();
      scan_empty_ = true;
      scan_mutex_.unlock();
    }
  }
}

// particle filter init
void PFLocalization::init_particles() {
  particles_.resize( n_particles_ );
  for ( int i = 0; i < n_particles_; ++ i ) {
    // randomise particles
    particles_[i].x = uniform_sampling( -map_width/2, map_width/2 );
    particles_[i].y = uniform_sampling( -map_height/2, map_height/2 );
    particles_[i].o = uniform_sampling( 0, 2*M_PI );
    particles_[i].weight = 1.0/n_particles_;
  }
}

// compute the likelihood using sense model
double PFLocalization::sense(double sigma, double x, double y, double theta) {
  double error = 0;
  double step_rad = M_PI/(scan_data_.size()-1);
  // compute particle position in pixel coordinate
  int px, py;
  px = (x+map_width/2)*100.0;
  py = (map_height-(y+map_height/2))*100.0;
  cv::Point2d stpt(px, py);

  for ( int i = 0; i < (int)scan_data_.size(); ++ i ) {
    // get accurate ray distance
    double raydist = 0.0;
    double c_theta = theta-M_PI/2+step_rad*i;
    if ( c_theta > M_PI*2 )
      c_theta -= M_PI*2;
    if ( c_theta < 0 )
      c_theta += M_PI*2;
    cv::Point2d tgpt;
    tgpt.x = stpt.x+1000*cos(2*M_PI-c_theta);
    tgpt.y = stpt.y+1000*sin(2*M_PI-c_theta);
    cv::Point2d obspt;
    cv::LineIterator it(map_img_, stpt, tgpt, 8);
    for ( int j = 0; j < it.count; ++ j, ++ it ){
      cv::Point2d pt = it.pos();
      if ( static_cast<int>(map_img_.at<uchar>(pt)) != 255 ||
           pt.x == 0 || pt.x == map_img_.cols-1 ||
           pt.y == 0 || pt.y == map_img_.rows-1 ) {
        obspt = it.pos();
        break;
      }
    }
    raydist = sqrt((obspt.x-stpt.x)/100.0*(obspt.x-stpt.x)/100.0+
                   (obspt.y-stpt.y)/100.0*(obspt.y-stpt.y)/100.0);

    // compute error probability
    double dist_error = abs( scan_data_[i]-raydist );
    error += pow(dist_error, 2);
  }
  error = exp(-error/(2*pow(sigma,2)));
  return error;
}

// motion model
void PFLocalization::motion(double dist, double ori) {
  // check dist negativity
  if ( dist < 0 ) {
    cout << "ERROR: distance < 0\n";
    exit(-1);
  }


  for ( int i = 0; i < (int)particles_.size(); ++ i ) {
    //add noise to motion
    double ndist = gaussian_sampling( dist, dist_noise_ );
    double nori  = gaussian_sampling( ori, ori_noise_ );
    particles_[i].x += ndist*cos(particles_[i].o);
    particles_[i].y += ndist*sin(particles_[i].o);
    particles_[i].o = fmod(particles_[i].o+nori, 2*M_PI);
  }
}

// calculate estimate robot pose
void PFLocalization::calc_estimate() {
  // reset estimated pose
  esti_pos_[0] = 0.0;
  esti_pos_[1] = 0.0;
  esti_pos_[2] = 0.0;

  // compute estimate pose
  double sin_o = 0, cos_o = 0;
  for ( int i = 0; i < (int)particles_.size(); ++ i ) {
    Particle & p = particles_[i];
    esti_pos_[0] += p.weight*p.x;
    esti_pos_[1] += p.weight*p.y;
    sin_o += sin(particles_[i].o) * particles_[i].weight;
    cos_o += cos(particles_[i].o) * particles_[i].weight;
  }
  esti_pos_[2] = atan2(sin_o, cos_o);
}

// resampling particles
void PFLocalization::resampling() {
  int ind = int(uniform_sampling(0., 1.)*double(particles_.size()));
  double beta = 0.0;
  double maxw = 0;
  for ( int i = 0; i < (int)particles_.size(); ++ i )
    if ( particles_[i].weight > maxw )
      maxw = particles_[i].weight;
  vector<Particle> n_particles;
  for ( int i = 0; i < (int)particles_.size(); ++ i ) {
    beta += uniform_sampling(0., 1.0)*2.0*maxw;
    bool found = false;
    while ( !found ) {
      beta -= particles_[ind].weight;
      ind = int(fmod((ind+1), double(particles_.size())));
      if ( particles_[ind].x > map_width/2 || particles_[ind].x < -map_width/2 ||
           particles_[ind].y > map_height/2 || particles_[ind].y < -map_height/2 )
        continue;
      if ( beta > particles_[ind].weight )
        continue;
      found = true;
    }
    n_particles.push_back( particles_[ind] );
  }
  particles_ = n_particles;

  // add jitter
  for( int k = 0; k < (int)particles_.size(); ++ k) {
    particles_[k].x += gaussian_sampling(0., 0.02);
    particles_[k].y += gaussian_sampling(0., 0.02);
    particles_[k].o += fmod(gaussian_sampling(0., 0.05),2*M_PI);
  }
}

// calculate mean error
double PFLocalization::calc_meanerror() {
  double dx = esti_pos_[0] - current_pos_[0];
  double dy = esti_pos_[1] - current_pos_[1];
  double error = sqrt(dx*dx+dy*dy);

  return error;
}

// generate uniform sampling
double PFLocalization::uniform_sampling( double min, double max ) {
  static boost::mt19937 rng( static_cast<unsigned> (time(0)) );
  boost::uniform_real<double> uni_dist(min, max);
  boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > uni_gen(rng, uni_dist);
  return uni_gen();
}

// generate gaussian sampling
double PFLocalization::gaussian_sampling( double mean, double sigma ) {
  static boost::mt19937 rng( static_cast<unsigned> (time(0)) );
  boost::normal_distribution<double> norm_dist( mean, sigma );
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > norm_gen(rng, norm_dist);
  return norm_gen();
}

// command callback
void PFLocalization::cmd_callback(const geometry_msgs::TwistConstPtr &cmd_msg) {
  ROS_INFO( "Command received" );
}


// PFLocalization constructor
PFLocalization::PFLocalization(ros::NodeHandle &nh, int n_particles ) {
  nh_ = &nh;
  exit_flag_ = false;
  scan_empty_ = true;
  map_img_ = cv::imread( ros::package::getPath("pf_localization")+"/data/map.png", CV_LOAD_IMAGE_GRAYSCALE );
  start_step_ = true;

  step_count_ = 0;

  // init noise in motion
  dist_noise_ = 0.01;
  ori_noise_ = 3.0/180.0*M_PI; // in degree

  scan_data_.resize( nbeams );
  scan_topic_name_ = "/base_scan";
  pose_topic_name_ = "/base_pose_ground_truth";

  particles_pub_ = nh.advertise<geometry_msgs::PoseArray>("/particles", 10);

  // reset estimated pose
  esti_pos_[0] = 0.0;
  esti_pos_[1] = 0.0;
  esti_pos_[2] = 0.0;
  esti_pos_pub_  = nh.advertise<geometry_msgs::PoseStamped>("/esti_pose", 10);

  n_particles_ = n_particles;

  robot_laser_sub_.subscribe( nh, scan_topic_name_, 1 );
  robot_pose_sub_.subscribe( nh, pose_topic_name_, 1 );

  cmd_sub_ = nh.subscribe("/cmd_vel", 1, &PFLocalization::cmd_callback, this);
  astar_client_ = nh.serviceClient<PLAA::pose>("/astar_pose");
  robot_sync_.reset( new message_filters::Synchronizer<robot_policy>(robot_policy(100), robot_laser_sub_, robot_pose_sub_) );
  robot_sync_->registerCallback( boost::bind(&PFLocalization::robot_callback, this, _1, _2) );

  process_thread_ = boost::thread( boost::bind(&PFLocalization::process, this) );
}

int main( int argc, char ** argv ) {
  ros::init( argc, argv, "pf_localizaton" );
  ros::NodeHandle nh("~");
  PFLocalization pf(nh, 1000);
  ros::spin();
}
