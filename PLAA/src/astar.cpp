// @author: Kanzhi Wu
// @contact:kanzhi.wu@gmail.com
// @data:   22/10/2015
// ASTAR path planner

#include "astar.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/thread.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>

using namespace std;

ASTAR::ASTAR( ros::NodeHandle & nh ) {
  nh_ = &nh;
  waypoints_done_ = false;
  initialised_ = false;
  pose_init_ = false;
  nh_->param<double>( "map_width", map_width_, 6.44 );
  nh_->param<double>( "map_height", map_height_, 3.33 );
  nh_->param<double>( "map_resolution", map_resolution_, 0.01 );
  nh_->param<double>( "grid_resolution", grid_resolution_, 0.06 );

  map_img_ = cv::imread( ros::package::getPath("astar_planner")+"/data/map.png", CV_LOAD_IMAGE_GRAYSCALE);
  plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
  ros::ServiceServer astar_srv = nh_->advertiseService( "/astar_pose", &ASTAR::astar_srv_callback, this );
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  boost::thread thread = boost::thread( boost::bind(&ASTAR::path_search, this) );
  ros::MultiThreadedSpinner spinner(2);
  ros::Rate loop(10);
  while ( ros::ok() ) {
      spinner.spin();
      loop.sleep();
  }
}

// service callback
bool ASTAR::astar_srv_callback( PLAA::pose::Request &req,
                                PLAA::pose::Response &resp ) {
  start_[0] = req.x;
  start_[1] = req.y;
  start_[2] = req.o;
  {
    boost::mutex::scoped_lock lock(srv_mutex_);
    pose_init_ = true;
  }
  srv_cond_.notify_one();
  resp.Start = true;
  return true;
}

// data type convert functions
double ASTAR::grid2meterX(int x) {
  double nx = x*grid_resolution_-map_width_/2+grid_resolution_/2;
  return nx;
}

double ASTAR::grid2meterY(int y) {
  double ny = map_height_/2-y*grid_resolution_-grid_resolution_/2;
  return ny;
}

int ASTAR::meterX2grid(double x) {
  int gx = round((x+map_width_/2)/grid_resolution_);
  if ( gx > grid_width_-1 )
    gx = grid_width_-1;
  if ( gx < 0 )
    gx = 0;
  return gx;
}

int ASTAR::meterY2grid(double y) {
  int gy = round((map_height_/2-y)/grid_resolution_);
  if ( gy > grid_height_-1 )
    gy = grid_height_-1;
  if ( gy < 0 )
    gy = 0;
  return gy;
}

// print functions
void ASTAR::print_list(vector<Node> nodelist) {
  for ( int i = 0; i < (int)nodelist.size(); ++ i ) {
    Node & n = nodelist[i];
    cout << "idx [" << n.x << ", " << n.y << "] with cost = " << n.cost << endl;
  }
}

void ASTAR::print_grid_props(const char *c) {
  for ( int i = 0; i < grid_height_; ++ i ) {
    for ( int j = 0; j < grid_width_; ++ j ) {
      if ( strcmp(c, "c") == 0 )
        cout << gridmap_[i][j].closed << " ";
      else if ( strcmp(c, "h") == 0 )
        cout << gridmap_[i][j].heuristic << " ";
      else if ( strcmp(c, "e") == 0 )
        cout << gridmap_[i][j].expand << " ";
      else if ( strcmp(c, "o") == 0 )
        cout << gridmap_[i][j].occupied << " ";
    }
    cout << endl;
  }
}

void ASTAR::print_waypoints(vector<Waypoint> waypoints) {
  cout << "Waypoints:\n";
  for ( int i = 0; i < (int)waypoints.size(); ++ i ) {
    Waypoint & w = waypoints[i];
    cout << "[" << w.x << ", " << w.y << "]\n";
  }
}



vector<Node> ASTAR::descending_sort(vector<Node> nodelist) {
  Node temp;
  for ( int i = 0; i < (int)nodelist.size(); ++ i ) {
    for ( int j = 0; j < (int)nodelist.size()-1; ++ j ) {
      if ( nodelist[j].cost < nodelist[j+1].cost ) {
        temp = nodelist[j];
        nodelist[j] = nodelist[j+1];
        nodelist[j+1] = temp;
      }
    }
  }
  return nodelist;
}



// initialise grid map
void ASTAR::setup_gridmap(){
  gridmap_.clear();
  grid_height_ = int(ceil(map_height_/grid_resolution_));
  grid_width_  = int(ceil(map_width_/grid_resolution_));

  cout << "grid: " << grid_height_ << ", " << grid_width_ << endl;

  for ( int y = 0; y < grid_height_; ++ y ) {
    vector<Grid> gridx;
    for ( int x = 0; x < grid_width_; ++ x ) {
      Grid d;
      d.closed = 0; d.occupied = 0;
      d.heuristic = 0; d.expand = numeric_limits<int>::max();
      gridx.push_back( d );
    }
    gridmap_.push_back( gridx );
  }

  // set occupied flag
  int step = (int)(grid_resolution_/map_resolution_);

  for ( int y = 0; y < grid_height_; ++ y ) {
    for ( int x = 0; x < grid_width_; ++ x ) {
      cv::Point tl, dr;
      tl.x = step*x; tl.y = step*y;
      dr.x = min( map_img_.cols, step*(x+1) );
      dr.y = min( map_img_.rows, step*(y+1) );

      bool found = false;
      for ( int i = tl.y; i < dr.y; ++ i ) {
        for ( int j = tl.x; j < dr.x; ++ j ) {
          if ( (int)map_img_.at<unsigned char>(i,j) != 255 ) {
            found = true;
          }
        }
      }
      gridmap_[y][x].occupied = found == true? 1: 0;
    }
  }
}


// init heuristic
void ASTAR::init_heuristic(Node goal_node) {
  // QUESTION
  // initialise heursitic value for each grids given goal node
  for ( int y = 0; y < grid_height_; ++ y ) {
    for ( int x = 0; x < grid_width_; ++ x ) {
      gridmap_[y][x].heuristic = abs(goal_node.x-x)+abs(goal_node.y-y);
    }
  }
}

// generate policy
void ASTAR::policy(Node start_node, Node goal_node) {
  // QUESTION

  Node current;
  current = goal_node;
  bool found = false;
  optimum_policy_.clear();
  optimum_policy_.push_back( current );

  //actions
  int action[4][2];
  action[0][0] = 0; action[0][1] = 1;
  action[1][0] = 1; action[1][1] = 0;
  action[2][0] = 0; action[2][1] = -1;
  action[3][0] = -1;action[3][1] = 0;

  while ( found == false ) {
    if ( current.x == start_node.x && current.y == start_node.y ) {
      found = true;
      cout << "Find optimum policy\n";
      reverse( optimum_policy_.begin(), optimum_policy_.end() );
      print_list( optimum_policy_ );
    }
    else {
      int minvalue = numeric_limits<int>::max();
      Node next;
      for ( int i = 0; i < 4; ++ i ) {
        int x2 = current.x+action[i][0];
        int y2 = current.y+action[i][1];
        if ( x2 >= 0 && x2 < grid_width_ && y2 >= 0 && y2 < grid_height_ ) {
          if ( gridmap_[y2][x2].expand < minvalue ) {
            next.x = x2;
            next.y = y2;
            minvalue = gridmap_[y2][x2].expand;
          }
        }
      }
      next.cost = minvalue;
      optimum_policy_.push_back( next );
      current = next;
    }
  }

}

// update waypoints
void ASTAR::update_waypoints(double *robot_pose) {
  waypoints_.clear();
  Waypoint w;
  w.x = robot_pose[0];
  w.y = robot_pose[1];
  waypoints_.push_back( w );
  for ( int i = 1; i < (int)optimum_policy_.size(); ++ i ) {
    w.x = grid2meterX(optimum_policy_[i].x);
    w.y = grid2meterY(optimum_policy_[i].y);
    waypoints_.push_back( w );
  }

  smooth_path( 0.5, 0.3);


  poses_.clear();
  string map_id = "/map";
  ros::Time plan_time = ros::Time::now();
  for ( int i = 0; i < (int)waypoints_.size(); ++ i ) {
    Waypoint & w = waypoints_[i];
    geometry_msgs::PoseStamped p;
    p.header.stamp = plan_time;
    p.header.frame_id = map_id;
    p.pose.position.x = w.x;
    p.pose.position.y = w.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 0.0;
    poses_.push_back( p );
  }
  waypoints_done_ = true;
}

// smooth generated path
void ASTAR::smooth_path(double weight_data, double weight_smooth) {
  double tolerance = 0.00001;
  double change = tolerance;

  vector<Waypoint> sm_waypoints = waypoints_;
   while( change >= tolerance){
    change = 0;
    //Find corner constrints
    for(int i = 1;i < (int)waypoints_.size()-1; ++ i) {
      Waypoint aux = sm_waypoints[i];
      sm_waypoints[i].x += weight_data*(waypoints_[i].x-sm_waypoints[i].x);
      sm_waypoints[i].y += weight_data*(waypoints_[i].y-sm_waypoints[i].y);

      sm_waypoints[i].x += weight_smooth*(sm_waypoints[i-1].x+sm_waypoints[i+1].x-2*(sm_waypoints[i].x));
      sm_waypoints[i].y += weight_smooth*(sm_waypoints[i-1].y+sm_waypoints[i+1].y-2*(sm_waypoints[i].y));
      change += abs(aux.x-sm_waypoints[i].x)+abs(aux.y-sm_waypoints[i].y);
    }
  }
  waypoints_.clear();
  waypoints_ = sm_waypoints;
}

// search for astar path planning
bool ASTAR::path_search() {
  {
    boost::mutex::scoped_lock lock(srv_mutex_);
    while(!pose_init_) {
      srv_cond_.wait( lock );
    }
  }

  {
    ROS_INFO("Localization done, start path planning ...");

    // set goal position
    cout << "Goal position x: "; cin >> goal_[0];
    cout << "Goal position y: "; cin >> goal_[1];
    cout << "Goal position o: "; cin >> goal_[2];

    setup_gridmap();

    Node start_node, goal_node;
    start_node.x = meterX2grid(start_[0]);
    start_node.y = meterY2grid(start_[1]);
    start_node.cost = 0;
    goal_node.x = meterX2grid( goal_[0] );
    goal_node.y = meterY2grid( goal_[1] );



    // check start of goal is occupied
    if ( gridmap_[start_node.y][start_node.x].occupied != 0 ||
         gridmap_[goal_node.y][goal_node.x].occupied != 0) {
      cout << "The goal/start is occupied\n";
      cout << "Please change another position\n";
      return false;
    }

    init_heuristic(goal_node);


    gridmap_[start_node.y][start_node.x].closed = 1;
    gridmap_[start_node.y][start_node.x].expand = 0;

    // create open list
    vector<Node> openlist;
    openlist.push_back( start_node );
    bool found = false, resign = false;
    int cost = 1;
    int expand_count = 1;
    Node next;

    // actions
    int action[4][2];
    action[0][0] = 0; action[0][1] = 1;
    action[1][0] = 1; action[1][1] = 0;
    action[2][0] = 0; action[2][1] = -1;
    action[3][0] = -1;action[3][1] = 0;

    while ( found == false && resign == false ) {
      if ( openlist.size() == 0 ) {
        cout << "Path search failed!\n";
        resign = true;
        return false;
      }
      else {
        openlist = descending_sort(openlist);
        next = openlist[openlist.size()-1];
        openlist.pop_back();

        // if the goal is reached
        if ( next.x == goal_node.x && next.y == goal_node.y ) {
          found = true;
          cout << "Goal reached: [" << next.x << ", " << next.y << "] with cost " << next.cost << endl;
        }
        else {
          for ( int i = 0; i < 4; ++ i ) {
            int x2 = next.x+action[i][0];
            int y2 = next.y+action[i][1];
            if ( x2 >=0 && x2 < grid_width_ && y2 >=0 && y2 < grid_height_ ) {
              if ( gridmap_[y2][x2].closed == 0 && gridmap_[y2][x2].occupied == 0 ) {
                Node nnode;
                nnode.cost = next.cost+cost;
                nnode.cost += gridmap_[y2][x2].heuristic;
                nnode.x = x2;
                nnode.y = y2;
                openlist.push_back( nnode );
                gridmap_[y2][x2].closed = 1;
                gridmap_[y2][x2].expand = expand_count;
                expand_count ++;
              }
            }
          }
        }
      }
    }
    policy( start_node, goal_node );
    update_waypoints( start_ );

    initialised_ = true;

    publish_plan(poses_);

    // compute linear velocity and angular velocity
    // turn orientation first
//    geometry_msgs::Twist t0;
//    double diffy_01 = waypoints_[1].y-waypoints_[0].y;
//    double diffx_01 = waypoints_[1].x-waypoints_[0].x;
//    double ori_01 = atan2(diffy_01, diffx_01);
//    cout << "Turn to orientation " << ori_01 << " for the 1st step\n";
//    t0.angular.x = 0; t0.angular.y = 0; t0.angular.z = (ori_01+2*M_PI)%(2*M_PI);
//    t0.linear.x = 0; t0.linear.y = 0; t0.linear.z = 0.0;
//    cmd_pub_.publish(t0);
    cout << "Press to publish command ..."; int a; cin >> a;
//    geometry_msgs::Twist t;
//    t.linear.x = 5;
//    t.linear.y = 0;
//    t.linear.z = 0;
//    t.angular.x = 0.0; t.angular.y = 0.0; t.angular.z = 0;
//    cmd_pub_.publish(t);

    ROS_INFO( "Publish command input" );
    double cur_pose[3];
    cur_pose[0] = start_[0]; cur_pose[1] = start_[1]; cur_pose[2] = start_[2];

    for ( int i = 0; i < (int)waypoints_.size()-1; ++ i ) {
      cout << "waypoint: " << i << endl;
      // compute linear velocity
      geometry_msgs::Twist ti;
      double dx = waypoints_[i].x-cur_pose[0];
      double dy = waypoints_[i].y-cur_pose[1];
      ti.linear.x = sqrt(dx*dx+dy*dy)*5;
      ti.linear.y = 0;
      ti.linear.z = 0;
      double diffy = waypoints_[i+1].y-waypoints_[i].y;
      double diffx = waypoints_[i+1].x-waypoints_[i].x;
      double diffo = fmod((atan2(diffy, diffx)+2*M_PI), 2*M_PI);
      ti.angular.x = 0.0; ti.angular.y = 0.0;
      if ( diffo > cur_pose[2] )
        ti.angular.z = diffo-cur_pose[2];
      else
        ti.angular.z = cur_pose[2]-diffo;
      ti.angular.z *= 5;
      cout << "z angle: " << ti.angular.z << endl;
      cmd_pub_.publish(ti);
      getchar();

      cur_pose[0] = waypoints_[i].x;
      cur_pose[1] = waypoints_[i].y;
      cur_pose[2] = diffo;
    }
    return true;
  }

}

void ASTAR::publish_plan(const vector<geometry_msgs::PoseStamped> &path) {
  if ( !initialised_ ) {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // create a message for the plan
  nav_msgs::Path path_msgs;
  path_msgs.poses.resize( path.size() );
  if (!path.empty()) {
    path_msgs.header.frame_id = path[0].header.frame_id;
    path_msgs.header.stamp = path[0].header.stamp;
  }
  cout << "---\n";
  for ( int i = 0; i < (int)path.size(); ++ i ) {
    path_msgs.poses[i] = path[i];
  }
  cout << "---\n";
//  while (initialised_) {
    plan_pub_.publish( path_msgs );
//    initialised_ = false;
//  }
  cout << "---\n";
}





int main( int argc, char ** argv ) {
  ros::init( argc, argv, "astar_planner" );
  ros::NodeHandle nh("~");
  ASTAR astar( nh );
  return 1;
}


