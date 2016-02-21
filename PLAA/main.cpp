// @author: Kanzhi Wu
// @contact:kanzhi.wu@gmail.com

#include "include/pf_localization.h"

#include <iostream>

using namespace std;

int main( int argc, char ** argv ) {
  ros::init( argc, argv, "pf_localizaton" );
  ros::NodeHandle nh("~");
  PFLocalization pf(nh, 1000);
  ros::spin();
}
