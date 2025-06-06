#ifndef CRANE_UTILITY
#define CRANE_UTILITY
#include <optional>
#include <string>



struct LuffingJibCraneConfig
{
  double x; // unit: meter
  double y; // unit: meter
  double h; // unit: meter
  double d; // unit: meter, the distance betwee the rotation axis of turntable and the rotation axis of jib
  double jib_length; // unit: meter
  double slewing_angle; // unit: degree
  double jib_angle; // unit: degree
  double hoisting_height; // unit: meter; the distance from hook to jib

};

struct TowerCraneConfig
{
  double x;
  double y;
  double h;
  double jib_length;
  double slewing_angle;
  double trolley_radius;
  double hoisting_height; // the distance from hook to jib
};

struct CraneConfig
{
  std::string name;
  int type; // 0: luffing jib crane; 1: tower crane
  double x; // unit: meter
  double y; // unit: meter
  double h; // unit: meter
  double d; // unit: meter, the distance betwee the rotation axis of turntable and the rotation axis of jib
  double jib_length; // unit: meter
  double slewing_angle; // unit: degree
  double trolley_radius_jib_angle; // unit: meter or degree
  double hoisting_height; // unit: meter; the distance from hook to jib

  double slewing_velocity; // unit: degree/s
  double current_time;
  bool is_main_crane; // true: master crane;false, master crane's neighbor
};

struct CraneJointState
{
    double current_time;
    double slewing_angle;
    double jib_trolley;
    double hoisting_height;
    double slewing_velocity; 

    CraneJointState() : 
    current_time(0.0),
    slewing_angle(0.0),
    jib_trolley(0.0),
    hoisting_height(0.0),
    slewing_velocity(0.0)
    {}
};


#endif