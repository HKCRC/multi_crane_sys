#ifndef CRANE_UTILITY
#define CRANE_UTILITY


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


#endif