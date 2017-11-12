#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>
#include "spline.h"
#include "udacity.h"

using std::vector;

class Waypoints {
  public: 
    Waypoints(vector<double> x, vector<double> y, vector<double> s, vector<double>d_x, vector<double>d_y);
    ~Waypoints(){};

    vector<double> map_x_;
    vector<double> map_y_;
    vector<double> map_s_;
    vector<double> map_d_x_;
    vector<double> map_d_y_;

    int NextWaypoint_go(double x, double y, double t);
};

#endif