#include "Waypoints.h"

Waypoints::Waypoints(vector<double> x, vector<double> y,vector<double> s, vector<double>d_x, vector<double>d_y):map_x_(x), map_y_(y), map_s_(s), map_d_x_(d_x), map_d_y_(d_y){
}

int Waypoints::NextWaypoint_go(double x, double y, double t){ 
    return NextWaypoint(x, y, t, this->map_x_, this->map_y_);
}