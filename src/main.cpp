#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"
#include "udacity.h"
#include "Waypoints.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double findVehicleLane(double vehicle){
  double lane = 1;
  if(vehicle > 0 && vehicle < 4) lane = 0;
  else if(vehicle >= 4 && vehicle <= 8) lane = 1;
  else if(vehicle > 8 && vehicle <= 12) lane = 2;
  return lane;
}

bool checkToChangeLane(const int p_value, const double vehicle_s, const double check_lane, const vector<vector<double>> & sensor_fusion_data)
{
  bool do_change_lane = false;
  double buffer_front = 100000;
  double buffer_rear  = -100000;

  for(int i = 0; i < sensor_fusion_data.size(); i++){
    float d = sensor_fusion_data[i][6];
    double detected_vehicles = findVehicleLane(d);

    if(detected_vehicles == check_lane){
      double vx = sensor_fusion_data[i][3];
      double vy = sensor_fusion_data[i][4];
      double vehicle_speed = sqrt(vx*vx + vy*vy);
      double vehicle_s_check = sensor_fusion_data[i][5];

      vehicle_s_check += ((double)p_value * 0.02 * vehicle_speed);

      double dist_s = vehicle_s_check - vehicle_s;
      double dist_pos = sqrt(dist_s * dist_s);  

      if(dist_s > 0){ 
        buffer_front = min(dist_s, buffer_front);
      }
      else if(dist_s <= 0){
        buffer_rear = max(dist_s, buffer_rear);
      }
    }
  } 

  if((buffer_front > 20) && (-1 * buffer_rear > 13)) 
  {
    do_change_lane = true;
  }
  return do_change_lane;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  int myLane = 1;
  int new_waypoint = 0;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  Waypoints waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s,map_waypoints_dx, map_waypoints_dy);

  h.onMessage([&waypoints, &myLane, &new_waypoint](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode){
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            double speed_limit = 49;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            double dist = 0.0;
            double front_space = 0.0;

            double vehicle_x = car_x;
            double vehicle_y = car_y;
            double vehicle_yaw = deg2rad(car_yaw);
            double vehicle_x_prev = 0.0;
            double vehicle_y_prev = 0.0;
            double vehicle_new_d = 0.0;
            double vehicle_new_x = 0.0;
            double vehicle_new_y = 0.0;
            double vehicle_new_distance = 0.0;
            double nearby_vehicle_vx = 0.0;
            double nearby_vehicle_vy = 0.0;
            double nearby_vehicle_s = 0.0;
            double nearby_vehicle_speed = 0.0;
            double nearby_vehicle_d = 0.0;
            double nearby_vehicle_lane = 0.0;
            
            double shift_x = 0.0;
            double shift_y = 0.0;
            double prev_vehicle_x = 0.0;
            double prev_vehicle_y = 0.0;
            
            double alpha = 0.0;
            double denom = 0.0; 
            double x_point = 0.0;
            double y_point = 0.0;
            double x_ref = 0.0;
            double y_ref = 0.0;
            double buffer = 100000;

            int path_length = previous_path_x.size();
            int next_waypoint = -1;
            int delta_waypoint = 0;
            int leftover_waypoint = 0;
            
            vector<double> anchor_x;
            vector<double> anchor_y;
            vector<double> next_waypoint_0;
            vector<double> next_waypoint_1;
            vector<double> next_waypoint_2;

            bool change_lane = false;
            bool collision_alert   = false;
            bool changed_lane = false;
            bool start_lane_change = true;

            if(path_length < 2){
                next_waypoint = waypoints.NextWaypoint_go(vehicle_x, vehicle_y, vehicle_yaw);
            }
            else{
              vehicle_x = previous_path_x[path_length-1];
              vehicle_y = previous_path_y[path_length-1];
              vehicle_x_prev = previous_path_x[path_length-2];
              vehicle_y_prev = previous_path_y[path_length-2];
              vehicle_yaw = atan2(vehicle_y - vehicle_y_prev, vehicle_x - vehicle_x_prev);

              next_waypoint = waypoints.NextWaypoint_go(vehicle_x, vehicle_y, vehicle_yaw);

              car_s = end_path_s;
              dist = sqrt((vehicle_x - vehicle_x_prev) * (vehicle_x - vehicle_x_prev) + (vehicle_y - vehicle_y_prev) * (vehicle_y - vehicle_y_prev));
              car_speed = (dist / 0.02) * 2.24;
            }

            for(int i = 0; i < sensor_fusion.size(); i++){
              nearby_vehicle_d = sensor_fusion[i][6];
              nearby_vehicle_lane = findVehicleLane(nearby_vehicle_d);
              nearby_vehicle_vx = sensor_fusion[i][3];
              nearby_vehicle_vy = sensor_fusion[i][4];
              nearby_vehicle_s = sensor_fusion[i][5];

              if(nearby_vehicle_lane == myLane){
                nearby_vehicle_speed = sqrt(nearby_vehicle_vx * nearby_vehicle_vx + nearby_vehicle_vy*nearby_vehicle_vy);
                nearby_vehicle_s += (path_length * 0.02 * nearby_vehicle_speed);
                front_space = nearby_vehicle_s - car_s; 

                if(front_space > 0 && front_space < 35.0 && front_space < buffer){
                  buffer = front_space;

                  if(front_space > 25.0){
                    speed_limit = nearby_vehicle_speed * 2.24;
                    change_lane = true;
                  } 
                  else{
                    // go slower than front car
                    speed_limit = nearby_vehicle_speed * 2.24 - 3.0; 
                    collision_alert = true;
                    change_lane = true;
                  }
                }
              }
            }

            delta_waypoint = next_waypoint - new_waypoint;
            leftover_waypoint = delta_waypoint % waypoints.map_x_.size();

            if(change_lane && leftover_waypoint > 2){
              changed_lane = false;
              if(myLane != 0 && !changed_lane){
                start_lane_change = true;
                start_lane_change = checkToChangeLane(path_length, car_s, myLane - 1, sensor_fusion);

                if(start_lane_change){ 
                  changed_lane = true;
                  myLane -= 1;
                  new_waypoint = next_waypoint;
                }
              }
              if(myLane != 2 && !changed_lane){
                start_lane_change = true;
                start_lane_change = checkToChangeLane(path_length, car_s, myLane + 1, sensor_fusion);
                
                if(start_lane_change){
                  changed_lane = true;
                  myLane += 1;
                  new_waypoint = next_waypoint;
                }
              }
            }

            if(path_length < 1){
              prev_vehicle_x = car_x - cos(car_yaw);
              prev_vehicle_y = car_y - sin(car_yaw);
              anchor_x.push_back(prev_vehicle_x);
              anchor_x.push_back(car_x);
              anchor_y.push_back(prev_vehicle_y);
              anchor_y.push_back(car_y);
            } 
            else{
              anchor_x.push_back(previous_path_x[path_length-2]);
              anchor_x.push_back(previous_path_x[path_length-1]);
              anchor_y.push_back(previous_path_y[path_length-2]);
              anchor_y.push_back(previous_path_y[path_length-1]);
            }

            vehicle_new_d = 2 + myLane * 4;
            next_waypoint_0 = getXY((car_s + 30), vehicle_new_d, waypoints.map_s_, waypoints.map_x_, waypoints.map_y_);
            next_waypoint_1 = getXY((car_s + 30*2), vehicle_new_d, waypoints.map_s_, waypoints.map_x_, waypoints.map_y_);
            next_waypoint_2 = getXY((car_s + 30*3), vehicle_new_d, waypoints.map_s_, waypoints.map_x_, waypoints.map_y_);
            anchor_x.push_back(next_waypoint_0[0]);
            anchor_x.push_back(next_waypoint_1[0]);
            anchor_x.push_back(next_waypoint_2[0]);
            anchor_y.push_back(next_waypoint_0[1]);
            anchor_y.push_back(next_waypoint_1[1]);
            anchor_y.push_back(next_waypoint_2[1]);

            for(int i = 0; i < anchor_x.size(); i++){
              shift_x = anchor_x[i] - vehicle_x;
              shift_y = anchor_y[i] - vehicle_y;
              anchor_x[i] = (shift_x * cos(0 - vehicle_yaw) - shift_y * sin(0 - vehicle_yaw));
              anchor_y[i] = (shift_x * sin(0 - vehicle_yaw) + shift_y * cos(0 - vehicle_yaw));
            }

            tk::spline s_spline;
            s_spline.set_points(anchor_x, anchor_y);

            for(int i = 0; i < path_length; i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            vehicle_new_x = 30;
            vehicle_new_y = s_spline(vehicle_new_x);
            vehicle_new_distance = sqrt((vehicle_new_x * vehicle_new_x) + (vehicle_new_y * vehicle_new_y));

            for(int i = 1; i < 50 - path_length; i++){
                if(car_speed < speed_limit) car_speed += (2.24 / 10);
                else if(car_speed > speed_limit) car_speed -= (2.24 / 10);

                denom = (vehicle_new_distance / (0.02 * car_speed/2.24));
                x_point = alpha + (vehicle_new_x) / denom;
                y_point = s_spline(x_point);
                alpha = x_point;
                x_ref = x_point;
                y_ref = y_point;
                x_point = (x_ref * cos(vehicle_yaw) - y_ref * sin(vehicle_yaw));
                y_point = (x_ref * sin(vehicle_yaw) + y_ref * cos(vehicle_yaw));
                x_point += vehicle_x;
                y_point += vehicle_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
