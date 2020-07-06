#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // initialization of variables across the whole process
    int lane = 1; // start on lane 1
    double ref_vel = 49.5; // mph
    const double MS_PER_MPH = 0.447; 
    const double TIME_STEP = 0.02;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_size = previous_path_x.size();
          // Creat a list of waypoints that are widely spaced. I'll use them to do spline fitting
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw_rad = deg2rad(car_yaw);

          // if previous path is close to empty
          if (prev_size<2){
            double prev_car_x = car_x - cos(deg2rad(car_yaw));
            double prev_car_y = car_y - sin(deg2rad(car_yaw));
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use previous path's end points as starting reference
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];
            ref_yaw_rad = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
          }
          // In frenet, add evenly spaced anchor points for spline fitting
          double SPACE = 30;
          vector<double> next_wp0 = getXY(car_s+SPACE, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+SPACE*2, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+SPACE*3, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          // transformation from global frame to car local frame, car head to +x, to help fitting&calculation
          for (int i=0; i<ptsx.size(); i++){
            double local_x = ptsx[i] - ref_x;
            double local_y = ptsy[i] - ref_y;
            ptsx[i] = local_x * cos(-ref_yaw_rad) - local_y * sin(-ref_yaw_rad);
            ptsy[i] = local_x * sin(-ref_yaw_rad) + local_y * cos(-ref_yaw_rad);
            std::cout << "x and y\n";
            std::cout << ptsx[i] << "\n";
            std::cout << ptsy[i] << "\n";
          }
          tk::spline spl;
          spl.set_points(ptsx, ptsy);
          // fill in all the previous points
          for (int i=0; i< prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // break up the spline points to represent velocity properly
          double end_x = 30;
          double end_y = spl(end_x);
          // estimate distance using a right triangle 
          double end_dist = sqrt(end_x*end_x+end_y*end_y);
          // fill up points to NPOINTS
          int NPOINTS = 50;
          // how may points to split the fitted spline
          double N = end_dist / (ref_vel * MS_PER_MPH * TIME_STEP);
          std::cout << "N\n";
          std::cout << N;
          std::cout << "prev_size\n";
          std::cout << prev_size;
          for (int i=0;i<NPOINTS-prev_size;i++){
            double cur_x = (i+1)*end_x/N;
            double cur_y = spl(cur_x);
            // transformation from car local frame to global frame
            double global_cur_x = cur_x * cos(ref_yaw_rad) - cur_y * sin(ref_yaw_rad);
            double global_cur_y = cur_x * sin(ref_yaw_rad) + cur_y * cos(ref_yaw_rad);
            next_x_vals.push_back(global_cur_x + ref_x);
            next_y_vals.push_back(global_cur_y + ref_y);
          }
          std::cout << "next x vals\n";
          for (int i=0;i<next_x_vals.size();i++){
            std::cout << next_x_vals[i] << "\n";
          }
          std::cout << "next y vals\n";
          for (int i=0;i<next_x_vals.size();i++){
            std::cout << next_y_vals[i] << "\n";
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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