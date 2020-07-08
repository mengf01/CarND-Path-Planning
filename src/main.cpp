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
    static int lane = 1; // start on lane 1
    static double ref_vel = 0.0; // mph
    const double MS_PER_MPH = 0.447; 
    const double TIME_STEP = 0.02;
    // we should not change 2 lanes at one time or change our mind after decided to change lane.
    // COOL_DOWN will decrease by 1 each time (until 0), and reset to maximum whenever we change lane.
    // Thus, it will keep lane value unchanged for some time, OOL_DOWN = 75 is roughly 75*(0.02*4)=6 seconds
    const int COOL_DOWN_MAX = 75;
    static int cool_down = COOL_DOWN_MAX; 
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
          
          if (cool_down>0){
            cool_down -= 1;
          }

          int prev_size = previous_path_x.size();
          if (prev_size>0){
            car_s = end_path_s;
          }
          // too close to the car ahead
          bool too_close = false;
          // // changing lane and not there yet
          // bool changing_lane = false;
          for (const auto& actor : sensor_fusion){
            double actor_d = actor[6];
            // actor in the same lane
            if (actor_d<(2+4*lane+2) && actor_d>(2+4*lane-2)){
              double actor_vx = actor[3];
              double actor_vy = actor[4];
              double check_speed = sqrt(actor_vx*actor_vx + actor_vy*actor_vy);
              double check_actor_s = actor[5];
              // assume the acotr will move at constant velocity
              check_actor_s += prev_size * TIME_STEP * check_speed;
              if (check_actor_s>car_s && check_actor_s<(car_s+30)){
                too_close = true;
              } 
            }
          }

          if (too_close){
            ref_vel -= 0.22;
            // It's appealing to change lane when there is a car ahead of us.
            // If we are currently changing lane (i.e., not close enough to the center of lane to change), skip the process
            // Here I'd like to change lane if meeting all the following criteria:
            // 1, ref_vel > LANE_CHANGE_VEL, so that the car can swiftly shift lane
            // 2, there is at least AHEAD_BUFFER (meters ahead of the ref point) in the available lane
            // 3, there is at least BEHIND_BUFFER (meters behind current car position) in the available lane
            // 4, If more than 1 lane is available, choose the lane that the car ahead is farther from us
            double LANE_CHANGE_VEL = 30;
            double AHEAD_BUFFER = 40;
            double BEHIND_BUFFER = 20;
            if ((ref_vel>LANE_CHANGE_VEL) && (cool_down==0)){
              vector<bool> is_lane_avail{true, true, true};
              if (car_d<4){
                is_lane_avail[0] = false;
                is_lane_avail[2] = false;
              }
              else if (car_d>8){
                is_lane_avail[2] = false;
                is_lane_avail[0] = false;
              }
              else{
                is_lane_avail[1] = false;
              }
              // a target car closest to us in front of us in the lane to change
              vector<double> target_actor_s_dif{9999,9999,9999};
              vector<double> LEFT_BOUND{0,4,8}; // left boundary of lane
              // in the available lane, find if the buffer is enough to change lane, update target car stats
              for (const auto& actor : sensor_fusion){
                double actor_d = actor[6];
                // loop through lanes (j)
                for (int j=0; j<3; j++){
                  if ((is_lane_avail[j]) && (actor_d>LEFT_BOUND[j]) && (actor_d<LEFT_BOUND[j]+4)){
                    double check_actor_s = actor[5];
                    double actor_vx = actor[3];
                    double actor_vy = actor[4];
                    double check_speed = sqrt(actor_vx*actor_vx + actor_vy*actor_vy);
                    check_actor_s += prev_size * TIME_STEP * check_speed;
                    // the lane is not safe to use if the actor is in the buffer
                    if (check_actor_s>(car_s-BEHIND_BUFFER) && check_actor_s<(car_s+AHEAD_BUFFER)){
                      is_lane_avail[j] = false;
                    } else{
                      // update if the car is good to be a target car
                      if (check_actor_s-car_s<target_actor_s_dif[j]){
                        target_actor_s_dif[j] = check_actor_s-car_s;
                      }
                    } // end check buffer
                  } // end if lane available
                } // end lane loop
                // actor not in the same lane
                if (!(actor_d<(2+4*lane+2) && actor_d>(2+4*lane-2))){
                  double actor_vx = actor[3];
                  double actor_vy = actor[4];
                  double check_speed = sqrt(actor_vx*actor_vx + actor_vy*actor_vy);
                  double check_actor_s = actor[5];
                  // assume the actor will move at constant velocity
                  check_actor_s += prev_size * TIME_STEP * check_speed;
                  if (check_actor_s>car_s && check_actor_s<(car_s+30)){
                    too_close = true;
                  } 
                }
              }
              // decide which lane to change
              int lane_decision = -1;
              double target_actor_s_dif_decision = 0;
              for (int j=0; j<3; j++){
                if (is_lane_avail[j]){
                  // just 1 candidate
                  if (lane_decision==-1){
                    lane_decision = j;
                    target_actor_s_dif_decision = target_actor_s_dif[j];
                  } else if (target_actor_s_dif[j]>target_actor_s_dif_decision){ // 2 candidates
                    lane_decision = j;
                  }
                }
              }
              // change lane with constant velocity, i.e., add the reduced velocity back
              if (lane_decision!=-1){
                lane = lane_decision;
                ref_vel += 0.22;
                cool_down = COOL_DOWN_MAX;
              }
            }// end if ref_vel>LANE_CHANGE_VEL
          }// end if too_close
          else if (ref_vel<49.5){
            ref_vel += 0.22;
          }

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

          for (int i=0;i<NPOINTS-prev_size;i++){
            double cur_x = (i+1)*end_x/N;
            double cur_y = spl(cur_x);
            // transformation from car local frame to global frame
            double global_cur_x = cur_x * cos(ref_yaw_rad) - cur_y * sin(ref_yaw_rad);
            double global_cur_y = cur_x * sin(ref_yaw_rad) + cur_y * cos(ref_yaw_rad);
            next_x_vals.push_back(global_cur_x + ref_x);
            next_y_vals.push_back(global_cur_y + ref_y);
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