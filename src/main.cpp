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

  // Starting lane
  int lane = 1;
  bool changing_lane = false;
  
  // Target velocity (mph)
  double target_speed = 49.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &target_speed, &max_s]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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

          int prev_size = previous_path_x.size();

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // BEGIN PROJECT CODE //

          // If we have a previous path, use the end of that path as the "car's location"
          if(prev_size > 0) {
            car_s = end_path_s;
          }

          // Resetting reference velocity to 49 mph
          target_speed = 49;

          // Value to make sure we always focus on the closest vehicle in front of the Ego vehicle
          double closest_car = std::numeric_limits<double>::max();

          // Iterating through all sensor fusion detections
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            // Getting detected car's lane
            float d = sensor_fusion[i][6];

            // If detected car is in Ego vehicle's lane
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
              // Getting velocity and frenet coordinate of detected vehicle
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double det_veh_speed = sqrt(vx * vx + vy * vy);
              double det_veh_s = sensor_fusion[i][5];

              // If we are using previous points, we can project location out in time
              det_veh_s += (static_cast<double>(prev_size) * 0.02 * det_veh_speed);
              // Resetting S coordinate if we exceeded max_s
              det_veh_s = fmod(det_veh_s, max_s);

              // If the detected vehicle is less than 25m in front of us
              if ((det_veh_s > car_s && det_veh_s - car_s < 25) ||
                  ((car_s + 25 > max_s) && fmod(car_s + 25, max_s) > det_veh_s)) {
              
                // If the detected vehicle is the closest vehicle in front of the Ego vehicle
                // save it's distance - otherwise skip to next for loop iteration
                // Edge case handling track loop
                if (det_veh_s < car_s) {
                  if (fmod(det_veh_s + car_s, max_s) < closest_car) {
                    closest_car = max_s - car_s + det_veh_s;
                  } else {
                    continue;
                  }
                } 
                // If the detected vehicle is the closest vehicle in front of the Ego vehicle
                // save it's distance - otherwise skip to next for loop iteration
                // Standard case
                else {
                  if (det_veh_s - car_s < closest_car) {
                    closest_car = det_veh_s - car_s;
                  } else {
                    continue;
                  }
                }

                // If the detected vehicle is travelling slower than our target velocity
                if (det_veh_speed < 49.0 / 2.24) {
                  // Trying to change lane left
                  if (lane - 1 >= 0 && canChangeLanes(lane - 1, car_s, sensor_fusion)) {
                    lane = lane - 1;
                    target_speed = 49.0;
                  } 
                  // Can't change lane left - trying to change lane right
                  else if (lane + 1 <= 2 && canChangeLanes(lane + 1, car_s, sensor_fusion)) {
                    lane = lane + 1;
                    target_speed = 49.0;
                  } 
                  // Can't change lane left or right but detected vehicle is more than 30m ahead
                  // Accelerating
                  else if (closest_car > 30) {
                    target_speed = 49;
                  } 
                  // Can't change lane left or right and vehicle is still within 30m
                  // Decelerating
                  else {
                    target_speed = (det_veh_speed * 2.24) - 5;
                  }
                }
              } 
            }
          }

          // Vectors to hold the points used to create the spline
          vector<double> spline_x_vals;
          vector<double> spline_y_vals;

          // Setting spline points to ego vehcile coordinates
          double spline_x_1 = car_x;
          double spline_y_1 = car_y;
          // Getting Ego vehicle yaw in radians
          double ego_yaw = deg2rad(car_yaw);
          // Declaring varible to hold the current value of the gap between waypoints
          // which cooresponds to the vehcile speed 
          double current_gap;

          // If we do not have enough previous waypoints
          if (prev_size < 2) {
            // Projecting an (x,y) backwards in time
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // Adding inital two points to spline point vectors
            spline_x_vals.push_back(prev_car_x);
            spline_x_vals.push_back(car_x);
            spline_y_vals.push_back(prev_car_y);
            spline_y_vals.push_back(car_y);
            // Vehcile is not moving so current gap is 0
            current_gap = 0.0;
          }
          else {  // We do have previous waypoints
            // Setting first two spline points as the last two waypoints in the previous path
            spline_x_1 = previous_path_x[prev_size - 1];
            spline_y_1 = previous_path_y[prev_size - 1];
            double spline_x_0 = previous_path_x[prev_size - 2];
            double spline_y_0 = previous_path_y[prev_size - 2];
            // Setting the ego vehicles yaw to the value it will be at the end of the previous path
            ego_yaw = atan2(spline_y_1 - spline_y_0, spline_x_1 - spline_x_0);

            // Adding inital two points to spline point vectors
            spline_x_vals.push_back(spline_x_0);
            spline_x_vals.push_back(spline_x_1);
            spline_y_vals.push_back(spline_y_0);
            spline_y_vals.push_back(spline_y_1);

            // Calculating current gap at the end of the previous path
            current_gap = sqrt(pow(spline_x_1 - spline_x_0, 2) + pow(spline_y_1 - spline_y_0, 2));
          }

          // Calculating 3 points to use to create the spline
          // Points are 30m, 60m, and 90m ahead of the end of the
          // previous path alone the lane contour (frenet)
          double dist_1 = fmod(car_s + 30, max_s);
          double dist_2 = fmod(car_s + 60, max_s);
          double dist_3 = fmod(car_s + 90, max_s);


          // Getting (x,y) locations for points projected along the lanes
          vector<double> spline_pt_0 =
            getXY(dist_1, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_pt_1 =
            getXY(dist_2, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_pt_2 =
            getXY(dist_3, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Pushing the projected points into the spline point vectors
          spline_x_vals.push_back(spline_pt_0[0]);
          spline_x_vals.push_back(spline_pt_1[0]);
          spline_x_vals.push_back(spline_pt_2[0]);

          spline_y_vals.push_back(spline_pt_0[1]);
          spline_y_vals.push_back(spline_pt_1[1]);
          spline_y_vals.push_back(spline_pt_2[1]);


          // Shifting spline points into car's reference frame
          for (int i = 0; i < spline_x_vals.size(); ++i) {
            double shift_x = spline_x_vals[i] - spline_x_1;
            double shift_y = spline_y_vals[i] - spline_y_1;
            spline_x_vals[i] = (shift_x * cos(0 - ego_yaw) - shift_y * sin(0 - ego_yaw));
            spline_y_vals[i] = (shift_x * sin(0 - ego_yaw) + shift_y * cos(0 - ego_yaw));
          }

          // Creating spline for smooth vehicle path
          tk::spline spline;

          // Building the spline with the intial and projected points
          spline.set_points(spline_x_vals, spline_y_vals);

          // Defining the actual points to be used for the planner
          vector<double> planned_path_x;
          vector<double> planned_path_y;

          // Pushing all previous path points into the vectors of waypoints to
          // send to the Ego vehicle
          for (int i = 0; i < previous_path_x.size(); ++i) {
            planned_path_x.push_back(previous_path_x[i]);
            planned_path_y.push_back(previous_path_y[i]);
          }

          // Value to hold x location of last waypoint
          double current_x = 0;

          // Calculating the target distance between waypoints based on the
          // target speed
          double target_gap = (target_speed / 2.24) * 0.02;

          // Looping to generate make sure we generate 50 planned path points
          for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
            
            // Acceration value is the amount that we increase the gap between
            // waypoints each time step.  This value is linearly proportional
            // to the differnce between the target gap and current gap
            double delta_gap = fabs(target_gap - current_gap) / 0.447 * 0.0035;

            // Setting the current gap size
            if (fabs(target_gap - current_gap) > 0.005) {
              if (target_gap > current_gap) {
                current_gap += delta_gap;
              } else if (target_gap < current_gap) {
                current_gap -= delta_gap;
              }
            }

            // Getting the x and y points along the spline
            double x_point = current_x + current_gap;
            double y_point = spline(x_point);
            // Incrementing the current x position
            current_x = x_point;

            // Reference (x,y) for transformation
            double x_ref = x_point;
            double y_ref = y_point;

            // Transforming (x,y) points back to world coordinate frame
            x_point = (x_ref * cos(ego_yaw) - y_ref * sin(ego_yaw));
            y_point = (x_ref * sin(ego_yaw) + y_ref * cos(ego_yaw));

            // Adding (x,y) values onto the end of the previous path
            x_point += spline_x_1;
            y_point += spline_y_1;

            // Pushing waypoints into planned path vectors
            planned_path_x.push_back(x_point);
            planned_path_y.push_back(y_point);
          }

          // END PROJECT CODE

          json msgJson;
          msgJson["next_x"] = planned_path_x;
          msgJson["next_y"] = planned_path_y;

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