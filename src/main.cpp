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
#include "transforms.h"
#include "obstacles.h"

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
  int LANE_ID = 1; //Initial lane
  double REF_V = 0.0; //reference velocity in mph


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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &LANE_ID, &REF_V](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
          	double car_speed = j[1]["speed"]; //mph

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	//Define all flags and constats

            double MAX_V = 49.5; //maximum velocity in mph
            int N_ANCHORS = 5; 
            int ANCHOR_SPACING = 30;
            double SPLINE_X_TARGET = 80; //Distance in meters that the path looks out to
            double SPLINE_X_INCREMENT = 0;
            int N_SPLINE_POINTS = 40;
            double TIME_INTERVAL = 0.02;
            double LANE_WIDTH = 4.0;
            double MIN_SAFE_GAP = 40.0; //minimum safe distance to maintain in meters
            double MAX_ACC = 10.0; //max acceleration in m/s2
            double MAX_JERK = 10.0; // max jerk in m/s3
            bool OBSTACLE_TOO_CLOSE = false;
            double MAX_VEL_CHANGE = MAX_ACC * TIME_INTERVAL * 2.23694; //max permissible change in velocity in mph
            double PERMISSIBLE_VEL_CHANGE = 0.8 * MAX_VEL_CHANGE;
            bool LEFT_LANE_CLEAR = true;
            bool RIGHT_LANE_CLEAR = true;

            //Define all variables
            vector<double> next_x_vals; //holds the next global x values
          	vector<double> next_y_vals; //holds the next global y values
            //vector<Obstacle> obstacles; //holds the detected vehicles
            int prev_size = previous_path_x.size(); //size of the previous path returned from the simulator
            double spline_y_target;
            double path_horizon;
            double target_v; //target velocity to track
            double ego_end_s;
            vector<Obstacle> obstacles;
            

            //Start the path with all the remaining points from the previous path
            for(int i=0; i<prev_size; ++i){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //Declare anchor points for use in spline
            vector<double> anchor_ptsx;
            vector<double> anchor_ptsy;

            //Declare spline
            tk::spline spline_traj;

            //Capture two points to use as anchor points for spline generation
            //Capture the current state of the ego vehicle to use as the reference state
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            double ref_s = car_s;
            //Define variables to hold the state previous to the reference state
            double prev_x;
            double prev_y;

            if (prev_size>0){
              ref_s = end_path_s;
            }


            //Sensor fusion is always a list of 12 vehicles of ID 0-11 in the order in which they are detected, I think.
            //If the number of detected vehicles is < 12, the corresponding d values are large & negative and S values are > 6000.
            for(int i=0; i<sensor_fusion.size(); ++i){
              //cout<<"Obstacle ID: "<<sensor_fusion[i][0]<<" s-dist: "<<(double)sensor_fusion[i][5] - ref_s<<" d: "<<sensor_fusion[i][6]<<endl;
              //Loop over all sensor fusion data and collect valid detected obstacles
              Obstacle obstacle;  
              obstacle.d = sensor_fusion[i][6];
              
              if (obstacle.d >= 0){
                obstacle.id = sensor_fusion[i][0];
                obstacle.x = sensor_fusion[i][1];
                obstacle.y = sensor_fusion[i][2];
                obstacle.v = distance(0.0, 0.0, (double)sensor_fusion[i][3], (double)sensor_fusion[i][4]) * 2.23694;
                obstacle.s = sensor_fusion[i][5];
                obstacle.lane = (int)obstacle.d/LANE_WIDTH;
                obstacle.distance = obstacle.s - car_s;
                obstacle.future_distance = obstacle.s + TIME_INTERVAL*(double)prev_size*obstacle.v/2.23694 - ref_s; 
                obstacles.push_back(obstacle);
                //Check for obstacles one lane to the right within the safe gap or safe gap/2 for vehicles behind us (more aggressive on lane changes)
                if ((LANE_ID+1 == obstacle.lane) && (((fabs(obstacle.future_distance) < MIN_SAFE_GAP) && (obstacle.future_distance>0)) || ((fabs(obstacle.future_distance) < MIN_SAFE_GAP/2.0) && (obstacle.future_distance<0)))  ){
                  cout<<"Found obstacle in lane "<<obstacle.lane<<" at distance: "<<obstacle.future_distance<<endl;
                  RIGHT_LANE_CLEAR = false;

                  }
                  
                //Check for obstacles one lane to the left within the safe gap or safe gap/2 for vehicles behind us (more aggressive on lane changes)      
                if ((LANE_ID-1 == obstacle.lane) && (((fabs(obstacle.future_distance) < MIN_SAFE_GAP) && (obstacle.future_distance>0)) || ((fabs(obstacle.future_distance) < MIN_SAFE_GAP/2.0) && (obstacle.future_distance<0)))  ){
                  cout<<"Found obstacle in lane "<<obstacle.lane<<" at distance: "<<obstacle.future_distance<<endl;
                  LEFT_LANE_CLEAR = false;
  
                  }
                  
                }
              } 

            for(int i=0; i<obstacles.size(); ++i){

              //Check if the car is in front of us, in the same lane and closer than the defined safe gap
              if((0 < obstacles[i].future_distance) && (obstacles[i].future_distance <= MIN_SAFE_GAP) && (obstacles[i].lane == LANE_ID)){
                //Move to the left lane
                if(LEFT_LANE_CLEAR && (LANE_ID != 0)){
                  cout<<"My lane is: "<<LANE_ID<<endl;
                  LANE_ID -= 1;
                  cout << "Changing to Lane: "<<LANE_ID<<endl;
                } //otherwise move to the right lane
                else if(RIGHT_LANE_CLEAR && (LANE_ID != 2)){
                  cout<<"My lane is: "<<LANE_ID<<endl;
                  LANE_ID += 1;
                  cout << "Changing to Lane: "<<LANE_ID<<endl;;
                } //otherwise slow down
                else{
                  OBSTACLE_TOO_CLOSE = true;
                  cout<<"Tracking obstacle velocity!"<<endl;
                  target_v = obstacles[i].v;
                  break;  
                }
                
              }
            }

            if(OBSTACLE_TOO_CLOSE && (REF_V > target_v)){
                REF_V -= 0.3*PERMISSIBLE_VEL_CHANGE; //gradually slow down
              }
            else if (REF_V < MAX_V && !OBSTACLE_TOO_CLOSE){
                REF_V += PERMISSIBLE_VEL_CHANGE;
                //cout<<"Increasing velocity by: "<<PERMISSIBLE_VEL_CHANGE<<endl;
              }

            //Create an additional point based on current reference state if the previous path has one point left
            if (prev_size < 2){
              prev_x = ref_x - cos(ref_yaw);
              prev_y = ref_y - sin(ref_yaw);
            }
            //Otherwise use the last two points available in the previous path
            else {
              /*for (int i=0; i<4; ++i){
                cout<<"Previous path point "<<i+1<<": ("<<previous_path_x[prev_size-1-i]<<","<<previous_path_y[prev_size-1-i]<<")\n";
              }*/
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              ref_yaw = atan2(ref_y-prev_y, ref_x-prev_x);
              prev_x = previous_path_x[prev_size-2];
              prev_y = previous_path_y[prev_size-2];
            }
            //Stack these points onto the anchor points vector
            anchor_ptsx.push_back(prev_x);
            anchor_ptsx.push_back(ref_x);
            anchor_ptsy.push_back(prev_y);
            anchor_ptsy.push_back(ref_y);

            //Create the remaining anchor points using the anchor point spacing and the current s-coordinate of the ego vehicle
            for (int i=0; i<N_ANCHORS-2; ++i){
              vector<double> anchor_pt = getXY(ref_s+(ANCHOR_SPACING*(i+1)), (2+4*LANE_ID), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              anchor_ptsx.push_back(anchor_pt[0]);
              anchor_ptsy.push_back(anchor_pt[1]);
            }

            //Rotate anchor points to car's frame of reference
            for (int i=0;i<anchor_ptsx.size();++i){
              //View the original anchor points
              /*cout<<"Anchor x: "<<anchor_ptsx[i]<<" y: "<<anchor_ptsy[i]<<endl;
              cout<<"-------------"<<endl;*/
              //offset the axis origin to the car location
              double shift_x = anchor_ptsx[i] - ref_x;
              double shift_y = anchor_ptsy[i] - ref_y;

              //Rotate the waypoint coordinates clockwise by psi
              //Waypoint coordinates are x, z
              //Vehicle coordinates are x', z'
              //Psi is measured positive from x to z
              //https://en.wikipedia.org/wiki/Transformation_matrix
              
              anchor_ptsx[i] = (shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw));
              anchor_ptsy[i] = (shift_x * -sin(ref_yaw) + shift_y * cos(ref_yaw));
              //cout<<"Anchor point "<<i+1<<": ("<<anchor_ptsx[i]<<','<<anchor_ptsy[i]<<")\n";
            }

            //Create the spline
            spline_traj.set_points(anchor_ptsx, anchor_ptsy); //Spline is created in ego vehicle's frame of reference

            //Pick out points from the spline
            spline_y_target = spline_traj(SPLINE_X_TARGET);
            path_horizon = distance(0.0, 0.0, SPLINE_X_TARGET, spline_y_target);

            for (int i=1; i<=N_SPLINE_POINTS-prev_size; ++i){
              //Causes unpredictable errors during startup
              /*if (REF_V < MAX_V && !OBSTACLE_TOO_CLOSE){
                REF_V += 0.8*PERMISSIBLE_VEL_CHANGE;
                //cout<<"Increasing velocity by: "<<PERMISSIBLE_VEL_CHANGE<<endl;
              }*/
              double N = path_horizon/(TIME_INTERVAL*REF_V/2.23694); //Create the spacing for the points
              double spline_x_pt_car = SPLINE_X_INCREMENT + (SPLINE_X_TARGET)/N; //Get the next car x-coordinate
              double spline_y_pt_car = spline_traj(spline_x_pt_car); //Get the next car y-coordinate

              SPLINE_X_INCREMENT = spline_x_pt_car; //increment x position forward

              //Rotate these points back to the global frame of reference
              double spline_x_pt_global = spline_x_pt_car*cos(ref_yaw) - spline_y_pt_car*sin(ref_yaw);
              double spline_y_pt_global = spline_x_pt_car*sin(ref_yaw) + spline_y_pt_car*cos(ref_yaw);
              //offset the points from the ego vehicle reference state
              spline_x_pt_global += ref_x; 
              spline_y_pt_global += ref_y;

              next_x_vals.push_back(spline_x_pt_global);
              next_y_vals.push_back(spline_y_pt_global);
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
