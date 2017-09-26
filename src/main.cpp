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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	double car_speed = j[1]["speed"];

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

            // Experiment 4: Stay in current lane as if there's only ego car, and
            //  use previous path waypoints to smooth trajectory.

            double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size();
            std::cout << "previous path size = " << path_size << std::endl;

            int lane = 1;
            double ref_v = 49.0;

            // compensate ego car for previous points
            if (path_size > 0)
            {
                car_s = end_path_s;
            }

            // Sensor fustion processing
            for(int i=0; i<sensor_fusion.size(); i++)
            {
                float d = sensor_fusion[i][6];

                // Other vehicle in current ego lane
                if (d<(2+4*lane+2) && d>(2+4*lane-1) )
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double car_in_lane_speed = sqrt(vx*vx+vy*vy);
                    double car_in_lane_s = sensor_fusion[i][5];

                    // Compensate for previous points on car_in_lane
                    car_in_lane_s += ((double)path_size*0.02*car_in_lane_speed);

                    // Check collision with the car_in_lane
                    if ((car_in_lane_s>car_s) && ((car_in_lane_s-car_s)<30))
                    {
                        ref_v = 29.0;
                    }
                }
            }

            //
            // Previous points left over processing
            //
            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // prepare for spline points from previous points batch
            vector<double> tkptsx;
            vector<double> tkptsy;
            double pos_x2, pos_y2;

            if(path_size < 2)
            {
                // Current car pos serves as the left-over points
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);

                tkptsx.push_back(pos_x);
                tkptsy.push_back(pos_y);
            }
            else
            {
                // Last two previous batch points
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];

                pos_x2 = previous_path_x[path_size-2];
                pos_y2 = previous_path_y[path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

                // push sequence matters. spline.h expects ascending order
                tkptsx.push_back(pos_x2);
                tkptsy.push_back(pos_y2);
                // Cornercase, add only one point
                if (angle!=0.0)
                {
                    tkptsx.push_back(pos_x);
                    tkptsy.push_back(pos_y);
                }
            }
            std::cout << "angle = " << angle << std::endl;

            // Add more points for spline curve generation
            // Pick 30 meters and 60 meters down the road in Frenet as anchor points
            vector<double>  nxt_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double>  nxt_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double>  nxt_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            tkptsx.push_back(nxt_wp0[0]);
            tkptsx.push_back(nxt_wp1[0]);
            tkptsx.push_back(nxt_wp2[0]);

            tkptsy.push_back(nxt_wp0[1]);
            tkptsy.push_back(nxt_wp1[1]);
            tkptsy.push_back(nxt_wp2[1]);

            // Convert spline anchor points from global x,y coordinates into local one
            //  originating from last waypoint of last batch (or ego position if no last waypoints).
            // Spline fit
            std::cout << "tkptsx = ";
            for (int i=0; i<tkptsx.size(); i++)
            {
                std::cout << tkptsx[i] << " ";
            }
            std::cout << std::endl;

            for (int i=0; i< tkptsx.size(); i++)
            {
                double shifted_x = tkptsx[i]-pos_x;
                double shifted_y = tkptsy[i]-pos_y;

                tkptsx[i] = (shifted_x*cos(0-angle) - shifted_y*sin(0-angle));
                tkptsy[i] = (shifted_x*sin(0-angle) + shifted_y*cos(0-angle));
            }


            // Spline fit
            std::cout << "(transformed) tkptsx = ";
            for (int i=0; i<tkptsx.size(); i++)
            {
                std::cout << tkptsx[i] << " ";
            }
            std::cout << std::endl;

            tk::spline spline;
            spline.set_points(tkptsx, tkptsy);

            // Generate intermediate waypoints to target point s=30.
            double target_x = 30.0; // Move forward 30 meters
            double target_y = spline(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);

            for(int i = 0; i < 50-path_size; i++)
            {
                // Divide target length into equal segments based on reference speed
                double N = target_dist/(0.02*ref_v/2.24);
                double x_i = i*target_x/N;
                double y_i = spline(x_i);

                double tmp_x = x_i;
                double tmp_y = y_i;
                // Rotate and translate coordinates back into global x,y coordinates
                x_i = tmp_x*cos(angle) - tmp_y*sin(angle) + pos_x;
                y_i = tmp_x*sin(angle) + tmp_y*cos(angle) + pos_y;

                vector<double>  tmp = getFrenet(x_i, y_i, angle, map_waypoints_x, map_waypoints_y);
                std::cout << "converted s: " << tmp[0] << ", converted d: " << tmp[1] << std::endl;

                next_x_vals.push_back(x_i);
                next_y_vals.push_back(y_i);
            }

            //double dist_inc = 0.3;
            //vector<double> pos_xy(2);
            //vector<double> pos_sd(2);
            //for(int i = 0; i < 50-path_size; i++)
            //{
            //    //next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
            //    //next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
            //    //pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            //    //pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            //    // Convert last waypoints from previous path in (x,y,theta) to (s,d)
            //    pos_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
            //    pos_sd[1] = 6;
            //    // Convert back of advancement in Frenet coordinates
            //    pos_xy = getXY(pos_sd[0]+(dist_inc*(i+1)), pos_sd[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //    next_x_vals.push_back(pos_xy[0]);
            //    next_y_vals.push_back(pos_xy[1]);
            //}

            // END TODO

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

            this_thread::sleep_for(chrono::milliseconds(300));
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
