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
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    Derivation: https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/db4b83a1-b304-4355-92c7-66e973102079/concepts/35ae38e8-3f68-4edf-9ea7-5b324d014d72
        or, here: http://www.shadmehrlab.org/book/minimum_jerk/minimumjerk.htm

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    MatrixXd A = MatrixXd(3,3);
    VectorXd b = MatrixXd(3,1);

    double T2 = T*T;
    double T3 = T*T2;
    double T4 = T*T3;
    double T5 = T*T4;
    A << T3, T4, T5,  3*T2, 4*T3, 5*T4,  6*T, 12*T2, 20*T3;
    b << end[0]-(start[0]+T*start[1]+0.5*T2*start[2]), end[1]-(start[1]+T*start[2]),
        end[2]-start[2];
    VectorXd x = MatrixXd(3,1);
    x = A.colPivHouseholderQr().solve(b);
    return {start[0],start[1],0.5*start[2],x[0],x[1],x[2]};

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

  int lane = 1; // Ego lane.
  int target_lane = 1; // Lane change target lane.
  double ref_v = 0.0; // Reference speed.
  bool startup_done = false; // Flag for initial start up.

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &target_lane, &ref_v, &startup_done](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            bool too_close = false;
            double close_dist = 30.0;

            double pos_x;
            double pos_y;
            double angle;
            int path_size = previous_path_x.size();
            //std::cout << "previous path size = " << path_size << std::endl;

            // compensate ego car for previous points
            if (path_size > 0)
            {
                car_s = end_path_s;
            }

            // closest cars in different lanes - initialized with large number to be overwritten.
            double left_fwd_closest = 5000;
            double left_bwd_closest = 5000;
            double right_fwd_closest = 5000;
            double right_bwd_closest = 5000;
            // Do not consider lane change at loop start/end
            if (car_s>50 && car_s<6900)
            {
                // Sensor fustion processing
                for(int i=0; i<sensor_fusion.size(); i++)
                {
                    float d = sensor_fusion[i][6];

                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double other_car_speed = sqrt(vx*vx+vy*vy);
                    double other_car_s = sensor_fusion[i][5];
                    // Compensate for previous points on car_in_lane
                    other_car_s += ((double)path_size*0.02*other_car_speed);

                    // left to ego lane
                    if (lane>0)
                    {
                        if (d<(2+4*(lane-1)+2) && d>(2+4*(lane-1)-2))
                        {
                            if (other_car_s>car_s && (other_car_s-car_s)<left_fwd_closest)
                                left_fwd_closest = other_car_s-car_s;
                            else if (other_car_s<car_s && (car_s-other_car_s)<left_bwd_closest)
                                left_bwd_closest = car_s-other_car_s;
                        }
                    }

                    // right to ego lane
                    if (lane<2)
                    {
                        if (d<(2+4*(lane+1)+2) && d>(2+4*(lane+1)-2))
                        {
                            if (other_car_s>car_s && (other_car_s-car_s)<right_fwd_closest)
                                right_fwd_closest = other_car_s-car_s;
                            else if (other_car_s<car_s && (car_s-other_car_s)<right_bwd_closest)
                                right_bwd_closest = car_s-other_car_s;
                        }
                    }

                    // Other vehicle in current ego lane
                    if (d<(2+4*lane+2) && d>(2+4*lane-2) )
                    {
                        // Check collision with the car_in_lane
                        if ((other_car_s>car_s) && ((other_car_s-car_s)<30))
                        {
                            too_close = true;
                            close_dist = other_car_s-car_s;
                        }
                    }
                }
            }

            // Speed adjustment - 0.4 delta translates to ~3m/s^2 acceleration in simulator
            if (too_close)
                ref_v -= min(0.8, 10.0/close_dist); // Slow down faster
		if (ref_v<0.0)
		    ref_v = 0.0;
            else if (ref_v < 49.5)
                ref_v += 0.3;

            // Initial start up flag
            if (car_speed > 25.0 && ~startup_done && too_close)
            {
                startup_done = true;
                //std::cout << "Start up process done!" << std::endl;
            }

            // Speed lower than threshold and safe to change lane, then change
            if (car_speed<45.0 && close_dist<30.0 && startup_done)
            {
                // Only change lane when previous lane change completed.
                // Please be noticed that condition is a bit stricter (only if car sits around lane center).
                bool lane_change_completed = car_d<(2+4*target_lane+1) && car_d>(2+4*target_lane-1);
                //std::cout << "Lane change completed. Current lane: " << target_lane << std::endl;

                if (lane_change_completed)
                {
                    if (lane>0 && left_fwd_closest>30 && left_bwd_closest>15) // left lane avail
                    {
                        lane -= 1;
                        target_lane = lane;
                        std::cout << "Trigger left lane change to LANE " << target_lane << std::endl;
                    }
                    else if (lane<2 && right_fwd_closest>30 && right_bwd_closest>15) // right lane avail
                    {
                        lane += 1;
                        target_lane = lane;
                        std::cout << "Trigger right lane change to LANE " << target_lane << std::endl;
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
            double pos_x3, pos_y3;

            if(path_size < 2)
            {
                // Current car pos serves as the left-over points
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);

                // Cornercase, add only one point
                if (angle!=0.0) // Smoother startup
                {
                    tkptsx.push_back(pos_x-sin(angle));
                    tkptsy.push_back(pos_y-cos(angle));
                }

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

                if (path_size>=3) {
                    pos_x3 = previous_path_x[path_size-3];
                    pos_y3 = previous_path_y[path_size-3];

                    // push sequence matters. spline.h expects ascending order
                    tkptsx.push_back(pos_x3);
                    tkptsy.push_back(pos_y3);
                }
                tkptsx.push_back(pos_x2);
                tkptsy.push_back(pos_y2);
                // Cornercase, add only one point
                if (angle!=0.0)
                {
                    tkptsx.push_back(pos_x);
                    tkptsy.push_back(pos_y);
                }
            }
//            std::cout << "angle = " << angle << std::endl;

            // Add more points for spline curve generation
            // Pick 30 meters and 60 meters down the road in Frenet as anchor points
            vector<double>  nxt_wp0;
            vector<double>  nxt_wp1;
            vector<double>  nxt_wp2;
            nxt_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            nxt_wp1 = getXY(car_s+50, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            nxt_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            tkptsx.push_back(nxt_wp0[0]);
            tkptsx.push_back(nxt_wp1[0]);
            tkptsx.push_back(nxt_wp2[0]);

            tkptsy.push_back(nxt_wp0[1]);
            tkptsy.push_back(nxt_wp1[1]);
            tkptsy.push_back(nxt_wp2[1]);

            // Convert spline anchor points from global x,y coordinates into local one
            //  originating from last waypoint of last batch (or ego position if no last waypoints).
            // Spline fit
//            std::cout << "tkptsx = ";
//            for (int i=0; i<tkptsx.size(); i++)
//            {
//                std::cout << tkptsx[i] << " ";
//            }
//            std::cout << std::endl;

            for (int i=0; i< tkptsx.size(); i++)
            {
                double shifted_x = tkptsx[i]-pos_x;
                double shifted_y = tkptsy[i]-pos_y;

                tkptsx[i] = (shifted_x*cos(0-angle) - shifted_y*sin(0-angle));
                tkptsy[i] = (shifted_x*sin(0-angle) + shifted_y*cos(0-angle));
            }

            // Spline fit
//            std::cout << "(transformed) tkptsx = ";
//            for (int i=0; i<tkptsx.size(); i++)
//            {
//                std::cout << tkptsx[i] << " ";
//            }
//            std::cout << std::endl;

            tk::spline spline;
            spline.set_points(tkptsx, tkptsy);

            // Generate intermediate waypoints to target point s=30.
            double target_x = 30.0; // Move forward 30 meters
            double target_y = spline(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);

            for(int i = 1; i < 50-path_size; i++)
            {
                // Divide target length into equal segments based on reference speed
                double N = target_dist/(0.02*max(2.0,ref_v)/2.25);
                double x_i = i*target_x/N;
                double y_i = spline(x_i);

                double tmp_x = x_i;
                double tmp_y = y_i;
                // Rotate and translate coordinates back into global x,y coordinates
                x_i = tmp_x*cos(angle) - tmp_y*sin(angle) + pos_x;
                y_i = tmp_x*sin(angle) + tmp_y*cos(angle) + pos_y;

                vector<double>  tmp = getFrenet(x_i, y_i, angle, map_waypoints_x, map_waypoints_y);
                //std::cout << "converted s: " << tmp[0] << ", converted d: " << tmp[1] << std::endl;

                next_x_vals.push_back(x_i);
                next_y_vals.push_back(y_i);
            }

            // END TODO

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(200));
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
