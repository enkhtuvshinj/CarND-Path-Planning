#include "trajectory.h"

vector<vector<double>> Trajectory(const vector<double>& car,
                                  const vector<double>& previous_path_x,
                                  const vector<double>& previous_path_y,
                                  const vector<double>& map_waypoints_x,
                                  const vector<double>& map_waypoints_y,
                                  const vector<double>& map_waypoints_s) {
  double car_x    = car[0];
  double car_y    = car[1];
  double car_s    = car[2];
  double car_yaw  = car[3];
  int lane        = car[4];
  int prev_size   = car[5];
  double ref_vel  = car[6];
        
  // create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will interpolate these waypoints with spline and fill it with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;
  
  // define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;
        
  // reference x, y, yaw
  // either we will reference the starting points as where the car is or at the previous paths end point
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  
  // if previous path is almost empty, use car as starting reference
  if(prev_size < 2)	{
    // use 2 points that make the path tangent tp the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // use the previous path's end points as starting reference
  else	{
    // redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    
    // use 2 points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  // in Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  for(int i=0; i<ptsx.size(); i++)	{
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    
    ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }
  
  // create a spline
  tk::spline s;
  
  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);
  
  // start with all of the previous path points from last time
  for(int i=0; i<previous_path_x.size(); i++)	{
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0;
  
  // fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  // simulation cannot run 50 points, maybe it run only 3 points and 47 points would be left.
  for(int i=1; i<= 50-previous_path_x.size(); i++)	{
    double N = (target_dist/(0.02*ref_vel/2.24));
    double x_point = x_add_on + (target_x/N);
    double y_point = s(x_point);
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
    
    x_point += ref_x;
    y_point += ref_y;
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  
  vector<vector<double>> trajectory;
  trajectory.push_back(next_x_vals);
  trajectory.push_back(next_y_vals);
  
  return trajectory;
}
