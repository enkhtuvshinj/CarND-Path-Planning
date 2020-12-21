#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"

using std::vector;

vector<vector<double>> Trajectory(const vector<double>& car,
                                  const vector<double>& previous_path_x,
                                  const vector<double>& previous_path_y,
                                  const vector<double>& map_waypoints_x,
                                  const vector<double>& map_waypoints_y,
                                  const vector<double>& map_waypoints_s);

#endif