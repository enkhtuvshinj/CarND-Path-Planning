#ifndef PREDICTION_H
#define PREDICTION_H

#include <cmath>
#include <string>
#include <vector>

using std::vector;

// Parameters for NaiveBayesClassifier from offline training
typedef struct naive_bayes_parameters	{
  // Changing lane to left
  double left_stddev[4];
  double left_mean[4];
  double left_prior;
  // Changing lane to right
  double right_stddev[4];
  double right_mean[4];
  double right_prior;
  // Keep lane
  double keep_stddev[4];
  double keep_mean[4];
  double keep_prior;
  
} NaiveBayesParameters_t;

/* 
NaiveBayesParameters_t param;

param.left_mean[0] = 19.7141;
param.left_mean[1] = 5.05181;
param.left_mean[2] = 9.91413;
param.left_mean[3] = -0.967087;
param.left_stddev[0] = 12.2899;
param.left_stddev[1] = 2.36011;
param.left_stddev[2] = 0.990239;
param.left_stddev[3] = 0.663282;
param.left_prior = 0.285333;

param.keep_mean[0] = 20.3242;
param.keep_mean[1] = 3.68024;
param.keep_mean[2] = 9.99854;
param.keep_mean[3] = 0.00581204;
param.keep_stddev[0] = 11.4363;
param.keep_stddev[1] = 3.40382;
param.keep_stddev[2] = 1.06863;
param.keep_stddev[3] = 0.168126;
param.keep_prior = 0.421333;

param.right_mean[0] = 19.4772;
param.right_mean[1] = 2.93405;
param.right_mean[2] = 9.94717;
param.right_mean[3] = 0.954022;
param.right_stddev[0] = 12.0855;
param.right_stddev[1] = 2.31228;
param.right_stddev[2] = 0.952085;
param.right_stddev[3] = 0.646845;
param.right_prior = 0.293333; 
*/

vector<double> NaiveBayesClassifier(const NaiveBayesParameters_t &param, vector<double> observation);

vector<vector<double>> Predict(vector<vector<double>> observations, int prev_size);


#endif