/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 12:57:36 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 14:45:45
 */

#include "dynamic_model.h"

#include "constant.h"

using namespace std;
// Giving the controlled driving or braking force, this function will transform 
// state of train. 
// Return: train state [s, v]

// TODO: 
vector<double> DynamicModel(double function, double space, double speed){
    double a = (function - A - B*speed  - T_f_C*speed*speed) / M;
    double v = speed + a * Ts;
	double s = space + speed * Ts + 0.5 * a * Ts * Ts;
	return vector<double>({s, v});
}
