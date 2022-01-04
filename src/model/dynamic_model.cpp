/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 12:57:36 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 14:41:16
 */

#include "dynamic_model.h"

#include <cmath>

#include "constant.h"

using namespace std;
// Giving the controlled driving or braking force, this function will transform 
// state of train. 
// Return: train state [s, v]

vector<double> DynamicModel(double function, double space, double speed){
    double a = (function - A - B*speed  - T_f_C*speed*speed) / M;
    double v = speed + a * Ts;
	double s = space + speed * Ts + 0.5 * a * Ts * Ts;
	return vector<double>({s, v});
}

/*
vector<double> DynamicModelByDistance(double function, double space, double speed){
    double a = (function - A - B*speed  - T_f_C*speed*speed) / M;
    double temp = speed * speed + 2 * a * delta_s_model;
    double v = temp >= 0 ? sqrt(temp) : 0;
    double s = space + delta_s_model;
    return vector<double>({s, v});
}
*/
