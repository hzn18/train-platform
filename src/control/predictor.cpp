/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:12:43 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 18:10:09
 */

#include "predictor.h"

#include <vector>

#include "constant.h"

using namespace std;

vector<vector<double>> NPPredictor(double space, double speed){
    vector<vector<double>> result;

    double function = -M * a_br;
    double v = speed;
	double s = space;
	for(int i = 0; i < NP_; i++){
        double a = (function - A - B * v  - T_f_C * v * v) / M;
        v += a * TS;
	    s += v * TS + 0.5 * a * TS * TS;
		result.push_back(vector<double>({s, v}));
	}

    return result;
}

vector<vector<double>> MBPredictor(double space){
    vector<vector<double>> result;
    for(int i = 0; i < NP_; i++){
		result.push_back(vector<double>({space, 0.001}));
	}

    return result;
}

vector<vector<double>> SHPredictor(vector<vector<double>> leader_mpc_result){
    return leader_mpc_result;
}

