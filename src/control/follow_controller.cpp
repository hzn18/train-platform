/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:36:02 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 15:41:46
 */

#include "follow_controller.h"

#include <vector>

#include "follow_MPC.h"
#include "leader_MPC.h"
#include "constant.h"
#include "predictor.h"

using namespace std;

vector<pair<double, double>> CutSpeedMaxInfo(vector<pair<double, double>>& speed_max_info, int speed_max_info_index){
    double max_delta_space =  v_max * Ts * Np;
    int delta_index = ceil(max_delta_space / delta_s);
    if(delta_index + speed_max_info_index > speed_max_info.size()-1)
		delta_index = speed_max_info.size() - 1 - speed_max_info_index;
    double v_max_temp = 0;
    for(int i = 0; i < delta_index; i++){
        if(v_max_temp < speed_max_info[speed_max_info_index + i].second){
            v_max_temp = speed_max_info[speed_max_info_index + i].second;
		}
	}
    max_delta_space = v_max_temp * Ts * Np;
	delta_index = ceil(max_delta_space / delta_s);
	if(delta_index + speed_max_info_index > speed_max_info.size()-1)
		delta_index = speed_max_info.size() - 1 - speed_max_info_index;
    
    vector<pair<double, double>> speed_max_info_part;
	for(int i = 0; i <= delta_index;i++){
		speed_max_info_part.push_back(make_pair(speed_max_info[i + speed_max_info_index].first, speed_max_info[i + speed_max_info_index].second));
    }
    
    return speed_max_info_part;
}

vector<vector<double>> FollowController(double leader_space, double space, double speed, vector<pair<double, double>> speed_max_info, int speed_max_info_index){
    vector<pair<double, double>> speed_max_info_part = CutSpeedMaxInfo(speed_max_info , speed_max_info_index);

    vector<vector<double>> MB_predictor = MBPredictor(leader_space);

    return FollowMPCCalculate(MB_predictor, space, speed, speed_max_info_part);
}



vector<vector<double>> FollowController(double leader_space, double leader_speed, double space, double speed, vector<pair<double, double>> speed_max_info, int speed_max_info_index){
    vector<pair<double, double>> speed_max_info_part = CutSpeedMaxInfo(speed_max_info , speed_max_info_index);

    vector<vector<double>> NP_predictor = NPPredictor(leader_space, leader_speed);

    return FollowMPCCalculate(NP_predictor, space, speed, speed_max_info_part);
}

vector<vector<double>> FollowController(vector<vector<double>> leader_mpc_result, double space, double speed, vector<pair<double, double>> speed_max_info, int speed_max_info_index){
    vector<pair<double, double>> speed_max_info_part = CutSpeedMaxInfo(speed_max_info , speed_max_info_index);

    vector<vector<double>> SH_predictor = SHPredictor(leader_mpc_result);
    
    return FollowMPCCalculate(SH_predictor, space, speed, speed_max_info_part);
}
