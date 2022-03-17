/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 13:30:42 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-05 14:12:14
 */
#include "leader_controller.h"

#include <iostream>
#include <math.h>
#include "logger.h"
#include "leader_MPC.h"
#include "constant.h"

using namespace std;

vector<pair<double, double>> CutSpeedMaxInfo(vector<pair<double, double>>& speed_max_info, int speed_max_info_index, int Np, double Ts){
    double max_delta_space =  v_max * Ts * Np;
    int delta_index = ceil(max_delta_space / delta_s);
    if(delta_index + speed_max_info_index > speed_max_info.size() - 1)
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

vector<pair<double, double>> CutSpeedLimitInfo(vector<pair<double, double>>& speed_limit_info, int speed_limit_info_index, int Np, double Ts){
    //TODO:
    return speed_limit_info;
}

vector<vector<double>> CutSampleSafeSet(double space, vector<vector<double>>& sample_safe_set, int sample_safe_set_start_index, int Np, double Ts){
    // find points from sample safe set
    double max_delta_space =  v_max * Ts * Np;
    // [space, space + max_delta_space] -> [sample_safe_set_start_index, sample_safe_set_end_index]
    // binary search
    int left_index = sample_safe_set_start_index;
    int right_index = sample_safe_set.size() - 1;
    int mid = (left_index + right_index) >> 1;
    while(left_index <= right_index){
        if(sample_safe_set[mid][0] < space + max_delta_space){
            left_index = mid + 1;
        } 
        else{
            right_index = mid - 1;
        }
        mid = (left_index + right_index) >> 1;
    }
    int sample_safe_set_end_index = right_index;
    // get a partly sample set
    vector<vector<double>> sample_set;
    for(int i = sample_safe_set_start_index; i<= sample_safe_set_end_index; i++){
        sample_set.push_back(sample_safe_set[i]);
    }
    return sample_set;
}

vector<vector<double>> LeaderController(double space, double speed, vector<pair<double, double>>& speed_max_info, int speed_max_info_index){
    vector<pair<double, double>> speed_max_info_part = CutSpeedMaxInfo(speed_max_info, speed_max_info_index, NP_, TS);
    return LeaderMPCCalculate(space, speed, KV, KU, NP_, TS, speed_max_info_part);
}



vector<vector<double>> DataDrivenLeaderController(double space, double speed, vector<pair<double, double>>& speed_limit_info, int speed_limit_info_index, vector<vector<double>>& sample_safe_set, int sample_safe_set_start_index){
    vector<pair<double, double>> speed_limit_info_part = CutSpeedLimitInfo(speed_limit_info, speed_limit_info_index, NP_DATA_DRIVEN, Ts_DATA_DRIVEN);
    vector<vector<double>> sample_set = CutSampleSafeSet(space, sample_safe_set, sample_safe_set_start_index, NP_DATA_DRIVEN, Ts_DATA_DRIVEN);
    return DataDrivenLeaderMPCCalculate(space, speed, KV_DATA_DRIVEN, KU_DATA_DRIVEN, NP_DATA_DRIVEN, Ts_DATA_DRIVEN, speed_limit_info_part, sample_set);
}

