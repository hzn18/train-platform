/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 13:30:42 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 15:45:53
 */
#include "leader_controller.h"

#include "leader_MPC.h"
#include "constant.h"

using namespace std;

vector<vector<double>> LeaderController(double space, double speed, vector<pair<double, double>>& speed_max_info, int speed_max_info_index){
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
    return LeaderMPCCalculate(space, speed, speed_max_info_part);
}



vector<vector<double>> DataDrivenLeaderController(double space, double speed, vector<vector<double>>& sample_safe_set, int sample_safe_set_start_index){
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
        else {
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
    return DataDrivenLeaderMPCCalculate(space, speed, sample_set);
}

