/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 13:30:42 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 14:37:59
 */
#include "leader_controller.h"

#include "leader_MPC.h"
#include "constant.h"

using namespace std;

vector<vector<double>> LeaderController(double space, double speed, vector<pair<double, double>>& speed_max_info, int speed_max_info_index, string mpc_filename ,spdlog::logger& logger){
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
    return LeaderMPCCaculate(space, speed, speed_max_info_part, mpc_filename, logger);
}




