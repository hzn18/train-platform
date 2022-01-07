/*
 * @Author: houzhinan 
 * @Date: 2022-01-03 20:11:50 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-05 14:13:59
 */

#include <string>
#include <math.h>
#include <fstream>

#include "logger.h"
#include "constant.h"
#include "filename.h"

#include "read_speed_max.h"
#include "read_speed_limit.h"
#include "leader_controller.h"
#include "dynamic_model.h"

using namespace std;

// consider speed and energy.
double ref_cost(double v, double u){
    double u_max = M * a_dr;
    return K_dp_v * (1 - v / v_max) * (1 - v / v_max) + K_dp_u * u / u_max * u / u_max; 
}

void save_result(vector<vector<double>> result, string filename){
	ofstream fout(filename);
    for(auto train_state : result){
        fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << endl;
    }
}

vector<vector<double>> MergeSampleSafeSet(vector<vector<double>> sample_set_1, vector<vector<double>> sample_set_2){
	// 归并排序
	vector<vector<double>> merged_sample_set;

    int set1_len = sample_set_1.size();
	int set2_len = sample_set_2.size();
	int index1 = 0;
	int index2 = 0;
	while(index1 < set1_len && index2 < set2_len){
		if(sample_set_1[index1][0] <= sample_set_2[index2][0])
		    merged_sample_set.push_back(sample_set_1[index1++]);
		else 
		    merged_sample_set.push_back(sample_set_2[index2++]);
	}
	while(index1 < set1_len)
	    merged_sample_set.push_back(sample_set_1[index1++]);
	while(index2 < set2_len)
	    merged_sample_set.push_back(sample_set_2[index2++]);

	return merged_sample_set;
}

vector<vector<double>> InitSampleIteration(vector<pair<double, double>> speed_max_info){
    double v_stable = 6;

    vector<vector<double>> result; //space, speed, function
	vector<vector<double>> sample_set;

	double space = 0;
	double speed = 0;

	int speed_max_info_index = 0;

	while(1){
		double function;
		if(space < 0.8 * speed_max_info[speed_max_info.size()-1].first){
			function = (speed < v_stable) ? M * a_dr : -M * a_br; 
		}
		else{
			// calculate the function
	    	vector<vector<double>> mpc_list = LeaderController(space, speed, speed_max_info, speed_max_info_index);
			function = mpc_list[0][2];
		}
        // control the train 
		vector<double> state = DynamicModel(function, space, speed);
        
		space = state[0];

		speed = state[1];

        speed_max_info_index = floor(space / delta_s);

		result.push_back(vector<double>{space, speed, function});
		sample_set.push_back(vector<double>{space, speed, ref_cost(speed, function)});

        LOGGER.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end
		if(speed_max_info[speed_max_info.size() - 1].first - space < 1 )
			break;
	}

    string save_filename = DATA_DRIVEN_LEADER_OUTPUT_DIR;
	save_filename +="leader_result" + to_string(0) + ".txt";

	save_result(result, save_filename);
    
	double tmp = 0;
	for(int i = sample_set.size()-1; i>=0 ;i--){
        sample_set[i][2] += tmp;
		tmp = sample_set[i][2];
	}

	return sample_set;
}

vector<vector<double>> RegularIteration(int iter_index, vector<pair<double, double>> speed_limit_info, vector<vector<double>> sample_safe_set){
    vector<vector<double>> result; //space, speed, function
	vector<vector<double>> one_iter_sample_safe_set;

	double space = 0;
	double speed = 0;

	int sample_safe_set_index = 0;

	while(1){
		// calculate the function
	    vector<vector<double>> mpc_list = DataDrivenLeaderController(space, speed, speed_limit_info, 0, sample_safe_set, sample_safe_set_index);
		double function = mpc_list[0][2];
        
		// control the train 
		vector<double> state = DynamicModel(function, space, speed);
        
		space = state[0];
		speed = state[1];

        // change sample_safe_set_index
		for(int i = sample_safe_set_index; i+1 < sample_safe_set.size() && sample_safe_set[i + 1][0] < space; i++){
            sample_safe_set_index++;
		}

		result.push_back(vector<double>{space, speed, function});
		one_iter_sample_safe_set.push_back(vector<double>{space, speed, ref_cost(speed, function)});

        if(speed < 0)
		    break;

        LOGGER.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end
		if(sample_safe_set[sample_safe_set.size() - 1][0] - space < 0.5)
			break;
	}


    string save_filename = DATA_DRIVEN_LEADER_OUTPUT_DIR;
	save_filename += "leader_result" + to_string(iter_index) + ".txt";

	save_result(result, save_filename);
    
	double tmp = 0;
	for(int i = one_iter_sample_safe_set.size()-1; i>=0 ;i--){
        one_iter_sample_safe_set[i][2] += tmp;
		tmp = one_iter_sample_safe_set[i][2];
	}

	return one_iter_sample_safe_set;
}

int main(){
	// read max speed info
	vector<pair<double, double>> speed_max_info = ReadSpeedMax(DP_SAFE_OUTPUT_FILENAME);

	vector<pair<double, double>> speed_limit_info = ReadSpeedLimit(DB_FILENAME);
    
	vector<pair<double, double>> transformed_speed_limit_info;
	for(int i = 0; i < speed_limit_info.size() - 1; i++){
		transformed_speed_limit_info.push_back(make_pair(speed_limit_info[i].first, speed_limit_info[i].second));
	}

    // init sample safe set
    vector<vector<double>> sample_safe_set = InitSampleIteration(speed_max_info);

    LOGGER.info("sample safe set length is {}", sample_safe_set.size());

    // iteration -> sample safe set
	for(int iter = 1; iter < 2; iter++){
		vector<vector<double>> iter_sample_set = RegularIteration(iter, transformed_speed_limit_info, sample_safe_set);
        sample_safe_set = MergeSampleSafeSet(sample_safe_set, iter_sample_set);
		LOGGER.info("sample safe set length is {}", sample_safe_set.size());
	}
}
