/*
 * @Author: houzhinan 
 * @Date: 2022-01-03 20:11:50 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 15:10:46
 */

#include <string>
#include <math.h>
#include <fstream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "constant.h"
#include "read_speed_max.h"
#include "leader_controller.h"
#include "dynamic_model.h"

using namespace std;

string logger_filename = "./logs/log.txt";
string mpc_logger_filename = "./logs/mpc_log.txt";
string dp_input_filename = "./db/dp_safe_result.txt";
string leader_output_dir = "./user/result/";

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

vector<vector<double>> InitSampleIteration(vector<pair<double, double>> speed_max_info, spdlog::logger logger, spdlog::logger mpc_logger){
    double v_stable = 3;

    vector<vector<double>> result; //space, speed, function
	vector<vector<double>> sample_set;

	double space = 0;
	double speed = 0;

	int speed_max_info_index = 0;

	while(1){
		double function;
		if(space < 1600){
			function = (speed < v_stable) ? M * a_dr : -M * a_br; 
		}
		else{
			// calculate the function
	    	vector<vector<double>> mpc_list = LeaderController(space, speed, speed_max_info, speed_max_info_index, mpc_logger_filename, mpc_logger);
			function = mpc_list[0][2];
		}
        // control the train 
		vector<double> state = DynamicModel(function, space, speed);
        
		space = state[0];

		speed = state[1];

        speed_max_info_index = floor(space / delta_s);

		result.push_back(vector<double>{space, speed, function});
		sample_set.push_back(vector<double>{space, speed, ref_cost(speed, function)});

        logger.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end
		if(speed_max_info[speed_max_info.size() - 1].first - space < 1 )
			break;
	}

	save_result(result, leader_output_dir + "leader_result" + to_string(0) + ".txt");
    
	double tmp = 0;
	for(int i = sample_set.size()-1; i>=0 ;i--){
        sample_set[i][2] += tmp;
		tmp = sample_set[i][2];
	}

	return sample_set;
}

vector<vector<double>> RegularIteration(int iter_index, vector<vector<double>> sample_safe_set, spdlog::logger logger, spdlog::logger mpc_logger){
    vector<vector<double>> result; //space, speed, function
	vector<vector<double>> one_iter_sample_safe_set;

	double space = 0;
	double speed = 0;

	int sample_safe_set_index = 0;

	while(1){
		// calculate the function
	    vector<vector<double>> mpc_list = DataDrivenLeaderController(space, speed, sample_safe_set, sample_safe_set_index, mpc_logger_filename, mpc_logger);
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

        logger.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end
		if(sample_safe_set[sample_safe_set.size() - 1][0] - space < 0.5)
			break;
	}

	save_result(result, leader_output_dir + "leader_result" + to_string(iter_index) + ".txt");
    
	double tmp = 0;
	for(int i = one_iter_sample_safe_set.size()-1; i>=0 ;i--){
        one_iter_sample_safe_set[i][2] += tmp;
		tmp = one_iter_sample_safe_set[i][2];
	}

	return one_iter_sample_safe_set;
}

int main(){
    auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(logger_filename, 23, 59);
	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    spdlog::sinks_init_list sink_list = { daily_sink, console_sink };
	spdlog::logger logger("console", sink_list.begin(), sink_list.end());
    logger.set_level(spdlog::level::info);

    auto debug_control_logger = spdlog::logger("leader_debug", daily_sink);
    debug_control_logger.set_level(spdlog::level::debug);

	auto mpc_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(mpc_logger_filename);
	auto mpc_logger= spdlog::logger("mpc_log", mpc_sink);
	mpc_logger.set_level(spdlog::level::debug);
	mpc_logger.flush_on(spdlog::level::info);

	// read max speed info
	vector<pair<double, double>> speed_max_info = ReadSpeedMax(dp_input_filename);

    // init sample safe set
    vector<vector<double>> sample_safe_set = InitSampleIteration(speed_max_info, logger, mpc_logger);

    // iteration -> sample safe set
	for(int iter = 1; iter < 2; iter++){
		vector<vector<double>> iter_sample_set = RegularIteration(iter, sample_safe_set, logger, mpc_logger);
        sample_safe_set = MergeSampleSafeSet(sample_safe_set, iter_sample_set);
	}
}
