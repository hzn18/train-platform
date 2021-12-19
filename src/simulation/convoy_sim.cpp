/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:35:50 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 19:18:54
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
#include "follow_controller.h"
#include "dynamic_model.h"

enum Predictor {SH, NP, MB};

using namespace std;

string logger_filename = "./log/follow_log.txt";
string mpc_logger_filename = "./log/follow_mpc_log.txt";
string dp_input_filename = "./result/DPResult.txt";
string follow_output_dir = "./result/";

Predictor predictor_method = MB;


int main(){
    // logger manage
    auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(logger_filename, 23, 59);
	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    spdlog::sinks_init_list sink_list = { daily_sink, console_sink };
	spdlog::logger logger("console", sink_list.begin(), sink_list.end());
    logger.set_level(spdlog::level::info);

    auto debug_logger = spdlog::logger("debug", daily_sink);
    debug_logger.set_level(spdlog::level::info);

	auto mpc_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(mpc_logger_filename);
	auto mpc_logger= spdlog::logger("mpc_log", mpc_sink);
	mpc_logger.set_level(spdlog::level::info);
	mpc_logger.flush_on(spdlog::level::info);

	// read max speed info
	vector<pair<double, double>> speed_max_info = ReadSpeedMax(dp_input_filename);
 
    vector<vector<vector<double>>> result; 
    // 1-dimension: train id
    // 2-dimension: time
    // 3-dimension: train state such as space, speed, function, time
    for(int i = 0; i < train_num; i++){
        result.push_back(vector<vector<double>>());
    }


    // temp variable
	vector<double> space_tmp(train_num, 0);
	vector<double> speed_tmp(train_num, 0);

	vector<int> speed_max_info_index(train_num, 0);

    vector<bool> isRun(train_num, false); 

    vector<bool> isArrived(train_num, false);

    double last_distance = 0;   //尾车的位置，发车动作需要维护该信息

    double simulation_time = 0;

	while(1){
        logger.info("simulation time is {}", simulation_time);
        // 发车
        if(isRun[0] == false){
            isRun[0] = true;
            logger.info("train {} starts to run", 0);
        }
        else{
            for(int i = 1; i < train_num; i++){
                if(!isRun[i] && last_distance > d_des){
                    logger.info("train {} starts to run", i);
                    isRun[i] = true; 
                    break;
                }
            }
        }

        // 控制环节
        vector<vector<double>> leader_mpc_list;

        // 前车控制
        if(!isArrived[0]){           
            // calculate the function
	        leader_mpc_list = LeaderController(space_tmp[0], speed_tmp[0], speed_max_info, speed_max_info_index[0], mpc_logger_filename, mpc_logger);

		    double function = leader_mpc_list[0][2];

            // control the train 
		    vector<double> state = DynamicModel(function, space_tmp[0], speed_tmp[0]);
            space_tmp[0] = state[0];
		    speed_tmp[0] = state[1];
            speed_max_info_index[0] = floor(space_tmp[0] / delta_s);

		    result[0].push_back(vector<double>({space_tmp[0], speed_tmp[0], function, simulation_time}));

            logger.info("id: {}, s: {}, v:{}, f:{}", 0, space_tmp[0], speed_tmp[0], function);

            last_distance = space_tmp[0];
        }
        else{
            result[0].push_back(vector<double>({space_tmp[0], 0, 0, simulation_time}));
        }

        // 后车控制
        // 移动距离预估
        for(int train_id = 1; train_id < train_num; train_id++)
        {
            if(!isRun[train_id]){   //这辆车还没有出发
                result[train_id].push_back(vector<double>({0, 0, 0, simulation_time}));
                break;
            }
            if(isArrived[train_id]){
                result[train_id].push_back(vector<double>({space_tmp[train_id], 0, 0, simulation_time}));
                continue;
            }

            // calculate the function
            if(!isArrived[train_id - 1]){
                // predictor
                switch(predictor_method){
                case MB:
                    leader_mpc_list = FollowController(space_tmp[train_id - 1], space_tmp[train_id], speed_tmp[train_id], speed_max_info, speed_max_info_index[train_id], mpc_logger_filename, mpc_logger);
                    break;
                case NP:
                    leader_mpc_list = FollowController(space_tmp[train_id - 1], speed_tmp[train_id - 1], space_tmp[train_id], speed_tmp[train_id], speed_max_info, speed_max_info_index[train_id], mpc_logger_filename, mpc_logger);
                    break;
                case SH:
                    leader_mpc_list = FollowController(leader_mpc_list, space_tmp[train_id], speed_tmp[train_id], speed_max_info, speed_max_info_index[train_id], mpc_logger_filename, mpc_logger);
                    break;
                }
	        }
            else 
                leader_mpc_list = LeaderController(space_tmp[train_id], speed_tmp[train_id], speed_max_info, speed_max_info_index[train_id], mpc_logger_filename, mpc_logger);

		    double function = leader_mpc_list[0][2];
		
            // control the train 
		    vector<double> state = DynamicModel(function, space_tmp[train_id], speed_tmp[train_id]);
            space_tmp[train_id] = state[0];
		    speed_tmp[train_id] = state[1];
            speed_max_info_index[train_id] = floor(space_tmp[train_id] / delta_s);

		    result[train_id].push_back(vector<double>{space_tmp[train_id], speed_tmp[train_id], function, simulation_time});

            logger.info("id: {}, s: {}, v:{}, f:{}", train_id, space_tmp[train_id], speed_tmp[train_id], function);
            last_distance = space_tmp[train_id];
        }
		
        for(int i = 0; i < train_num; i++)
            if(speed_max_info[speed_max_info.size() - 1].first - space_tmp[i] < 1 )
			    isArrived[i] = true;
        
        if(isArrived[train_num - 1])
            break;
        
        simulation_time += Ts;
    }
    
    for(int train_id = 0; train_id < train_num; train_id++){
        string filename = follow_output_dir + "FollowResult_" + to_string(train_id) + ".txt";
        ofstream fout(filename);
        logger.info("{} result is saved into {}", train_id, filename);
        for(auto train_state : result[train_id]){
            fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << " " << train_state[3] << endl;
        }
    }
    return 0;
}