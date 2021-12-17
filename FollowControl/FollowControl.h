#include <string>
#include <math.h>
#include <fstream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "../Constant.h"
#include "../utils/ReadMaxSpeed.h"
#include "../Optimizer/LeaderMPC.h"
#include "../Optimizer/FollowMPC.h"

using namespace std;

string logger_filename = "../log/follow_log.txt";
string mpc_logger_filename = "../log/follow_mpc_log.txt";
string dp_input_filename = "../result/DPResult.txt";
string follow_output_filename = "../result/FollowResult.txt";

vector<float> dynamicModel(float function, float space, float speed){
	float a = (function - A - B*speed  - T_f_C*speed*speed) / M;
    float v = speed + a * Ts;
	float s = space + speed * Ts + 0.5 * a * Ts * Ts;
	return vector<float>({s, v});
}

void FollowControl(){
    // logger manage
    auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(logger_filename, 23, 59);
	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    spdlog::sinks_init_list sink_list = { daily_sink, console_sink };
	spdlog::logger logger("console", sink_list.begin(), sink_list.end());
    logger.set_level(spdlog::level::info);

    auto debug_control_logger = spdlog::logger("debug", daily_sink);
    debug_control_logger.set_level(spdlog::level::debug);

	auto mpc_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(mpc_logger_filename);
	auto mpc_logger= spdlog::logger("mpc_log", mpc_sink);
	mpc_logger.set_level(spdlog::level::debug);
	mpc_logger.flush_on(spdlog::level::info);

	// read max speed info
	vector<pair<float, float>> speedMaxInfo = readSpeedMax(dp_input_filename);
 
    vector<vector<vector<float>>> result; 
    // 1-dimension: train id
    // 2-dimension: time
    // 3-dimension: train state such as space, speed, function


    // temp variable
	vector<float> space_tmp(train_num, 0);
	vector<float> speed_tmp(train_num, 0);

	vector<int> index(train_num, 0);

    vector<int> isRun(train_num, 0); 

    int last_distance = 0;   //尾车的位置，发车动作需要维护该信息

	while(1){
        // 发车
        if(isRun[0] == 0){
            isRun[1] = 1;
        }
        else{
            for(int i = 0; i < train_num; i++){
                if(isRun[i] == 0 && last_distance > d_des){
                    isRun[i] = 1; 
                    break;
                }
            }
        }

        // 控制环节

        // 前车控制
        if(1){           
            double max_delta_space =  v_max * Ts * Np;

            int delta_index = ceil(max_delta_space / delta_s);

            if(delta_index + index[0] > speedMaxInfo.size()-1)
		        delta_index = speedMaxInfo.size() - 1 - index[0];

            double v_max_temp = 0;

            for(int i = 0; i < delta_index; i++){
                if(v_max_temp < speedMaxInfo[index[0] + i].second){
                    v_max_temp = speedMaxInfo[index[0] + i].second;
			    }
		    }

		    max_delta_space = v_max_temp * Ts * Np;

		    delta_index = ceil(max_delta_space / delta_s);

		    if(delta_index + index[0] > speedMaxInfo.size()-1)
		        delta_index = speedMaxInfo.size() - 1 - index[0];

            vector<pair<float, float>> speedMaxInfoPart;

		    for(int i = 0; i <= delta_index;i++){
			    speedMaxInfoPart.push_back(make_pair(speedMaxInfo[i + index[0]].first, speedMaxInfo[i + index[0]].second));
		        debug_control_logger.debug("space = {}, max = {}", speedMaxInfo[i+index[0]].first, speedMaxInfo[i+index[0]].second);
		    }

            // calculate the function
	        vector<vector<float>> mpc_list = LeaderMPCCaculate(space, speed, speedMaxInfoPart, mpc_logger_filename, mpc_logger);
         
		    float function = mpc_list[2][0];
		
            // control the train 
		    vector<float> state = dynamicModel(function, space_tmp[0], speed_tmp[0]);
            space_tmp[0] = state[0];
		    speed_tmp[0] = state[1];
            index[0] = floor(space_tmp[0] / delta_s);

		    result[0].push_back(vector<float>{space_tmp[0], speed_tmp[0], function});

            logger.info("id: {}, s: {}, v:{}, f:{}", 0, space_tmp[0], speed_tmp[0], function);
        }

        // 后车控制
        // 移动距离预估
        for(int i = 0; i < train_num; i++)
        {
            if(isRun[i] == 0)   //这辆车还没有出发
                break;
            
            double max_delta_space =  v_max * Ts * Np;

            int delta_index = ceil(max_delta_space / delta_s);

            if(delta_index + index[i] > speedMaxInfo.size()-1)
		        delta_index = speedMaxInfo.size() - 1 - index[i];

            double v_max_temp = 0;

            for(int i = 0; i < delta_index; i++){
                if(v_max_temp < speedMaxInfo[index[i] + i].second){
                    v_max_temp = speedMaxInfo[index[i] + i].second;
			    }
		    }

		    max_delta_space = v_max_temp * Ts * Np;

		    delta_index = ceil(max_delta_space / delta_s);

		    if(delta_index + index[i] > speedMaxInfo.size()-1)
		        delta_index = speedMaxInfo.size() - 1 - index[i];

            vector<pair<float, float>> speedMaxInfoPart;

		    for(int i = 0; i <= delta_index;i++){
			    speedMaxInfoPart.push_back(make_pair(speedMaxInfo[i + index[i]].first, speedMaxInfo[i + index[i]].second));
		        debug_control_logger.debug("space = {}, max = {}", speedMaxInfo[i+index[i]].first, speedMaxInfo[i+index[i]].second);
		    }

            // calculate the function
	        vector<vector<float>> mpc_list = FollowMPCCaculate(space, speed, speedMaxInfoPart, leader_info, mpc_logger_filename, mpc_logger);
         
		    float function = mpc_list[2][0];
		
            // control the train 
		    vector<float> state = dynamicModel(function, space_tmp[i], speed_tmp[i]);
            space_tmp[i] = state[0];
		    speed_tmp[i] = state[1];
            index[i] = floor(space_tmp[i] / delta_s);

		    result[i].push_back(vector<float>{space_tmp[i], speed_tmp[i], function});

            logger.info("id: {}, s: {}, v:{}, f:{}", 0, space_tmp[i], speed_tmp[i], function);
        }
		
    }
        // simulation end

/*
		if(speedMaxInfo[speedMaxInfo.size() - 1].first - space < 1 )
			break;
	}
    
	ofstream fout(leader_output_filename);
    for(auto trainState : result){
        fout << trainState[0] << " " << trainState[1] << " " << trainState[2] << endl;
    }
*/
}


