// MPC example

#include <string>
#include <math.h>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "Constant.h"
#include "ReadMaxSpeed.h"
#include "MPC.h"

using namespace std;

vector<float> dynamicModel(float function, float space, float speed){
	float a = (function - A - B*speed  - T_f_C*speed*speed) / M;
    float v = speed + a * Ts;
	float s = space + speed * Ts + 0.5 * a * Ts * Ts;
	return vector<float>({s, v});
}

void LeaderControl(){
    auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>("./log/log.txt", 23, 59);
	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    spdlog::sinks_init_list sink_list = { daily_sink, console_sink };
	spdlog::logger logger("console", sink_list.begin(), sink_list.end());
    logger.set_level(spdlog::level::info);

    auto debug_control_logger = std::make_shared<spdlog::logger>("leader_debug", daily_sink);
	auto debug_mpc_logger = std::make_shared<spdlog::logger>("mpc_debug", daily_sink);

    debug_control_logger->set_level(spdlog::level::debug);
	debug_mpc_logger->set_level(spdlog::level::debug);

	// read max speed info
	vector<pair<float, float>> speedMaxInfo = readSpeedMax("./result/DPResult.txt");
 
    vector<vector<float>> result; //space, speed, function

	float space = 0;
	float speed = 0;

	int index = 0;

//	while(space < speedMaxInfo[1].first){
	
	for(int k = 0; k < 1000;k++){
		logger.info("{} iteration:", k);

		double max_delta_space =  v_max * Ts * Np;

        int delta_index = ceil(max_delta_space / delta_s);

        double v_max_temp = 0;

        for(int i = 0; i < delta_index; i++){
            if(v_max_temp < speedMaxInfo[index + i].second){
                v_max_temp = speedMaxInfo[index + i].second;
			}
		}

		max_delta_space = v_max_temp * Ts * Np;

		delta_index = ceil(max_delta_space / delta_s);

        vector<pair<float, float>> speedMaxInfoPart;

		for(int i = 0; i <= delta_index;i++){
			speedMaxInfoPart.push_back(make_pair(speedMaxInfo[i + index].first, speedMaxInfo[i + index].second));
		}

        // calculate the function
	    float function = MPCCaculate(space, speed, speedMaxInfoPart);

        // control the train 
		vector<float> state = dynamicModel(function, space, speed);
        
		space = state[0];

		speed = state[1];

        index = floor(space / delta_s);

        logger.info("s: {}, v:{}, f:{}", space, speed, function);
	}

}


