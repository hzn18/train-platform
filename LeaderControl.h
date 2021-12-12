// MPC example

#include <string>
#include <math.h>
#include <fstream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

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

    auto debug_control_logger = spdlog::logger("leader_debug", daily_sink);
    debug_control_logger.set_level(spdlog::level::debug);

	auto mpc_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("./log/mpc_log.txt");
	auto mpc_logger= spdlog::logger("mpc_log", mpc_sink);
	mpc_logger.set_level(spdlog::level::debug);
	mpc_logger.flush_on(spdlog::level::info);


	// read max speed info
	vector<pair<float, float>> speedMaxInfo = readSpeedMax("./result/DPResult.txt");
 
    vector<vector<float>> result; //space, speed, function

	float space = 0;
	float speed = 0;

	int index = 0;

	while(1){

		double max_delta_space =  v_max * Ts * Np;

        int delta_index = ceil(max_delta_space / delta_s);

        if(delta_index + index > speedMaxInfo.size()-1)
		    delta_index = speedMaxInfo.size() - 1 - index;


        double v_max_temp = 0;

        for(int i = 0; i < delta_index; i++){
            if(v_max_temp < speedMaxInfo[index + i].second){
                v_max_temp = speedMaxInfo[index + i].second;
			}
		}

		max_delta_space = v_max_temp * Ts * Np;

		delta_index = ceil(max_delta_space / delta_s);

		if(delta_index + index > speedMaxInfo.size()-1)
		    delta_index = speedMaxInfo.size() - 1 - index;

        vector<pair<float, float>> speedMaxInfoPart;

		for(int i = 0; i <= delta_index;i++){
			speedMaxInfoPart.push_back(make_pair(speedMaxInfo[i + index].first, speedMaxInfo[i + index].second));

		    debug_control_logger.debug("space = {}, max = {}", speedMaxInfo[i+index].first, speedMaxInfo[i+index].second);
		}

        // calculate the function
	    float function = MPCCaculate(space, speed, speedMaxInfoPart, mpc_logger);
         
        // control the train 
		vector<float> state = dynamicModel(function, space, speed);
        
		space = state[0];

		speed = state[1];

        index = floor(space / delta_s);

		result.push_back(vector<float>{space, speed, function});

        logger.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end

		if(speedMaxInfo[speedMaxInfo.size() - 1].first - space < 1 )
			break;
	}
    
	ofstream fout("./result/LeaderResult.txt");
    for(auto trainState : result){
        fout << trainState[0] << " " << trainState[1] << " " << trainState[2] << endl;
    }

}


