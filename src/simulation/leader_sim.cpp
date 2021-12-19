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

string logger_filename = "..log/log.txt";
string mpc_logger_filename = "./log/mpc_log.txt";
string dp_input_filename = "./result/DPResult.txt";
string leader_output_filename = "./result/LeaderResult.txt";

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
 
    vector<vector<double>> result; //space, speed, function

	double space = 0;
	double speed = 0;

	int speed_max_info_index = 0;

	while(1){

        // calculate the function
	    vector<vector<double>> mpc_list = LeaderController(space, speed, speed_max_info, speed_max_info_index, mpc_logger_filename, mpc_logger);
         
		double function = mpc_list[0][2];
		
        // control the train 
		vector<double> state = DynamicModel(function, space, speed);
        
		space = state[0];

		speed = state[1];

        speed_max_info_index = floor(space / delta_s);

		result.push_back(vector<double>{space, speed, function});

        logger.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end

		if(speed_max_info[speed_max_info.size() - 1].first - space < 1 )
			break;
	}
    
	ofstream fout(leader_output_filename);
    for(auto train_state : result){
        fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << endl;
    }

}
