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

enum Predictor {SH, NP, MB};

using namespace std;

string logger_filename = "../log/follow_log.txt";
string mpc_logger_filename = "../log/follow_mpc_log.txt";
string dp_input_filename = "../result/DPResult.txt";
string follow_output_dir = "../result/";

Predictor predictor_method = MB;

vector<double> dynamicModel(double function, double space, double speed){
	double a = (function - A - B*speed  - T_f_C*speed*speed) / M;
    double v = speed + a * Ts;
	double s = space + speed * Ts + 0.5 * a * Ts * Ts;
	return vector<double>({s, v});
}

vector<vector<double>> NPPredictor(double space, double speed){
    vector<vector<double>> result;

    double function = -M * a_br;
    double v = speed;
	double s = space;
	for(int i = 0; i < Np; i++){
        double a = (function - A - B * v  - T_f_C * v * v) / M;
        v += a * Ts;
	    s += v * Ts + 0.5 * a * Ts * Ts;
		result.push_back(vector<double>({s, v}));
	}

    return result;
}

vector<vector<double>> MBPredictor(double space){
    vector<vector<double>> result;
    for(int i = 0; i < Np; i++){
		result.push_back(vector<double>({space, 0.001}));
	}

    return result;
}

void FollowControl(){
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
	vector<pair<double, double>> speedMaxInfo = readSpeedMax(dp_input_filename);
 
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

	vector<int> index(train_num, 0);

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
        vector<vector<double>> predictor;

        // 前车控制
        if(!isArrived[0]){           
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

            vector<pair<double, double>> speedMaxInfoPart;

		    for(int i = 0; i <= delta_index;i++){
			    speedMaxInfoPart.push_back(make_pair(speedMaxInfo[i + index[0]].first, speedMaxInfo[i + index[0]].second));
		        debug_logger.debug("space = {}, max = {}", speedMaxInfo[i+index[0]].first, speedMaxInfo[i+index[0]].second);
		    }

            // calculate the function
	        vector<vector<double>> mpc_list = LeaderMPCCaculate(space_tmp[0], speed_tmp[0], speedMaxInfoPart, mpc_logger_filename, mpc_logger);

            // predictor
            switch(predictor_method){
                case SH:
                    predictor = mpc_list;
                    break;
                case NP:
                    predictor = NPPredictor(space_tmp[0], speed_tmp[0]);
                    break;
                case MB:
                    predictor = MBPredictor(space_tmp[0]);
                    break;
            }

		    double function = mpc_list[0][2];

            // control the train 
		    vector<double> state = dynamicModel(function, space_tmp[0], speed_tmp[0]);
            space_tmp[0] = state[0];
		    speed_tmp[0] = state[1];
            index[0] = floor(space_tmp[0] / delta_s);

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

            double max_delta_space =  v_max * Ts * Np;

            int delta_index = ceil(max_delta_space / delta_s);

            if(delta_index + index[train_id] > speedMaxInfo.size()-1)
		        delta_index = speedMaxInfo.size() - 1 - index[train_id];

            double v_max_temp = 0;

            for(int i = 0; i < delta_index; i++){
                if(v_max_temp < speedMaxInfo[index[train_id] + i].second){
                    v_max_temp = speedMaxInfo[index[train_id] + i].second;
			    }
		    }

		    max_delta_space = v_max_temp * Ts * Np;

		    delta_index = ceil(max_delta_space / delta_s);

		    if(delta_index + index[train_id] > speedMaxInfo.size()-1)
		        delta_index = speedMaxInfo.size() - 1 - index[train_id];

            vector<pair<double, double>> speedMaxInfoPart;

		    for(int i = 0; i <= delta_index;i++){
			    speedMaxInfoPart.push_back(make_pair(speedMaxInfo[i + index[train_id]].first, speedMaxInfo[i + index[train_id]].second));
		        debug_logger.debug("id = {}, space = {}, max = {}", train_id, speedMaxInfo[i+index[train_id]].first, speedMaxInfo[i+index[train_id]].second);
		    }

            // calculate the function
            vector<vector<double>> mpc_list;
            if(!isArrived[train_id - 1]){
                debug_logger.debug("id = {}, space = {}, speed = {}", train_id, space_tmp[train_id], speed_tmp[train_id]);
                for(int i = 0; i < predictor.size();i++){
                    debug_logger.debug("leader {} step: space = {}, speed = {}, function = {}", i, predictor[i][0], predictor[i][1], predictor[i][2]);
                }
	            mpc_list = FollowMPCCaculate(space_tmp[train_id], speed_tmp[train_id], speedMaxInfoPart, predictor, mpc_logger_filename, mpc_logger);
            }
            else 
                mpc_list = LeaderMPCCaculate(space_tmp[train_id], speed_tmp[train_id], speedMaxInfoPart, mpc_logger_filename, mpc_logger);
            
            // predictor
            switch(predictor_method){
                case SH:
                    predictor = mpc_list;
                    break;
                case NP:
                    predictor = NPPredictor(space_tmp[train_id], speed_tmp[train_id]);
                    break;
                case MB:
                    predictor = MBPredictor(space_tmp[train_id]);
                    break;
            }

		    double function = mpc_list[0][2];
		
            // control the train 
		    vector<double> state = dynamicModel(function, space_tmp[train_id], speed_tmp[train_id]);
            space_tmp[train_id] = state[0];
		    speed_tmp[train_id] = state[1];
            index[train_id] = floor(space_tmp[train_id] / delta_s);

		    result[train_id].push_back(vector<double>{space_tmp[train_id], speed_tmp[train_id], function, simulation_time});

            logger.info("id: {}, s: {}, v:{}, f:{}", train_id, space_tmp[train_id], speed_tmp[train_id], function);
            last_distance = space_tmp[train_id];
        }
		
        for(int i = 0; i < train_num; i++)
            if(speedMaxInfo[speedMaxInfo.size() - 1].first - space_tmp[i] < 1 )
			    isArrived[i] = true;
        
        if(isArrived[train_num - 1])
            break;
        
        simulation_time += Ts;
    }
    
    for(int train_id = 0; train_id < train_num; train_id++){
        string filename = follow_output_dir + "FollowResult_" + to_string(train_id) + ".txt";
        ofstream fout(filename);
        logger.info("{} result is saved into {}", train_id, filename);
        for(auto trainState : result[train_id]){
            fout << trainState[0] << " " << trainState[1] << " " << trainState[2] << " " << trainState[3] << endl;
        }
    }

}


