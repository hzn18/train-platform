/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:02:37 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 16:38:11
 */
#include <string>
#include <math.h>
#include <fstream>

#include "logger.h"
#include "constant.h"
#include "filename.h"

#include "read_speed_max.h"
#include "leader_controller.h"
#include "dynamic_model.h"

using namespace std;

int main(){
    //logger manage
	logger_config();

	// read max speed info
	vector<pair<double, double>> speed_max_info = ReadSpeedMax(DP_SAFE_OUTPUT_FILENAME);
 
    vector<vector<double>> result; //space, speed, function

	double space = 0;
	double speed = 0;

	int speed_max_info_index = 0;

	while(1){

        // calculate the function
	    vector<vector<double>> mpc_list = LeaderController(space, speed, speed_max_info, speed_max_info_index);
         
		double function = mpc_list[0][2];
		
        // control the train 
		vector<double> state = DynamicModel(function, space, speed);
        
		space = state[0];

		speed = state[1];

        speed_max_info_index = floor(space / delta_s);

		result.push_back(vector<double>{space, speed, function});

        LOGGER.info("s: {}, v:{}, f:{}", space, speed, function);

        // simulation end

		if(speed_max_info[speed_max_info.size() - 1].first - space < 1 )
			break;
	}
    
	ofstream fout(LEADER_OUTPUT_FILENAME);
    for(auto train_state : result){
        fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << endl;
    }

}
