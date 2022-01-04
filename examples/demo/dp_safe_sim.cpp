/*
 * @Author: houzhinan 
 * @Date: 2022-01-02 18:29:20 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 21:43:31
 */

#include <iostream>
#include <string>
#include <vector>
#include <climits>
#include <cfloat>
#include <cmath>
#include <fstream>

#include "constant.h"
#include "filename.h"
#include "read_speed_limit.h"
#include "dp.h"

using namespace std;

//function: control train by dynamic algorithm
int main(){
    // input data
    vector<pair<double, double>> speed_limit_info = ReadSpeedLimit(DB_FILENAME);
    // dp algorithm
    vector<vector<double>> state_list = dp_safe(speed_limit_info); 

    // saving result
    ofstream fout(DP_SAFE_OUTPUT_FILENAME);
    for(auto train_state : state_list){
        fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << endl;
    }

    return 0;
}
