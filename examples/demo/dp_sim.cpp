/*
 * @Author: houzhinan 
 * @Date: 2022-01-02 18:29:20 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-02 20:02:13
 */

#include <iostream>
#include <string>
#include <vector>
#include <climits>
#include <cfloat>
#include <cmath>
#include <fstream>

#include "constant.h"
#include "read_speed_limit.h"
#include "dp.h"

using namespace std;

string db_filename = "./db/speed_limit.txt";
string dp_safe_output_filename = "./db/dp_safe_result.txt";
string dp_ref_output_filename = "./db/dp_ref_result.txt"; 



//function: control train by dynamic algorithm
int main(){
    // input data
    vector<pair<double, double>> speed_limit_info = ReadSpeedLimit(db_filename);

    // dp algorithm
    // vector<vector<double>> state_list = dp_safe(speed_limit_info); 
    vector<vector<double>> state_list = dp_ref(speed_limit_info); 

    // saving result
    // ofstream fout(dp_safe_output_filename);
    ofstream fout(dp_ref_output_filename);
    for(auto train_state : state_list){
        fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << endl;
    }

    return 0;
}
