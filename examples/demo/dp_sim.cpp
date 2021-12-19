#include <iostream>
#include <string>
#include <vector>
#include <climits>
#include <cfloat>
#include <cmath>
#include <fstream>

#include "constant.h"
#include "read_speed_limit.h"

using namespace std;

string db_filename = "./db/SpeedLimit.txt";
string dp_output_filename = "./db/dp_result.txt";

int get_N1(vector<pair<double, double>>& speed_limit_info){
    return int(speed_limit_info[speed_limit_info.size() - 1].first / delta_s) + 1;
}
    
int get_N2(vector<pair<double, double>>& speed_limit_info){
    int limit = 0;
    for(int i = 1; i < speed_limit_info.size(); i++)
        limit = (limit > speed_limit_info[i].second) ? limit : speed_limit_info[i].second;
    return int(limit / delta_v) + 1;
}

//function: control train by dynamic algorithm
int main(){
    // input data
    vector<pair<double, double>> speed_limit_info = ReadSpeedLimit(db_filename);

    //N1: simluation location dimension state, N2: simulation speed dimension state
    int N1 = get_N1(speed_limit_info);
    int N2 = get_N2(speed_limit_info);

    //initialize
    vector<vector<double>> cost_state_matrix(N1, vector<double>(N2, DBL_MAX));  //DBL_MAX means this state is infeasiable
    vector<vector<int>> pre_state_matrix(N1, vector<int>(N2, 0));
    int speed_limit_info_index = speed_limit_info.size() - 1;
    double speed_limit =  speed_limit_info[speed_limit_info_index].second;
    cost_state_matrix[N1 - 1][0] = 0;
    
    //result
    vector<double> limit_list(N1, 0);

    //backward -> calculate costStateMatrix
    for(int pos1 = N1 - 2; pos1 >= 0; pos1--){
        for(int pos2 = 0; pos2 < N2; pos2++){
            double now_v = pos2 * delta_v;
            double max_v_next = sqrt( 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v + M * a_dr) + now_v * now_v);  
            double temp = 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v - M * a_br) + now_v * now_v;
            temp = (temp > 0) ? sqrt(temp) : 0;
            double min_v_next = temp;
            //Now train is at pos2*delta_s. When it running at max driving force, speed of train will become max_v_next at (pos2+1)*delta_s.  
            int max_pos = floor(max_v_next / delta_v);
            int min_pos = ceil(min_v_next / delta_v);  

            //state transition equation
            if(pos2 * delta_v > speed_limit)  //exceed the speed limit 
                break;
            if(cost_state_matrix[pos1+1][min_pos] == DBL_MAX) //the next state must exceed the speed limit
                break;
            double q_cost = (speed_limit - pos2 * delta_v) / 100;
            for(int pos3 = min_pos; pos3 <= max_pos && pos3 < N2; pos3++){
                if(cost_state_matrix[pos1+1][pos3] == DBL_MAX)
                    break;
                if(cost_state_matrix[pos1+1][pos3] < cost_state_matrix[pos1][pos2])
                {
                    pre_state_matrix[pos1][pos2] = pos3;
                    cost_state_matrix[pos1][pos2] = cost_state_matrix[pos1+1][pos3];
                }
            }
            if(cost_state_matrix[pos1][pos2] != DBL_MAX)
                cost_state_matrix[pos1][pos2] += q_cost;
        }

        //update speed limit
        limit_list[pos1] = speed_limit;

        if(speed_limit_info_index > 0 && speed_limit_info[speed_limit_info_index - 1].first >= pos1 * delta_s){
            speed_limit = speed_limit_info[--speed_limit_info_index].second;
        }
    }
    //forward -> get optimal solution
    vector<vector<double>> state_list;

    double cost = cost_state_matrix[0][0];
    state_list.push_back(vector<double>({0.0, 0.0, 0.0}));

    int tmp = 0;
    for(int pos1 = 1; pos1 < N1; pos1++){
        state_list.push_back(vector<double>({pos1 * delta_s, tmp * delta_v, limit_list[pos1]}));
        tmp = pre_state_matrix[pos1][tmp];
    }

    // saving result
    ofstream fout(dp_output_filename);
    for(auto train_state : state_list){
        fout << train_state[0] << " " << train_state[1] << " " << train_state[2] << endl;
    }

    return 0;
}


