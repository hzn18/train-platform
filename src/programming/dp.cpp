#include "dp.h"
#include <iostream>
#include <string>
#include <climits>
#include <cfloat>
#include <cmath>
#include <fstream>

#include "constant.h"

using namespace std;

int get_N1(vector<pair<double, double>>& speed_limit_info){
    return int(speed_limit_info[speed_limit_info.size() - 1].first / delta_s) + 1;
}
    
int get_N2(vector<pair<double, double>>& speed_limit_info){
    int limit = 0;
    for(int i = 1; i < speed_limit_info.size(); i++)
        limit = (limit > speed_limit_info[i].second) ? limit : speed_limit_info[i].second;
    return int(limit / delta_v) + 1;
}

// only consider speed.
double safe_cost(double speed_limit, double v){
    return speed_limit - v;
}

// consider speed and energy.
double ref_cost(double v, double u){
    double u_max = M * a_dr;
    return K_dp_v * (1 - v / v_max) * (1 - v / v_max) + K_dp_u * u / u_max * u / u_max; 
}

vector<vector<double>> dp_safe(vector<pair<double, double>> speed_limit_info){
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
            
            //infeasible solution: this state exceed the speed limit
            if(pos2 * delta_v > speed_limit)   
                break;

            // now_v -> [min_v_next, max_v_next]:
            // Supposing train is at "pos2",
            // if this ran at max driving force, speed of train will become max_v_next at "pos2+1".  
            // if this ran at min braking force, speed of train will become min_v_next at "pos2+1".

            double now_v = pos2 * delta_v;
            double max_v_next = sqrt( 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v + M * a_dr) + now_v * now_v);  
            double temp = 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v - M * a_br) + now_v * now_v;
            temp = (temp > 0) ? sqrt(temp) : 0;
            double min_v_next = temp;
                        
            int max_pos = floor(max_v_next / delta_v);
            int min_pos = ceil(min_v_next / delta_v);  

            // state transition equation
            // infeasible solution: the next state must exceed the speed limit
            if(cost_state_matrix[pos1+1][min_pos] == DBL_MAX) 
                break;
            
            // cost of this state: we design a function for this. 
            double q_cost = safe_cost(speed_limit, now_v);
            for(int pos3 = min_pos; pos3 <= max_pos && pos3 < N2; pos3++){
                if(cost_state_matrix[pos1+1][pos3] == DBL_MAX)
                    break;
                if(cost_state_matrix[pos1+1][pos3] < cost_state_matrix[pos1][pos2])
                {
                    pre_state_matrix[pos1][pos2] = pos3;
                    cost_state_matrix[pos1][pos2] = cost_state_matrix[pos1+1][pos3];
                }
            }

            // cost may be DBL_MAX, this means this train will exceed the speed limit at next state whatever you drive it.
            if(cost_state_matrix[pos1][pos2] != DBL_MAX)  
                cost_state_matrix[pos1][pos2] += q_cost;
        }

        //update speed limit
        limit_list[pos1] = speed_limit;

        if(speed_limit_info_index > 0 && speed_limit_info[speed_limit_info_index - 1].first >= pos1 * delta_s){
            speed_limit = speed_limit_info[--speed_limit_info_index].second;
        }
    }
    
    // forward -> get optimal solution
    // state_list -> 3-dimension array. these dimension separately means distance, programming_speed, limit 
    vector<vector<double>> state_list; 

    double cost = cost_state_matrix[0][1]; 

    int tmp = 1; // a trick: we get a non-zero starting speed
    for(int pos1 = 0; pos1 < N1; pos1++){
        state_list.push_back(vector<double>({pos1 * delta_s, tmp * delta_v, limit_list[pos1]}));
        tmp = pre_state_matrix[pos1][tmp];
    }

    // some info:
    cout << "the cost of safe programming curve: " << cost << endl;

    return state_list;
}

vector<vector<double>> dp_ref(vector<pair<double, double>> speed_limit_info){
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
            
            //infeasible solution: this state exceed the speed limit
            if(pos2 * delta_v > speed_limit)   
                break;

            // now_v -> [min_v_next, max_v_next]:
            // Supposing train is at "pos2",
            // if this ran at max driving force, speed of train will become max_v_next at "pos2+1".  
            // if this ran at min braking force, speed of train will become min_v_next at "pos2+1".

            double now_v = pos2 * delta_v;
            double max_v_next = sqrt( 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v + M * a_dr) + now_v * now_v);  
            double temp = 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v - M * a_br) + now_v * now_v;
            temp = (temp > 0) ? sqrt(temp) : 0;
            double min_v_next = temp;
                        
            int max_pos = floor(max_v_next / delta_v);
            int min_pos = ceil(min_v_next / delta_v);  

            // state transition equation
            // infeasible solution: the next state must exceed the speed limit
            if(cost_state_matrix[pos1+1][min_pos] == DBL_MAX) 
                break;
            
            for(int pos3 = min_pos; pos3 <= max_pos && pos3 < N2; pos3++){
                if(cost_state_matrix[pos1+1][pos3] == DBL_MAX)
                    break;
                
                // calculate the function which transpose from now_v to next_v
                double next_v = pos3 * delta_v;
                double f = A + B * now_v + T_f_C * now_v * now_v + 0.5 * M * (next_v * next_v - now_v * now_v) / delta_s; 

                double q_cost = ref_cost(now_v, f);
                if(cost_state_matrix[pos1+1][pos3] + q_cost < cost_state_matrix[pos1][pos2])
                {
                    pre_state_matrix[pos1][pos2] = pos3;
                    cost_state_matrix[pos1][pos2] = cost_state_matrix[pos1+1][pos3] + q_cost;
                }
            }
        }

        //update speed limit
        limit_list[pos1] = speed_limit;

        if(speed_limit_info_index > 0 && speed_limit_info[speed_limit_info_index - 1].first >= pos1 * delta_s){
            speed_limit = speed_limit_info[--speed_limit_info_index].second;
        }
    }
    
    // forward -> get optimal solution
    // state_list -> 3-dimension array. these dimension separately means distance, programming_speed, limit 
    vector<vector<double>> state_list; 

    double cost = cost_state_matrix[0][1]; 

    int tmp = 1; // a trick: we get a non-zero starting speed
    for(int pos1 = 0; pos1 < N1; pos1++){
        state_list.push_back(vector<double>({pos1 * delta_s, tmp * delta_v, limit_list[pos1]}));
        tmp = pre_state_matrix[pos1][tmp];
    }

    // some info:
    cout << "the cost of ref programming curve: " << cost << endl;

    return state_list;
}
