/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 20:13:37 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 20:58:02
 */

#include "read_speed_limit.h"

#include <iostream>
#include <fstream>

using namespace std;


//function: transform speed limit txt file into vector variables
vector<pair<double, double>> ReadSpeedLimit(string speed_limit_file){
    ifstream fin;
    fin.open(speed_limit_file, ios::in);
    string buf1 = {0};
    string buf2 = {0};
    vector<pair<double, double>> speed_limit_info;
    while(fin >> buf1 && fin >> buf2){
        speed_limit_info.push_back(make_pair(atof(buf1.c_str()), atof(buf2.c_str())));
    }
    return speed_limit_info;
}
