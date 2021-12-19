/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:02:27 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 21:01:29
 */
#include "read_speed_max.h"

#include <iostream>
#include <fstream>

using namespace std;

vector<pair<double, double>> ReadSpeedMax(string speed_max_file){
    ifstream fin;
    fin.open(speed_max_file, ios::in);
    string buf1 = {0};
    string buf2 = {0};
    string buf3 = {0};
    vector<pair<double, double>> speed_max_info;
    while(fin >> buf1 && fin >> buf2 && fin >> buf3){
        speed_max_info.push_back(make_pair(atof(buf1.c_str()), atof(buf2.c_str())));
    }
    return speed_max_info;
}


