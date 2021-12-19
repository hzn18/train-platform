#include "read_speed_max.h"

#include <iostream>
#include <fstream>

using namespace std;

vector<pair<double, double>> ReadSpeedMax(string speed_limit_file){
    ifstream fin;
    fin.open(speed_limit_file, ios::in);
    string buf1 = {0};
    string buf2 = {0};
    string buf3 = {0};
    vector<pair<double, double>> speed_max_info;
    while(fin >> buf1 && fin >> buf2 && fin >> buf3){
        speed_max_info.push_back(make_pair(atof(buf1.c_str()), atof(buf2.c_str())));
    }
    return speed_max_info;
}
