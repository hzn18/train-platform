#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

vector<pair<double, double>> readSpeedMax(string speedLimitFile){
    ifstream fin;
    fin.open(speedLimitFile, ios::in);
    string buf1 = {0};
    string buf2 = {0};
    string buf3 = {0};
    vector<pair<double, double>> speedMaxInfo;
    while(fin >> buf1 && fin >> buf2 && fin >> buf3){
        speedMaxInfo.push_back(make_pair(atof(buf1.c_str()), atof(buf2.c_str())));
    }
    return speedMaxInfo;
}