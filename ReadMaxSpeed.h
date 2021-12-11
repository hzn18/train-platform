#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

vector<pair<float, float>> readSpeedMax(string speedLimitFile){
    ifstream fin;
    fin.open(speedLimitFile, ios::in);
    string buf1 = {0};
    string buf2 = {0};
    vector<pair<float, float>> speedMaxInfo;
    while(fin >> buf1 && fin >> buf2){
        speedMaxInfo.push_back(make_pair(atof(buf1.c_str()), atof(buf2.c_str())));
    }
    return speedMaxInfo;
}