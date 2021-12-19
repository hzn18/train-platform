#include <iostream>
#include <vector>
#include <fstream>
#include <climits>

using namespace std;

class SpeedLimitInfo{
public:
    vector<pair<double, double>> speedLimitList; 
    int get_N1(double delta_s){
        return int(speedLimitList[speedLimitList.size() - 1].first / delta_s) + 1;
    }
    int get_N2(double delta_v){
        int limit = 0;
        for(int i = 1; i < speedLimitList.size(); i++)
            limit = (limit > speedLimitList[i].second) ? limit : speedLimitList[i].second;
        return int(limit / delta_v) + 1;
    }
    int getListLength(){
        return speedLimitList.size();
    }
    double getLimitLocation(int index){
        return speedLimitList[index].first;
    }
    double getLimitSpeed(int index){
        return speedLimitList[index].second;
    }
    void insertLimitInfo(double location, double speed){
        speedLimitList.push_back(make_pair(location, speed));
        return ;
    }
    void clearInfo(){
        speedLimitList.clear();
    }
    void showInfo(){
        for(auto limitSpeed : speedLimitList){
            cout << limitSpeed.first << "  " << limitSpeed.second << endl;
        }
    }
};

class ContextInfo{
public:
    SpeedLimitInfo speedLimitInfo;
};

//function: transform speed limit txt file into vector variables
vector<SpeedLimitInfo> readSpeedLimit(string speedLimitFile){
    vector<SpeedLimitInfo> speedLimitInfoList;
    //TODO:
    ifstream fin;
    fin.open(speedLimitFile, ios::in);
    string buf1 = {0};
    string buf2 = {0};
    SpeedLimitInfo speedLimitInfo;
    while(fin >> buf1 && fin >> buf2){
        if(buf2 == "0")
        {
            speedLimitInfoList.push_back(speedLimitInfo);
            speedLimitInfo.clearInfo();
        }
        else
            speedLimitInfo.insertLimitInfo(atof(buf1.c_str()), atof(buf2.c_str()));
    }
    return speedLimitInfoList;
}
