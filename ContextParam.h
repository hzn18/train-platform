#include <iostream>
#include <vector>
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
