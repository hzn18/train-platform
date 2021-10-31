#include <iostream>
#include <string>
#include <vector>
#include <climits>
#include <cfloat>
#include "ControlResult.h"
#include "ContextParam.h"
#include <cmath>
#include <fstream>
using namespace std;

//model
//We use the discrete model according to README.md. And we calcuate with F_e because we simulation it in a Straight Line

//params: train model parameters
#define A 9155      //unit: N
#define B 633.6     //unit: Ns/m
#define T_f_C 46.84 //unit: Ns^2/m^2
#define M 160800    //unit: kg

#define a_br  0.92   //unit: m/s^2   braking
#define a_dr  0.9    //unit: m/s^2   driving
#define P_br  220000 //unit: w
#define P_dr  120000 //unit: w
#define j_max 0.98   //unit: m/s^3

//params: simluation parameters
#define delta_s 0.5 //unit: m
#define delta_v 0.01 //unit: m/s

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

//function: control train by dynamic algorithm
TrainControlResult controlTrain(SpeedLimitInfo speedLimitInfo){
    //N1: simluation location dimension state, N2: simulation speed dimension state
    int N1 = speedLimitInfo.get_N1(delta_s);
    int N2 = speedLimitInfo.get_N2(delta_v);
    //initialize
    vector<vector<double>> costStateMatrix(N1, vector<double>(N2, DBL_MAX));  //DBL_MAX means this state is infeasiable
    vector<vector<int>> preStateMatrix(N1, vector<int>(N2, 0));
    int speedLimitIndex = speedLimitInfo.getListLength() - 1;
    double limit =  speedLimitInfo.getLimitSpeed(speedLimitIndex);
    costStateMatrix[N1 - 1][0] = 0;
    //result
    vector<double> limitList(N1, 0);

    //backward -> calculate costStateMatrix
    for(int pos1 = N1 - 2; pos1 >= 0; pos1--){
        for(int pos2 = 0; pos2 < N2; pos2++){
            double now_v = pos2 * delta_v;
            double max_v_next = sqrt( 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v + M * a_dr) + now_v * now_v);  
            double temp = 2*delta_s / M * (-1 * A - B * now_v - T_f_C * now_v * now_v - M * a_br) + now_v * now_v;
            temp = (temp > 0) ? sqrt(temp) : 0;
            double min_v_next = temp;
            int max_pos = floor(max_v_next / delta_v);
            int min_pos = ceil(min_v_next / delta_v);
     //       cout << max_pos << ends;
            //debug
    //        if(pos1 == N1 - 2 && pos2 == 1)
     //           cout << min_pos << "#" << endl;

            //state transition equation
            if(pos2 * delta_v > limit)  //exceed the speed limit 
                break;
            if(costStateMatrix[pos1+1][min_pos] == DBL_MAX) //the next state must exceed the speed limit
                break;
            double q_cost = (limit - pos2 * delta_v) / 100;
                //TODO: 可以实现二分查找
            for(int pos3 = min_pos; pos3 <= max_pos && pos3 < N2; pos3++){
                if(costStateMatrix[pos1+1][pos3] == DBL_MAX)
                    break;
                if(costStateMatrix[pos1+1][pos3] < costStateMatrix[pos1][pos2])
                {
                    preStateMatrix[pos1][pos2] = pos3;
                    costStateMatrix[pos1][pos2] = costStateMatrix[pos1+1][pos3];
                }
            }
        //    cout << preStateMatrix[pos1][pos2] << ends;
            if(costStateMatrix[pos1][pos2] != DBL_MAX)
                costStateMatrix[pos1][pos2] += q_cost;
        }
       // cout << limit << ends;

        //update speed limit
        limitList[pos1] = limit;
        if(speedLimitIndex > 0 && speedLimitInfo.getLimitLocation(speedLimitIndex - 1) >= pos1 * delta_s){
            limit = speedLimitInfo.getLimitSpeed(--speedLimitIndex);
        }
    }
    //forward -> get optimal solution
    vector<double> spaceList;
    vector<double> speedList;
    vector<double> forceList;
    double cost = costStateMatrix[0][0];
    spaceList.push_back(0.0);
    speedList.push_back(0.0);
    forceList.push_back(0.0);
    int index = 0;
//    for(int pos2 = 0; pos2 < N2;pos2++){
//        if(costStateMatrix[0][pos2] < cost){
//            cost = costStateMatrix[0][pos2];
//            index = pos2;
//        }
//   }
    for(int pos1 = 1; pos1 < N1; pos1++){
        spaceList.push_back(pos1 * delta_s);
        speedList.push_back(index * delta_v);
        forceList.push_back(0.0);
        index = preStateMatrix[pos1][index];
    }
    TrainControlResult trainControlResult(spaceList, limitList, speedList, forceList);
    return trainControlResult;
}

//function: 
TrainControlResult concatControlResult(vector<TrainControlResult> controlResultList){
    TrainControlResult concatControlResult;
    //TODO:

    return concatControlResult;
}


//int main(){
//    string speedLimitFile = "./data/SpeedLimit.txt";
//    auto speedLimitInfoList = readSpeedLimit(speedLimitFile);
//    vector<TrainControlResult> controlResultList;
//    for(auto speedLimitInfo : speedLimitInfoList){
//        controlResultList.push_back(controlTrain(speedLimitInfo))
//    }
//    TrainControlResult controlResult = concatControlResult(controlResultList)
//    return 0;
//}