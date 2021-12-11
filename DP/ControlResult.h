#include <vector>
#include <fstream>
#include <iostream>
using namespace std;


class TrainState{
public:
    double space;  // s_i
    double limit;  // v_i_max: train limit speed at s_i location
    double speed;  // v_i: train speed at s_i location
    double force;  // u_i: train force at s_i location
    TrainState(double space, double limit, double speed, double force){
        this->space = space;
        this->limit = limit;
        this->speed = speed;
        this->force = force;
    }
};

class TrainControlResult{
public:
    vector<TrainState> trainStateList;
    TrainControlResult(){
        ;
    }
    TrainControlResult(vector<double> spaceList, vector<double> limitList, vector<double> speedList, vector<double> forceList){
        for(int i = 0; i< spaceList.size(); i++){
             trainStateList.push_back(TrainState(spaceList[i], limitList[i], speedList[i], forceList[i]));
        }
    }
    void showControlResult(){
        for(auto trainState : trainStateList){
            cout << trainState.space << "  " << trainState.limit << " " << trainState.speed << "  " << trainState.force << ends << ends;
        }
    }
    void saveControlResult(string filename){
        ofstream fout(filename);
        for(auto trainState : trainStateList){
            //fout << trainState.space << " " << trainState.limit << " " << trainState.speed << " " << trainState.force << endl;
            fout << trainState.space << " " << trainState.speed << endl;
        }
    }
};



