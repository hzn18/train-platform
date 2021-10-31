#include "LeaderDP.cpp"

void testSpeedLimitInfoList(vector<SpeedLimitInfo> speedLimitInfoList){
    for(auto speedLimitInfo : speedLimitInfoList){
        cout << "********************" << endl;
        speedLimitInfo.showInfo();
    }
    cout << "********************" << endl;
}

void testDPAlgorithm(SpeedLimitInfo speedLimitInfo){
    auto controlResult = controlTrain(speedLimitInfo);
    //controlResult.showControlResult();
    controlResult.saveControlResult("./result/DPResult.txt");
}

int main(){
    string speedLimitFile = "./data/SpeedLimit.txt";
    vector<SpeedLimitInfo> speedLimitInfoList = readSpeedLimit(speedLimitFile);
    testSpeedLimitInfoList(speedLimitInfoList);
    testDPAlgorithm(speedLimitInfoList[0]);
}
