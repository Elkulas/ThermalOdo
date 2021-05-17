#include "thermalodo.h"

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

#include "ReadData.h"


int main( int argc, char** argv)
{
    // 设置读取的情况
    vector<pair<double, Mat>> vThermalImg;
    string sThermalDataPath = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/";
    string sThermalConfigPath = "/media/jjj/shuaibi/NGC_data/desk_xyz/";

    GetImageDataWithNum(vThermalImg, sThermalDataPath, sThermalConfigPath, 5);
    
    return 0;
}
