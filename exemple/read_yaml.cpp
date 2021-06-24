#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

using namespace std::chrono;

using namespace std;

int main()
{
    cv::FileStorage fsSettings("../data/yaml/param.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    std::string phrase;
    fsSettings["Param.phrase"] >> phrase;
    std::cout << phrase;
}