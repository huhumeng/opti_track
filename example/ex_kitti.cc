#include "config.h"
#include "parameter.h"
#include "slam_util.h"
#include "feature_tracker.h"

#include <signal.h>

#include <iomanip>

#include <opencv2/opencv.hpp>

using namespace std;

bool stop = false;

void exitHandle(int){
    stop = true;
    Info("Bye Bye\n");
}

int main(int argc, char** argv){

    if(argc != 2){
        Fatal("error argument... Usage ./ex_kitti path_to_config_file");
    }

    Config::setParameterFile(argv[1]);
    readAllParameters();

    FeatureTracker featureTracker;

    string left_image_path  = DATA_ROOT_DIR + "/image_2/";
    string right_image_path = DATA_ROOT_DIR + "/image_3/";

    string times_file_path = DATA_ROOT_DIR + "/times.txt";

    vector<double> timestamps;
    double timestamp;
    ifstream fin(times_file_path);
    if(!fin)
        Fatal("Error file path\n");
    
    while(fin >> timestamp)
    {
        timestamps.push_back(timestamp);
    }
    fin.close();


    cv::Mat image0, image1;

    signal(SIGINT, exitHandle);
    
    int count = 0;
    while(!stop && count < 500){
        
        printf("---------- frame %06d ----------\n", count);

        stringstream ss;
        ss << setfill('0') << setw(6) << count;
        string left_name  = left_image_path + ss.str() + ".png";
        string right_name = right_image_path + ss.str() + ".png";

        image0 = cv::imread(left_name, CV_LOAD_IMAGE_GRAYSCALE);
        image1 = cv::imread(right_name, CV_LOAD_IMAGE_GRAYSCALE);

        if(image0.empty() || image1.empty())
        {
            Warn("No normal image input\n");
            break;
        }
        
        featureTracker.trackImage(timestamps[count], image0, image1);

        count++;
    }
    
    return 0;
}