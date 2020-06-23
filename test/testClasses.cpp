//
// Created by abhishekh.baskaran on 21/03/2019.
//

#include "testClasses.h"


int TestInterface::TestClasses::testGetDiffImage(const cv::Mat& currentImage,const cv::Mat& backgroundImage) {
    cv::Mat diffImage;
    MotionDetectionInterface::MotionDetection::getDiffImage(currentImage,backgroundImage,diffImage);
    bool equal = cv::countNonZero(diffImage) == 0;
    if(equal){
        return 1;
    } else{
        return -1;
    }
}

int TestInterface::TestClasses::testCreateForegroundMask() {
    cv::Mat inputImage=cv::imread("../exp/testBenchForTestFunctions/inputToTestCreateForegroundMask.jpg");
    cv::Mat foregroundMask;
    cvtColor(inputImage, inputImage, CV_BGR2GRAY);
    MotionDetectionInterface::MotionDetection::createForegroundMask(inputImage,foregroundMask);
    cv::Mat testImage=cv::imread("../exp/testBenchForTestFunctions/outputToTestCreateForegroundMask.png",0);

    cv::Mat diff;

    cv::absdiff(testImage,foregroundMask,diff);
    bool compare=cv::countNonZero(diff)==0;

    if(compare){
        return 1;
    } else{
        return 0;
    }

}

