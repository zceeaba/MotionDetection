//
// Created by abhishekh.baskaran on 19/03/2019.
//
#include "ext/utilities/ext/doctest/doctest/doctest.h"
#include "inc/motionDetection.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/ocl.hpp>
#include "inc/imageUtils.h"
#include "inc/signalHandler.h"
#include "inc/customDataTypes.h"
#include "testClasses.h"
#include "iostream"
#include "fstream"
#include <chrono>
#include <thread>



void runMotionDetection(std::fstream& testCoordinates, MotionDetectionInterface::MotionDetection& motionDetector,
        cv::Mat& frame, std::chrono::duration<double>& elapsed, bool mode){
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::high_resolution_clock::now();
    std::vector<CustomDataTypes::Coordinate> boundingBoxes;
    motionDetector.backgroundSubtractor(frame,boundingBoxes);
    std::chrono::time_point<std::chrono::system_clock> end=std::chrono::high_resolution_clock::now();
    elapsed+=(end-start);


    if(!mode) {
        for (CustomDataTypes::Coordinate coord:boundingBoxes) {
            for (int element:coord.getCoordPair()) {
                int value;
                testCoordinates >>value;
                CHECK((element - value) == 0);
            }
        }
    }

}

void runTestOnVideos(std::fstream& testCoordinates,const Config::configParser& configParser){

    cv::ocl::setUseOpenCL(false);

    /// Initialize signal Handler
    SignalHandler signalHandler;

    /// Check if code should run in debug mode
    bool debug = configParser.getDetectorDebug();


    /// Initialize motion detector constructor
    MotionDetectionInterface::MotionDetection motionDetector("../cfg/motionDetectionConfig.ini", debug);

    /// Record start time
    int frameCounter=0;

    // Video Numbers 67-74 to work with selected test videos from the ../exp/testVideosMotionDetection folder
    int someNumber=configParser.getTestVideoStart();

    std::cout<<someNumber;
    int videoNumber=configParser.getTestVideoStart();

    /// Parameter to run in case of profiling
    bool profileMode= configParser.getProfileMode();

    std::chrono::duration<double> elapsed(0);

    while(videoNumber<=configParser.getTestVideoEnd()){

        cv::VideoCapture videoHandle(configParser.getVideoFileLocation()+std::to_string(videoNumber)+".mp4");
        while(true){
            cv::Mat frame;
            /// Capture frame-by-frame
            videoHandle >> frame;

            ///  If the frame is empty, break immediately
            if (frame.empty()){
                break;
            }

            frameCounter++;

            runMotionDetection(testCoordinates, motionDetector, frame, elapsed, profileMode);




        }
        videoNumber++;
    }

    if(debug){
        motionDetector.releaseWriter();
    }

    if(profileMode)
    {
        std::cout<<" Time per frame: "<<elapsed.count()/frameCounter;
        std::cout << " Total Elapsed time: " << elapsed.count() << " s\n";
    }

}


TEST_CASE("Performance Testing The Motion Detection algorithms")
{
    /// Config parser
    Config::configParser configParser("../test/motionDetectionTestConfig.ini");
    configParser.loadParser(false);

    int algorithm=configParser.getTestAlgorithm();
    switch(algorithm){
        case 1:{
            std::fstream testCoordinates;
            testCoordinates.open("../exp/testBenchForTestFunctions/MOG2GPUtestCoordinates.txt",std::ios::in);
            runTestOnVideos(testCoordinates,configParser);
            testCoordinates.close();
            break;
        }
        case 2:{
            std::fstream testCoordinates;
            testCoordinates.open("../exp/testBenchForTestFunctions/MOG2CPUtestCoordinates.txt",std::ios::in);
            runTestOnVideos(testCoordinates,configParser);
            testCoordinates.close();
            break;
        }
        case 0:
        default:{
            /// Read data from test file
            std::fstream testCoordinates;
            testCoordinates.open("../exp/testBenchForTestFunctions/FrameDifferencetestCoordinates.txt",
                                 std::ios::in);
            runTestOnVideos(testCoordinates,configParser);
            testCoordinates.close();
            break;
        }
    }
}



TEST_CASE("Testing the get diff image function")
{
    cv::Mat inputImage=cv::imread("../exp/inputToTestCreateForegroundMask.jpg");
    TestInterface::TestClasses testingObject;
    CHECK(testingObject.testGetDiffImage(inputImage,inputImage)==1);
}

TEST_CASE("Testing the foreground Mask Function")
{
    TestInterface::TestClasses testingObject;
    CHECK(testingObject.testCreateForegroundMask()==1);
}

