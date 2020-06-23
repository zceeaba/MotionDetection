#ifndef INC_TESTCLASSES_H
#define INC_TESTCLASSES_H

//
// Created by abhishekh.baskaran on 21/03/2019.
//


#include "inc/motionDetection.h"

namespace TestInterface{
    /**
     * @class TestClasses
     * @brief Derived class for testing the protected methods of other classes in the library
     */
    class TestClasses : public MotionDetectionInterface::MotionDetection{

    public:
        /// Constructor Definition
        TestClasses()= default;

        /// Destructor Definition needs to override base class destructor which should be virtual
        ~TestClasses() override= default;

        /// Test Difference image function
        int testGetDiffImage(const cv::Mat& currentImage,const cv::Mat& backgroundImage);

        /// Test foreground mask subtractor function
        int testCreateForegroundMask();



    };
}


#endif //INC_TESTCLASSES_H
