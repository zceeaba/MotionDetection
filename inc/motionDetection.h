#ifndef INC_MOTIONDETECTION_H
#define INC_MOTIONDETECTION_H

//
// Created by abhishekh.baskaran on 19/03/2019.
//


#include "opencv2/opencv.hpp"
#include "inc/imageUtils.h"
#include "inc/configParser.h"
#include "inc/customDataTypes.h"
#include "inc/signalHandler.h"
#include <stdexcept>
#include "ext/NMS/nms.hpp"

namespace MotionDetectionInterface{
    /**
     * @class MotionDetection
     * @brief Main class with functions to perform background subtraction and other motion detection algorithms
     */
    class MotionDetection {
    public:
        /**
         * @brief Default constructor that initializes member variables
         */
        MotionDetection();
        /**
         * @brief Constructor initializes private variables
         * @param debug Parameter to specify whether to operate in debug mode or not
         */
        MotionDetection(const std::string& configFileLocation,bool debug);

        /**
         * @brief Background subtraction function to separate images from the frame background
         * @note Return values are a coordinate describing a bounding box around the image
         * @param newFrame Frame input from another source
         * @param outputDetections Vector with bounding boxes and a foreground mask
         * @return
         */
        void backgroundSubtractor(const cv::Mat &newFrame,std::tuple<std::vector<CustomDataTypes::Coordinate>,
                cv::Mat> &outputDetections);

        /**
         * @brief Background subtraction function overloaded to return a resized bounding box that can be fitted
         * onto the original image
         * @param newFrame Frame input from another source
         * @param boundingBoxes A vector of coordinate objects drawn around the blobs from foreground segmentation
         */
        void backgroundSubtractor(const cv::Mat &newFrame,std::vector<CustomDataTypes::Coordinate>& boundingBoxes);

        /**
         * @brief Release private video writer variable
         */
        void releaseWriter();

        /**
        * @brief Destroys the interface with motion Detection and releases resources.
        */
        virtual ~MotionDetection()= default;



    protected:
        /**
         * @brief Get Difference image between two images
         * @param currentImage
         * @param backgroundImage
         * @param diffImage
         * @return
         */
        void getDiffImage(const cv::Mat& currentImage,const cv::Mat& backgroundImage,cv::Mat& diffImage);
        /**
         * @brief Get foreground mask by using a threshold value
         * @param diffImage
         * @param foregroundMask
         * @return
         */
        void createForegroundMask(cv::Mat& diffImage,cv::Mat& foregroundMask);
    private:
        /**
         * @brief Compute moving average over a defined set of previous images, buffer size defined in config
         * @note Uses internal buffer
         * @param currentImage
         * @return
         */
        void movingAverage(const cv::Mat& currentImage);
        /**
         * @brief Input Frame for comparison from a buffer using MOG2 Algorithm
         * @param frame
         * @param foreground
         * @param proccesor
         * @return
         */
        void compareTwoMOG2GPU(const cv::Mat &frame, cv::Mat &foreground);
        /**
         * @brief Input frames for comparison using frame difference method
         * @param frame1
         * @param foregroundMask
         * @return
         */
        void compareTwoUsingFrameDifference(cv::Mat frame1,cv::Mat& foregroundMask);
        /**
         * @brief Program to find the contours of the image and extract the blob
         * @param foregroundImage
         * @param displayImage
         * @return Bounding box rectangles
         */
        std::vector<CustomDataTypes::Coordinate> extractBlob(cv::Mat &foregroundImage,cv::Mat &displayImage);
        /**
         * @brief Takes input as foreground image and reduces thickness/whiteness of foreground object
         * @param foregroundImage
         * @param erosionElement Type of erosion to be performed-> e.g:Cross or Rectangle
         * @param erosionSize Size of kernel an odd number(kernel size:(2*(dilationSize)+1))
         * @return
         */
        void erosionImage(cv::Mat &foregroundImage,
                          CustomDataTypes::morphingTechniquesEnum erosionElement,int erosionSize);
        /**
         * @brief Increases the foreground object area and accentuates features
         * @param foregroundImage
         * @param dilationElement Type of dilation to be performed-> e.g:Cross or Rectangle
         * @param dilationSize Size of kernel an odd number(kernel size:(2*(dilationSize)+1))
         * @return dilated Image matrix
         */
        void dilationImage(cv::Mat &foregroundImage,
                           CustomDataTypes::morphingTechniquesEnum dilationElement,int dilationSize);
        /**
         * @brief Generates a bounding box to replace overlapping bounding boxes
         * @param boundingBoxes
         * @return
         */
        void nonMaximumSuppression(const std::vector<CustomDataTypes::Coordinate>& boundingBoxes,std::vector<cv::Rect>& nmsBoundingBoxes);

        /**
         * @brief Add foreground mask to original image
         * @param foregroundMask
         * @param displayFrame
         * @return Returns modified image for display
         */
        void addBlobToImageForDisplay(cv::Mat& foregroundMask, cv::Mat& displayFrame);


        /// Threshold value defines the selection criteria for pixel values
        double threshold;

        /// Initialize handler for opencv background subtraction
        cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2;

        /* FIXME: CUDA not enabled while compiling opencv.:
         * TODO: Change to cv::cuda::BackgroundSubtractorMOG2()
         * */
        cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2GPU;

        /// Initialize image utility object
        Utils::imageUtils Utility;

        /// Map to hold mappings between processor name and id
        CustomDataTypes::processorEnum processorMap;

        /// The background model frame
        cv::Mat meanFrame;

        /// Moving average buffer size
        double learningRate;

        /// Scalar value added to background model
        double gamma;

        /// Video Writer object
        cv::VideoWriter bgVideo;

        /// Choose Algorithm
        CustomDataTypes::algorithmEnum algorithm;

        /// Erosion and dilation parameters
        int dilationKernelSize;
        int erosionKernelSize;
        CustomDataTypes::morphingTechniquesEnum dilationType;
        CustomDataTypes::morphingTechniquesEnum erosionType;

        /// Debug parameter
        bool debug;

        /// Shadow value for the Background subtractor MOG2
        int shadowValue;

        /// Tells MOG2 whether to detect shadow or not
        bool shadowDetect;

        /// Area threshold for bounding box
        float areaThreshold;

        /// Resizing value for background subtraction method
        float resizeFactor;

        /// Non maximum suppression threshold for finding the largest bounding box from a group of overlapping boxes
        float nmsThreshold;                // Overlap threshold for boxes in the nms algorithm

        /// Mog config parameters
        int history;                      // History denotes the number of frames to use to adjust the background model
        int mogThreshold;                 // Values below the threshold are removed
        int mogLearningRate;              // Learning rate is used to set how fast to update the background model

        /// Contour config paramter
        int douglasPeuckerConstant;       // The douglas peucker constant is used in contour approximation

        /// Debug parameters
        float debugAlpha; // This constant defines the weight of the first array element in the addweighted function
        float debugGamma; // This is a scalar added to the sum after using addweighted

    };


}


#endif //INC_MOTIONDETECTION_H
