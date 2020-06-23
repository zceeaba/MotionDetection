#ifndef INC_IMAGEUTILS_H
#define INC_IMAGEUTILS_H

//
// Created by abhishekh.baskaran on 20/03/2019.
//


#include "opencv2/opencv.hpp"
#include "inc/customDataTypes.h"



namespace Utils{
    /**
     * @class imageUtils
     * @brief Utility Class with support functions for testing and working with data
     */
    class imageUtils {
    public:
        /**
         * @brief Default constructor
         */
        imageUtils()=default;


        /**
        * @brief Destroys the imageUtils object and releases resources.
        *
        */
        virtual ~imageUtils()= default;

        /**
         * @brief Function to write an image to a file
         * @param motionImage
         * @param fileName
         * @return Return 0 or 1 based on success/failure
         */
        int writeImageToFile(cv::Mat & motionImage, const std::string & fileName);



        /**
         * @brief Create a video writer object with optimal codec
         * @param videoFileLocation
         * @param frameRate
         * @param frameWidth
         * @param frameHeight
         * @param isColour
         * @return
         */
        static cv::VideoWriter createVideoWriter(const std::string& videoFileLocation,int frameRate,int frameWidth,
                int frameHeight,bool isColour);

        /**
         * @brief Draw bounding boxes on display image for display purposes
         * @param displayImage
         * @param nmsBoundingBoxes
         */
        void drawRectangles(cv::Mat &displayImage,std::vector<cv::Rect> &nmsBoundingBoxes);

        /**
         * @brief convert a vector of opencv rectangles to a vector of bounding boxes
         * @param Rectangles
         */
        void convertRectsToBB(const std::vector<cv::Rect> &Rectangles,
                              std::vector<CustomDataTypes::Coordinate>& boundingBoxes);

    };
}


#endif //MOTIONDETECTION_IMAGEUTILS_H
