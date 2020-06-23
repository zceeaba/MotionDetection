//
// Created by abhishekh.baskaran on 20/03/2019.
//

#include "inc/imageUtils.h"



namespace Utils{
    /**
 * @brief Utility function to write an image to a file
 * @param motionImage
 * @param fileName
 * @return
 */
    int imageUtils::writeImageToFile(cv::Mat & motionImage, const std::string & fileName) {
        std::vector<int> compressionParams;
        compressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
        compressionParams.push_back(100);
        try {
            const char* fileNameConstant=fileName.c_str();
            cv::imwrite(fileNameConstant, motionImage, compressionParams);
        }
        catch (std::runtime_error& ex) {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
            return 1;
        }

        return 0;

    }


/**
 * @brief Create a video writer object with optimal codec
 * @param videoFileLocation
 * @param frameRate
 * @param frameWidth
 * @param frameHeight
 * @param isColour
 * @return
 */
    cv::VideoWriter imageUtils::createVideoWriter(const std::string& videoFileLocation,int frameRate,int frameWidth,
                                                  int frameHeight,bool isColour){
        cv::VideoWriter bgVideo(videoFileLocation,cv::VideoWriter::fourcc('h','2','6','4'),
                                frameRate ,cv::Size(frameWidth,frameHeight),isColour);
        return bgVideo;
    }

    /**
 * @brief Draw bounding boxes on display image for display purposes
 * @param displayImage
 * @param nmsBoundingBoxes
 */

    void imageUtils::drawRectangles(cv::Mat &displayImage,std::vector<cv::Rect> &nmsBoundingBoxes) {
        for(const cv::Rect& bb:nmsBoundingBoxes){

            cv::rectangle(displayImage,
                          bb,
                          cv::Scalar(0,0,255));
        }
    }
    /**
     * @brief Convert rectangles to a vector of coordinates(bounding boxes)
     * @param Rectangles
     * @param boundingBoxes
     */
    void imageUtils::convertRectsToBB(const std::vector<cv::Rect> &Rectangles,
            std::vector<CustomDataTypes::Coordinate>& boundingBoxes) {
        boundingBoxes.clear();
        for(const cv::Rect& rectangle:Rectangles){
            boundingBoxes.emplace_back(CustomDataTypes::Coordinate(rectangle));
        }
    }

}




