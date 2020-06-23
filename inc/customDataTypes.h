#ifndef INC_CUSTOMDATATYPES_H
#define INC_CUSTOMDATATYPES_H

//
// Created by abhishekh.baskaran on 09/04/2019.
//


#include "opencv2/opencv.hpp"


namespace CustomDataTypes{
    /**
     * @class Coordinate
     * @brief Stores coordinate data points for output in the form of (x coordinate,y coordinate,width,height)
     */
    class Coordinate{

    public:
        /**
         * @brief Set the x coordinate, y coordinate of the top-left diagonal vertex, width and height
         * @param xcoordinate
         * @param ycoordinate
         * @param width
         * @param height
         */
        Coordinate(int xcoordinate,int ycoordinate,int width,int height);
        /**
         * @brief Constructor to initialize class members by unwrapping a cv rectangle object
         * @param rectangle
         */
        explicit Coordinate(const cv::Rect & rectangle);
        /**
         * @brief Returns a tuple with the coordinate attributes
         * @return
         */
        std::vector<int> getCoordPair();
        /**
         * @brief Calculates the are of the bounding box by multiplying width and height
         * @return area of the bounding box
         */
        int calculateArea();
        /**
         * @brief Return opencv rectangle object from the coordinate object parameters
         * @return
         */
        cv::Rect convertToRect();

        void resizeCoordinate(int scalefactorRow,int scalefactorCol);
    private:
        int rectVertexX;
        int rectVertexY;
        int width;
        int height;
    };
    /// Enum to specify which algorithm to use
    enum class algorithmEnum {
        FrameDifference,
        MOG2,
        _ERROR
    };
    /// Enum to specify the morphing technique for dilation/erosion
    enum class morphingTechniquesEnum{
        RECTANGLE=0,
        CROSS=1,
        ELLIPSE=2,
        _ERROR=255
    };
    /// Enum to hold processor types for mog2 algorithm
    enum class processorEnum{
        GPU,
        CPU,
        _ERROR
    };
}


#endif //MOTIONDETECTION_CUSTOMDATATYPES_H
