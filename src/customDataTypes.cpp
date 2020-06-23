//
// Created by abhishekh.baskaran on 09/04/2019.
//

#include "inc/customDataTypes.h"

/**
 * @brief Function to set coordinates
 * @note Constructor is set using initializer list
 * @param xcoordinate
 * @param ycoordinate
 */
CustomDataTypes::Coordinate::Coordinate(int xcoordinate,int ycoordinate,int width,int height) : rectVertexX(xcoordinate),
rectVertexY(ycoordinate),width(width),height(height){

}

/**
 * @brief Function to return top left coordinates and width, height of a coordinate pair
 * @return vector with coordinate pair , width and height
 */
std::vector<int> CustomDataTypes::Coordinate::getCoordPair() {
    std::vector<int> coordinateOutput={rectVertexX,rectVertexY,width,height};
    return coordinateOutput;
}

/**
 * @brief Function to wrap rectangle to Coordinate
 * @param rectangle
 */
CustomDataTypes::Coordinate::Coordinate(const cv::Rect & rectangle) : rectVertexX(rectangle.x),rectVertexY(rectangle.y),
width(rectangle.width),height(rectangle.height){

}

int CustomDataTypes::Coordinate::calculateArea() {
    int area=width*height;
    return area;
}

cv::Rect CustomDataTypes::Coordinate::convertToRect() {
    return cv::Rect(rectVertexX,rectVertexY,width,height);
}

void CustomDataTypes::Coordinate::resizeCoordinate(int rowFactor,int colFactor) {
    rectVertexX*=rowFactor;
    rectVertexY*=colFactor;
    width*=rowFactor;
    height*=colFactor;
}
