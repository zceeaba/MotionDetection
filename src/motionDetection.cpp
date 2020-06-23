//
// Created by abhishekh.baskaran on 19/03/2019.
//



#include <chrono>
#include "inc/motionDetection.h"



namespace MotionDetectionInterface{
    MotionDetection::MotionDetection() : threshold(0),processorMap(CustomDataTypes::processorEnum::_ERROR),
    learningRate(0),
    gamma(0),
    algorithm(CustomDataTypes::algorithmEnum::FrameDifference),
    dilationKernelSize(0),
    erosionKernelSize(0),
    dilationType(CustomDataTypes::morphingTechniquesEnum::_ERROR),
    erosionType(CustomDataTypes::morphingTechniquesEnum::_ERROR),
    debug(false),
    shadowValue(0),
    shadowDetect(true),
    areaThreshold(0),
    resizeFactor(0),
    nmsThreshold(0),
    history(0),
    mogThreshold(0),
    mogLearningRate(0),
    douglasPeuckerConstant(0),
    debugAlpha(0.0),
    debugGamma(0.0)
    {

    }
    /**
 * @brief Constructor initializes private variables
 * @param threshold
 * @param learningRate
 * @param algorithm
 * @param gamma
 */
    MotionDetection::MotionDetection(const std::string& configFileLocation,bool debug=false) {


        /// Config Parser
        Config::configParser configParser(configFileLocation);
        bool checkConfigParser=configParser.loadParser(debug);
        if(!checkConfigParser){
            throw std::invalid_argument("Unable to parse config,"
                                        " check if file exists in passed location: "+configFileLocation);
        }

        /// Initialize config values for background subtractor
        this->gamma=(double)configParser.getGamma();
        if(gamma<0){
            throw std::invalid_argument("Wrong gamma value choose a float between (0,1)");
        }

        this->threshold=(float)configParser.getForegroundMaskThreshold();
        if(threshold<0){
            throw std::invalid_argument("Wrong threshold value choose a float value between (0,255)");
        }


        /// Get the number of frames to consider for moving average
        this->learningRate=(double)configParser.getLearningRate();
        if(learningRate<0){
            throw std::invalid_argument("Wrong learning rate chose a value between (0,255.0)");
        }

        /// Set the algorithm for the motion detector
        this->algorithm = configParser.getAlgorithm();
        if(algorithm==CustomDataTypes::algorithmEnum ::_ERROR){
            throw std::invalid_argument("Algorithm needs to be either FrameDifference or MOG2");
        }


        this->dilationType = configParser.getDilationType();
        if(dilationType==CustomDataTypes::morphingTechniquesEnum::_ERROR){
            throw std::invalid_argument("Unknown dilation type, change value");
        }

        this->erosionType = configParser.getErosionType();
        if(erosionType==CustomDataTypes::morphingTechniquesEnum::_ERROR){
            throw std::invalid_argument("Unknown erosion type, change value");
        }

        this->dilationKernelSize = configParser.getDilationKernelSize();
        if(dilationKernelSize<0){
            throw std::invalid_argument("Dilation Kernel size not defined properly");
        }

        this->erosionKernelSize=configParser.getErosionKernelSize();
        if(erosionKernelSize<0){
            throw std::invalid_argument("Erosion Kernel size not defined properly");
        }


        /// If mog algorithm set processor type
        this->processorMap=configParser.getProcessorType();
        if(processorMap==CustomDataTypes::processorEnum::_ERROR){
            throw std::invalid_argument("Processor type needs to be either cpu or gpu");
        }

        /// Shadow value for the Background subtractor
        this->shadowValue=configParser.getShadowValue();
        if(shadowValue<0){
            throw std::invalid_argument("Shadow value needs to be between (0,255)");
        }

        /// MOG2 to detect shadow or not
        this->shadowDetect=configParser.getDetectShadows();

        /// Area threshold for bounding box
        this->areaThreshold=configParser.getAreaThreshold();
        if(areaThreshold<0){
            throw std::invalid_argument("Area threshold needs to be a float value and greater than zero");
        }

        /// Resizing value for background subtraction method
        this->resizeFactor=configParser.getResizeFactor();
        if(resizeFactor<0){
            throw std::invalid_argument("Resize factor is not a proper float value and greater than zero");
        }

        /// Mog config Parameters
        mogThreshold = configParser.getMogThreshold();
        if(mogThreshold<0){
            throw std::invalid_argument("mog Threshold needs to be defined and an integer, greater than 0");
        }

        mogLearningRate = configParser.getMogLearningRate();
        if(mogLearningRate>1){
            throw std::invalid_argument("mog Learning Rate needs to be a float value between 0 and 1");
        }

        history = configParser.getHistory();
        if(history<0){
            throw std::invalid_argument("mog history needs to be defined, an integer and greater than zero");
        }



        /// create Background Subtractor objects
        switch(processorMap){
            case CustomDataTypes::processorEnum::CPU:
                pMOG2 = cv::createBackgroundSubtractorMOG2(history,mogThreshold);
                pMOG2->setDetectShadows(shadowDetect);
                pMOG2->setShadowValue(shadowValue);
                break;
            case CustomDataTypes::processorEnum::GPU:
                if(cv::cuda::getCudaEnabledDeviceCount()==0){
                    throw std::invalid_argument("OPENCV not compiled with cuda support,use the CPU version");
                }
                else{
                    /* FIXME: CUDA not enabled while compiling opencv.:
                     * TODO: Change to cv::cuda::createBackgroundSubtractorMOG2()
                     * */
                    pMOG2GPU = cv::createBackgroundSubtractorMOG2(); //MOG2 approach
                    pMOG2GPU->setDetectShadows(shadowDetect);
                    pMOG2GPU->setShadowValue(shadowValue);
                }
                break;
            default:
            case CustomDataTypes::processorEnum::_ERROR:
                throw std::invalid_argument("No processor available");
        }

        nmsThreshold = configParser.getNmsThreshold();
        if(nmsThreshold<0){
            throw std::invalid_argument("Nms threshold needs to be a float value between 0 and 1");
        }

        douglasPeuckerConstant = configParser.getDouglasPeuckerConstant();
        if(douglasPeuckerConstant<0){
            throw std::invalid_argument("Douglas peucker constant needs to be an integer and greater than zero");
        }



        /// Set debug value
        this->debug=debug;

        if(debug){
            bgVideo=Utils::imageUtils::createVideoWriter("../exp/videoForDemo.avi",15,1920,1080, false);
            debugAlpha = configParser.getDebugAlpha();
            if(debugAlpha==-1){
                throw std::invalid_argument("debugAlpha needs to be a float value between 0 and 1");
            }
            debugGamma = configParser.getDebugGamma();
            if(debugGamma==-1){
                throw std::invalid_argument("debugGamma needs to be a float value between 0 and 1");
            }
        }

    }
/**
 * @brief Input two frames for comparison using frame difference method
 * @param frame1
 * @return foreground mask matrix
 */
    void MotionDetection::compareTwoUsingFrameDifference(cv::Mat frame1,cv::Mat& foregroundMask) {

        /// Hardcoded 1 as the output will always be in single channel to assist in finding contours
        cv::cvtColor(frame1,frame1,cv::COLOR_BGR2GRAY,1);

        cv::Mat diffImage;

        /**
         * Check if the frame is empty to clarify whether the background model has been updated,if not declare the
         * mean frame to take the value of the current frame: This avoids a transient response in the background model
         * during initialization
         */
        if(meanFrame.empty()){
            meanFrame=frame1;
        }
        else{
            movingAverage(frame1);
        }

        /**
         * Find difference between the images
         */
        getDiffImage(frame1,meanFrame,diffImage);


        /// Perform erosion and dilation of images
        dilationImage(diffImage,dilationType,dilationKernelSize);


        erosionImage(diffImage,erosionType,erosionKernelSize);




        /**
         * Extract Foreground from both images
         */
        createForegroundMask(diffImage,foregroundMask);

    }

/**
 * @brief Compute moving average over a defined set of previous images, buffer size defined in config
 * @param currentImage
 * @return
 */
    void MotionDetection::movingAverage(const cv::Mat& currentImage){
        /**
         * Calculates the moving average of all the previous images upto the current image with the help of a learning
         * rate and gamma value(white balance)
         */
        cv::addWeighted(currentImage,(1.0-learningRate),meanFrame,learningRate,gamma,meanFrame);
    }
/**
 * @brief Get Difference image between two images
 * @param currentImage
 * @param backgroundImage
 * @return return Difference image between the current image and background
 */
    void MotionDetection::getDiffImage(const cv::Mat& currentImage,const cv::Mat& backgroundImage,cv::Mat& diffImage){
        if(backgroundImage.empty()){
            diffImage=cv::Mat::zeros(currentImage.rows,currentImage.cols,currentImage.type());
        }
        else{
            cv::absdiff(currentImage,backgroundImage, diffImage);
        }

    }
/**
 * @brief Get foreground mask by using a threshold value
 * @param diffImage
 * @param foregroundMask
 * @return Return matrix with the foreground mask
 */
    void MotionDetection::createForegroundMask(cv::Mat& diffImage,cv::Mat& foregroundMask){
        /// Thresholding takes the input moving image and generates a foreground mask
        /// Thresh_Binary constant helps in thresholding by setting values of parameters that are greater than threshold
        /// To 255
        cv::threshold(diffImage,foregroundMask,threshold,255,cv::THRESH_BINARY);
    }
/**
 * @brief Background subtraction function to separate images from the video background
 * @note Return values are a coordinate describing the centroid of an image
 * @param newFrame
 * @param algorithm specifies the algorithm for background subtraction to be used
 * @return
 */
    void MotionDetection::backgroundSubtractor(const cv::Mat &newFrame,
            std::tuple<std::vector<CustomDataTypes::Coordinate>,cv::Mat> &outputDetections){
        cv::Mat movingObject;
        cv::Mat resizedImage;

        /// Downscale image to reduce noise and speed up background subtraction
        cv::resize(newFrame,resizedImage,cv::Size(),resizeFactor,resizeFactor);

        if(algorithm==CustomDataTypes::algorithmEnum::FrameDifference){
            compareTwoUsingFrameDifference(resizedImage,movingObject);
        }
        else if(algorithm==CustomDataTypes::algorithmEnum::MOG2){
            compareTwoMOG2GPU(resizedImage,movingObject);
        } else{
            throw std::invalid_argument("Received a wrong algorithm");
        }



        /// Extract bounding box from image
        std::vector<CustomDataTypes::Coordinate> result;
        result=extractBlob(movingObject,resizedImage);
        outputDetections=std::make_tuple(result,movingObject);

    }
/**
 * @brief Input Frame for comparison from a buffer using MOG2 Algorithm
 * @param frame
 * @param foreground
 * @param proccesor The processor holds the type of processor to run the MOG2 algorithm on
 * @return an Image with the foreground
 */

    void MotionDetection::compareTwoMOG2GPU(const cv::Mat &frame, cv::Mat &foreground) {


        switch (processorMap){
            case CustomDataTypes::processorEnum::GPU:{
                cv::cuda::GpuMat image = cv::cuda::GpuMat(frame);
                cv::cuda::GpuMat foregroundMaskGPU;


                pMOG2GPU->apply(image, foregroundMaskGPU, mogLearningRate);

                foregroundMaskGPU.download(foreground);


                break;
            }
            case CustomDataTypes::processorEnum::CPU:{
                pMOG2->apply(frame, foreground,mogLearningRate);

                break;
            }
            default:{
                throw std::invalid_argument("Received a wrong proccesor type");
            }

        }

    }
/**
 * @brief Erosion of image function to remove noise
 * @param foregroundImage
 * @param erosionElement The type of erosion to be performed
 * @param erosionSize Size of the structuring element for erosion
 * @return Matrix with output image
 */
    void MotionDetection::erosionImage(cv::Mat &foregroundImage,
            CustomDataTypes::morphingTechniquesEnum erosionElement,int erosionSize) {
        int erosionAlgorithm;
        cv::Mat kernel;

        switch(erosionElement){
            case CustomDataTypes::morphingTechniquesEnum::RECTANGLE:{
                erosionAlgorithm=cv::MORPH_RECT;
                break;
            }
            case CustomDataTypes::morphingTechniquesEnum::CROSS:{
                erosionAlgorithm=cv::MORPH_CROSS;
                break;
            }
            case CustomDataTypes::morphingTechniquesEnum::ELLIPSE:{
                erosionAlgorithm=cv::MORPH_ELLIPSE;
                break;
            }
            default:{
                throw std::invalid_argument("Received a wrong erosion type");
            }
        }


        if(erosionSize>0){
            /// Kernel based on structuring element with configurable size and center
            kernel = getStructuringElement(erosionAlgorithm,cv::Size( 2*erosionSize + 1, 2*erosionSize+1 ),
                                           cv::Point( erosionSize, erosionSize ));
        }
        else{
            /// Default kernel for performing erosion
            kernel=cv::Mat::ones((unsigned)3,(unsigned)3,CV_8UC1);
        }



        /// Erode the foreground
        cv::erode(foregroundImage,foregroundImage,kernel);

    }
/**
 * @brief Performs dilation on the input images to accentuate white ares compared to the darker
 * @param foregroundImage
 * @param dilationElement The type of dilation to perform
 * @param dilationSize The size of the dilation structuring element
 * @return
 */
    void MotionDetection::dilationImage(cv::Mat &foregroundImage,
                                        CustomDataTypes::morphingTechniquesEnum dilationElement,int dilationSize) {
        cv::Mat kernel;

        int dilationAlgorithm;
        switch(dilationElement){
            case CustomDataTypes::morphingTechniquesEnum::RECTANGLE:{
                dilationAlgorithm=cv::MORPH_RECT;
                break;
            }
            case CustomDataTypes::morphingTechniquesEnum::CROSS:{
                dilationAlgorithm=cv::MORPH_CROSS;
                break;
            }
            case CustomDataTypes::morphingTechniquesEnum::ELLIPSE:{
                dilationAlgorithm=cv::MORPH_ELLIPSE;
                break;
            }
            default:{
                throw std::invalid_argument("Received a wrong dilation type");
            }
        }

        if(dilationSize>0){
            /// Kernel based on structuring element with configurable size and center
            kernel = getStructuringElement(dilationAlgorithm,cv::Size( 2*dilationSize + 1, 2*dilationSize+1 ),
                                           cv::Point(dilationSize, dilationSize ));
        }
        else{
            /// Default kernel for performing erosion
            kernel=cv::Mat::ones((unsigned)3,(unsigned)3,CV_8UC1);
        }

        /// Dilate the foreground
        cv::dilate(foregroundImage,foregroundImage,kernel);

    }
/**
 * @brief Produces the biggest overlapping bounding box from the set of boxes
 * @param boundingBoxes Bounding boxes found in the contour detection procedure
 * @param nmsBoundingBoxes Non maximum suprressed bounding boxes
 */
    void MotionDetection::nonMaximumSuppression(
            const std::vector<CustomDataTypes::Coordinate>& boundingBoxes,std::vector<cv::Rect>& nmsBoundingBoxes) {
        if(!boundingBoxes.empty()){

            std::vector<std::vector<float>> newRectangles;
            for(CustomDataTypes::Coordinate box:boundingBoxes){
                std::vector<float> rectangleVector;
                float xMin=(float)box.getCoordPair()[0];
                float yMin=(float)box.getCoordPair()[1];
                float xMax=xMin+(float)box.getCoordPair()[2];
                float yMax=yMin+(float)box.getCoordPair()[3];
                rectangleVector.push_back(xMin);
                rectangleVector.push_back(yMin);
                rectangleVector.push_back(xMax);
                rectangleVector.push_back(yMax);
                newRectangles.push_back(rectangleVector);
            }
            nmsBoundingBoxes=nms(newRectangles,nmsThreshold);

        }
    }
/**
 * @brief Function to extract blob, rectangle from the image and return a bounding box that
 * corresponds to the segmented background
 * @param foregroundImage
 * @param displayImage
 * @return Vector with a list of coordinates
 */
    std::vector<CustomDataTypes::Coordinate> MotionDetection::extractBlob(cv::Mat &foregroundImage,
            cv::Mat &displayImage){
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        /// Using chain approx simple algorithm to save memory as it only utilizes end points
        cv::findContours(foregroundImage,contours,hierarchy,cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE);
        std::vector<CustomDataTypes::Coordinate> boundingBoxes;
        std::vector<cv::Rect> nmsBoundingBoxes;

        int imageArea=displayImage.rows*displayImage.cols;

        for(const std::vector<cv::Point_<int>,std::allocator<cv::Point_<int>>> & contour : contours)
        {
            /// Produce a closed polygon with object's contours
            std::vector<cv::Point> polygon;

            /// Douglas Peucker algorithm(approximate a closed polygon i.e.closest to our contour)
            /// Closes the contour by connecting the first and last vertices
            approxPolyDP(contour, polygon, douglasPeuckerConstant, true);

            /// Get polygon's bounding rectangles
            cv::Rect rectangle = boundingRect(polygon);
            CustomDataTypes::Coordinate boundingBox(rectangle);
            if(boundingBox.calculateArea()>(areaThreshold*imageArea)){
                boundingBoxes.push_back(boundingBox);
            }
        }
        nonMaximumSuppression(boundingBoxes,nmsBoundingBoxes);


        Utility.convertRectsToBB(nmsBoundingBoxes,boundingBoxes);


        return boundingBoxes;
    }


    /**
     * @brief Overloaded background subtraction function to seperate the image from its background and return
     * a bounding box where motion was detected
     * @param newFrame
     * @param boundingBoxes
     */
    void MotionDetection::backgroundSubtractor(const cv::Mat &newFrame,
                                               std::vector<CustomDataTypes::Coordinate>& boundingBoxes
                                               ) {

        /// Generate foregorund mask placeholder for passing to overloaded background subtractor function
        cv::Mat foreGroundMask;


        std::tuple<std::vector<CustomDataTypes::Coordinate>,cv::Mat> inputDetections;

        cv::Mat displayFrame=newFrame;

        /// Call overloaded background subtractor function
        inputDetections=std::make_tuple(boundingBoxes,foreGroundMask);

        MotionDetection::backgroundSubtractor(newFrame,inputDetections);

        foreGroundMask=std::get<1>(inputDetections);


        boundingBoxes=std::get<0>(inputDetections);

        /**
         *  UP scale or downscale the image to the original image size based on the ratio between the
         *  image shape after background subtraction and image shape before background subtraction
         */
        int scaleFactorRows=(int)(newFrame.rows/foreGroundMask.rows);
        int scaleFactorCols=(int)(newFrame.cols/foreGroundMask.cols);

        cv::Mat dest;

        for(CustomDataTypes::Coordinate& boundingBox:boundingBoxes){
            boundingBox.resizeCoordinate(scaleFactorRows,scaleFactorCols);
            if(debug){
                cv::rectangle(displayFrame,boundingBox.convertToRect(),cv::Scalar(0,255,0),3);
            }
        }

        if(debug){
            addBlobToImageForDisplay(foreGroundMask, displayFrame);
            /// Write output frame to video file
            bgVideo.write(displayFrame);

            cv::imshow("output",displayFrame);
            cv::waitKey(1);

        }

    }

    /**
     * @brief Add blob to image is a debug function used to superpose the foreground mask onto the original .
     * This superposition is performed using the addweighted function.
     * @param foregroundMask
     * @param displayFrame
     */
    void MotionDetection::addBlobToImageForDisplay(cv::Mat &foregroundMask, cv::Mat &displayFrame) {
        /// Resize Foreground mask back to original size image size
        cv::resize(foregroundMask,foregroundMask,cv::Size(),(1/resizeFactor),(1/resizeFactor));
        cv::Mat foreGroundMaskBGR;
        cv::Mat greenImg = cv::Mat(displayFrame.size(), displayFrame.type(), cv::Scalar(0,255,0));
        greenImg.copyTo(foreGroundMaskBGR, foregroundMask);

        cv::addWeighted(foreGroundMaskBGR, debugAlpha, displayFrame, 1-debugAlpha, debugGamma, displayFrame);

    }

    void MotionDetection::releaseWriter() {
        bgVideo.release();
    }


}


