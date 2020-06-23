#ifndef INC_CONFIGPARSER_H
#define INC_CONFIGPARSER_H

//
// Created by abhishekh.baskaran on 25/03/2019.
//

#include <iostream>
#include "inc/customDataTypes.h"



namespace Config{
    /**
     * @class configParser
     * @brief Config parser class to store configs and expose getter methods for the data
     */
    class configParser {


    public:
        /**
         * @param string denoting the path to file location of config
         * @brief Sets up config parser object to be read from specified location
         */
        explicit configParser(const std::string& fileLocation);

        /**
         * @brief Function loads the parameters from the specified config location into member variables
         * @param debug
         * @return
         */
        bool loadParser(bool debug);

        virtual ~configParser() = default;

    private:
        std::string configLocation;
        /// Config Parameters
        std::string videoFileLocation;
        std::string fileWriteLocation;
        long compressionRatio;
        double foregroundMaskThreshold;
        int bufferSize;
        double learningRate;
        double gamma;
        std::string dilationType;
        std::string erosionType;
        int dilationKernelSize;
        int erosionKernelSize;
        std::string processorType;
        std::string algorithm;
        float areaThreshold;
        int shadowValue;
        int detectShadows;
        float resizeFactor;
        float nmsThreshold;
        int testAlgorithm;
        int testVideoStart;
        int testVideoEnd;

        /// Parameters for mog algorithm
        int history;
        int mogThreshold;
        int mogLearningRate;

        /// Check whether detector should run in debug mode for the test
        int detectorDebug;

        /// Check whether detector should run in profile mode for the test
        bool profileMode;

        /// Parameter for douglas peucker closeness algorithm
        int douglasPeuckerConstant;

        /// Parameters for debug mode in the motion detection source code
        float debugAlpha; // This constant defines the weight of the first array element in the addweighted function
        float debugGamma; // This is a scalar added to the sum after using addweighted

    public:
        CustomDataTypes::morphingTechniquesEnum getDilationType() const;
        CustomDataTypes::morphingTechniquesEnum getErosionType() const;
        int getDilationKernelSize() const;
        int getErosionKernelSize() const;
        CustomDataTypes::processorEnum getProcessorType() const;
        const std::string &getConfigLocation() const;
        const std::string &getVideoFileLocation() const;
        const std::string &getFileWriteLocation() const;
        long getCompressionRatio() const;
        double getForegroundMaskThreshold() const;
        int getBufferSize() const;
        double getLearningRate() const;
        double getGamma() const;
        float getAreaThreshold() const;
        int getShadowValue() const;
        bool getDetectShadows() const;
        float getResizeFactor() const;
        CustomDataTypes::algorithmEnum getAlgorithm() const;
        float getNmsThreshold() const;
        int getTestAlgorithm() const;
        int getTestVideoStart() const;
        int getTestVideoEnd() const;
        int getHistory() const;
        int getMogThreshold() const;
        int getMogLearningRate() const;
        bool getDetectorDebug() const;
        bool getProfileMode() const;
        int getDouglasPeuckerConstant() const;
        float getDebugAlpha() const;
        float getDebugGamma() const;

    };

}


#endif //INC_CONFIGPARSER_H
