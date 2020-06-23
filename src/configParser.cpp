//
// Created by abhishekh.baskaran on 25/03/2019.
//

#include "inc/configParser.h"

#include "ext/inih-headeronly/INIReader.h"


Config::configParser::configParser(const std::string& configFileLocation ) :
compressionRatio(0),
foregroundMaskThreshold(0),
bufferSize(0),learningRate(0),
gamma(0),
dilationKernelSize(0),
erosionKernelSize(0),
areaThreshold(0),
shadowValue(0),
detectShadows(0),
resizeFactor(0),
nmsThreshold(0),
testAlgorithm(0),
testVideoStart(0),
testVideoEnd(0),
history(0),
mogThreshold(0),
mogLearningRate(0),
detectorDebug(0),
profileMode(false),
douglasPeuckerConstant(0),
debugAlpha(0.0),
debugGamma(0)
{
    this->configLocation=configFileLocation;
}

bool Config::configParser::loadParser(bool debug=false) {

    INIReader reader(configLocation);

    if (reader.ParseError() < 0) {
        return false;
    }
    else{
        videoFileLocation=reader.Get("test", "videoFileLocation", "UNKNOWN");
        fileWriteLocation=reader.Get("test", "fileWriteLocation", "UNKNOWN");
        compressionRatio=reader.GetInteger("test", "compressionRatio", -1);
        foregroundMaskThreshold=reader.GetReal("MOVINGAVERAGE", "foregroundMaskThreshold", -1);
        bufferSize=reader.GetInteger("test", "bufferSize", -1);
        learningRate=reader.GetReal("MOVINGAVERAGE","learningRate",-1);
        gamma=reader.GetReal("MOVINGAVERAGE","gammaMovingAverage",-1);
        dilationType=reader.Get("MORPHOLOGICAL", "dilationType", "UNKNOWN");
        erosionType=reader.Get("MORPHOLOGICAL", "erosionType", "UNKNOWN");
        dilationKernelSize=reader.GetInteger("MORPHOLOGICAL","dilationKernelSize",-1);
        erosionKernelSize=reader.GetInteger("MORPHOLOGICAL","erosionKernelSize",-1);
        algorithm=reader.Get("ALGORITHM","algorithm","UNKNOWN");
        processorType=reader.Get("ALGORITHM","processorType","UNKNOWN");
        areaThreshold=reader.GetReal("GENERAL","areaThresholdForBb",-1);
        shadowValue=reader.GetInteger("MOG","shadowValue",-1);
        detectShadows=reader.GetInteger("MOG","detectShadows",-1);
        resizeFactor=reader.GetReal("GENERAL","resizeFactor",-1);
        nmsThreshold=reader.GetReal("GENERAL","nmsThreshold",-1);
        testAlgorithm=reader.GetInteger("test","testAlgorithm",-1);
        testVideoStart=reader.GetInteger("test","testVideoStart",-1);
        testVideoEnd=reader.GetInteger("test","testVideoEnd",-1);
        history=reader.GetInteger("MOG","history",-1);
        mogThreshold=reader.GetInteger("MOG","mogThreshold",-1);
        mogLearningRate=reader.GetInteger("MOG","mogLearningRate",-1);
        detectorDebug=reader.GetInteger("test","debug",-1);
        profileMode=reader.GetBoolean("test","profileMode",-1);
        douglasPeuckerConstant=reader.GetInteger("CONTOURS","douglasPeuckerConstant",-1);
        debugAlpha = reader.GetReal("DEBUG","debugAlpha",-1);
        debugGamma = reader.GetReal("DEBUG","debugGamma",-1);
        if(debug){
            std::cout << "Config loaded from "<<configLocation<<std::endl<< ", videoFileLocation="
                      << videoFileLocation <<std::endl << ", fileWriteLocation="
                      << fileWriteLocation <<std::endl << ", compressionRatio="
                      << compressionRatio <<std::endl << ", foregroundMaskThreshold="
                      << foregroundMaskThreshold <<std::endl << ", bufferSize="
                      << bufferSize << std::endl<<",learningRate="
                      << learningRate <<std::endl<<",gamma="
                      << gamma <<std::endl<<",dilation Type="
                      << dilationType <<std::endl<<"erosion Type="
                      << erosionType <<std::endl<<"dilation Kernel Size="
                      << dilationKernelSize <<std::endl<<"erosion Kernel Size="
                      << erosionKernelSize <<std::endl<<"processorType="
                      << processorType <<std::endl<<"algorithm="
                      << algorithm <<std::endl<<"areaThreshold="
                      << areaThreshold<<std::endl<<"shadowValue="
                      << shadowValue<<std::endl<<"shadowDetect="
                      << detectShadows<<std::endl<<"resizeFactor="
                      << resizeFactor<<std::endl<<"nmsThreshold="
                      << nmsThreshold<<std::endl<<"Test Algorithm="
                      << testAlgorithm<<std::endl<<"Start point for test videos="
                      << testVideoStart<<std::endl<<"End point for test videos="
                      << testVideoEnd<<std::endl<<"History value for MOG="
                      << history<<std::endl<<"Threshold value for MOG="
                      << mogThreshold<<std::endl<<"Learning Rate for MOG="
                      << mogLearningRate<<std::endl<<"Debug Value for test="
                      << detectorDebug<<std::endl<<"Douglas Peucker Constant="
                      << douglasPeuckerConstant<<std::endl<<"Alpha value for addweighted in debug mode"
                      << debugAlpha<<std::endl<<"Gamma value for addweighted in debug mode"
                      << debugGamma<<std::endl;
        }
        return true;
    }


}

const std::string &Config::configParser::getConfigLocation() const {
    return configLocation;
}

const std::string &Config::configParser::getVideoFileLocation() const {
    return videoFileLocation;
}

const std::string &Config::configParser::getFileWriteLocation() const {
    return fileWriteLocation;
}

long Config::configParser::getCompressionRatio() const {
    return compressionRatio;
}

double Config::configParser::getForegroundMaskThreshold() const {
    return foregroundMaskThreshold;
}

int Config::configParser::getBufferSize() const {
    return bufferSize;
}

double Config::configParser::getLearningRate() const {
    return learningRate;
}

double Config::configParser::getGamma() const {
        return gamma;
}

CustomDataTypes::morphingTechniquesEnum Config::configParser::getDilationType() const {
    CustomDataTypes::morphingTechniquesEnum dilationEnumType;
    if(dilationType=="RECTANGLE"){
        dilationEnumType=CustomDataTypes::morphingTechniquesEnum::RECTANGLE;
        return dilationEnumType;
    }
    else if(dilationType=="CROSS"){
        dilationEnumType=CustomDataTypes::morphingTechniquesEnum::CROSS;
        return dilationEnumType;
    }
    else if(dilationType=="ELLIPSE"){
        dilationEnumType=CustomDataTypes::morphingTechniquesEnum::ELLIPSE;
        return dilationEnumType;
    } else{
        return CustomDataTypes::morphingTechniquesEnum::_ERROR;
    }
}

CustomDataTypes::morphingTechniquesEnum Config::configParser::getErosionType() const {
    CustomDataTypes::morphingTechniquesEnum erosionEnumType;
    if(erosionType=="RECTANGLE"){
        erosionEnumType=CustomDataTypes::morphingTechniquesEnum::RECTANGLE;
        return erosionEnumType;
    }
    else if(erosionType=="CROSS"){
        erosionEnumType=CustomDataTypes::morphingTechniquesEnum::CROSS;
        return erosionEnumType;
    }
    else if(erosionType=="ELLIPSE"){
        erosionEnumType=CustomDataTypes::morphingTechniquesEnum::ELLIPSE;
        return erosionEnumType;
    }
    else{
        return CustomDataTypes::morphingTechniquesEnum::_ERROR;
    }

}

int Config::configParser::getDilationKernelSize() const {
    return dilationKernelSize;
}

int Config::configParser::getErosionKernelSize() const {
    return erosionKernelSize;
}

CustomDataTypes::processorEnum Config::configParser::getProcessorType() const {
    if(processorType=="GPU"){
        return CustomDataTypes::processorEnum::GPU;
    }
    else if(processorType=="CPU"){
        return CustomDataTypes::processorEnum::CPU;
    }
    else{
        return CustomDataTypes::processorEnum::_ERROR;
    }
}

float Config::configParser::getAreaThreshold() const {
    return areaThreshold;
}

int Config::configParser::getShadowValue() const {
    return shadowValue;
}

bool Config::configParser::getDetectShadows() const {
    if(detectShadows==1){
        return true;
    }
    else if(detectShadows==0){
        return false;
    }
    else{
        throw std::invalid_argument("Wrong boolean value for detect shadows");
    }

}

float Config::configParser::getResizeFactor() const {
    return resizeFactor;
}

CustomDataTypes::algorithmEnum Config::configParser::getAlgorithm() const {
    if(algorithm=="FrameDifference"){
        return CustomDataTypes::algorithmEnum::FrameDifference;
    }
    else if(algorithm=="MOG2"){
        return CustomDataTypes::algorithmEnum::MOG2;
    }
    else{
        return CustomDataTypes::algorithmEnum::_ERROR;
    }
}

float Config::configParser::getNmsThreshold() const {
    return nmsThreshold;
}

int Config::configParser::getTestAlgorithm() const {
    return testAlgorithm;
}

int Config::configParser::getTestVideoStart() const {
    return testVideoStart;
}

int Config::configParser::getTestVideoEnd() const {
    return testVideoEnd;
}

int Config::configParser::getHistory() const {
    return history;
}

int Config::configParser::getMogThreshold() const {
    return mogThreshold;
}

int Config::configParser::getMogLearningRate() const {
    return mogLearningRate;
}

bool Config::configParser::getDetectorDebug() const {
    if(detectorDebug == 1){
        return true;
    }
    else if(detectorDebug == 0){
        return false;
    }
    else{
        throw std::invalid_argument("Wrong integer value for debug mode");
    }
}

bool Config::configParser::getProfileMode() const {
    return profileMode;
}

int Config::configParser::getDouglasPeuckerConstant() const {
    return douglasPeuckerConstant;
}

float Config::configParser::getDebugAlpha() const {
    return debugAlpha;
}

float Config::configParser::getDebugGamma() const {
    return debugGamma;
}


