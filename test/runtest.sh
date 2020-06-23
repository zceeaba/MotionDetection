#!/usr/bin/env bash

# Shell script for downloading testbench for motion detector and running test binary

# Exit on failed command
set -o errexit

# Location of video files locally
readonly videoStorageLocation="../exp/"

readonly DownloadLocation=""

# Name of zip to download
readonly fileName="testVideosMotionDetection.tar.bz2"

function main(){
# Change directory to exp folder
cd ../exp/

# Obtain package from download location
curl -O ${DownloadLocation}

# Extract zipped file
tar -xvjf ${fileName}

# Build testRunner makefile using cmake
cd ../build
cmake -DBUILD_TESTS=ON ..

# Build testRunner binary
make -j${nproc}

# Change to binary folder
cd ../bin/

# Run test Runner Binary
./motionDetectionTestRunner
}

main "$@"
