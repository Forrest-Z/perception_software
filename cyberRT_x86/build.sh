#!/bin/bash
BUILD_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
third_path="${BUILD_PATH}/third_party"

buildDirName="${BUILD_PATH}/build"
if [ ! -d $buildDirName ];then
mkdir $buildDirName
else
echo $buildDirName
fi

export LD_LIBRARY_PATH=${third_path}/protobuf/lib:$LD_LIBRARY_PATH

cd build
cmake -DCMAKE_INSTALL_PREFIX=${buildDirName}/cyberRT ..
make -j8
make install


