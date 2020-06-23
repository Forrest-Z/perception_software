#!/bin/bash
BUILD_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
third_path="${BUILD_PATH}/third_party"
build_path="${BUILD_PATH}/build"
if [ -n "$1" ]; then
    install_prefix=$1
else
    install_prefix=${build_path}/cyberRT
fi
setup_path=${install_prefix}/bin/setup.bash

if [ ! -d $build_path ];then
    mkdir $build_path
else
    echo $build_path
fi


export LD_LIBRARY_PATH=${third_path}/protobuf/lib:$LD_LIBRARY_PATH

cd build
cmake -DCMAKE_INSTALL_PREFIX=${install_prefix} ..
make -j8
make install

echo "source ${setup_path}" >> ~/.bashrc



