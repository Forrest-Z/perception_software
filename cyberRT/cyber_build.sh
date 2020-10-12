#!/bin/bash
BUILD_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
third_path="${BUILD_PATH}/third_party"
build_path="${BUILD_PATH}/build"
if [ -n "$2" ]; then
    install_prefix=$2
else
    install_prefix=${build_path}/cyberRT
fi
setup_path=${install_prefix}/bin/setup.bash

function setBulidPath()
{
    if [ ! -d $build_path ];then
        mkdir $build_path
    else
        echo $build_path
    fi
}

function helpDoc()
{
    echo ""
    echo "compile project : ./build.sh [aarch64/x86_64/clean/install] [install dir]"
    echo ""
}

if [ "$1" == 'aarch64' ]; then
    echo "platform is $1"
    setBulidPath
    cd build
    export LD_LIBRARY_PATH=${third_path}/$1/protobuf/lib:$LD_LIBRARY_PATH
    cmake -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 -DCMAKE_INSTALL_PREFIX=${install_prefix} ..
    make -j8
elif [ "$1" == 'x86_64' ]; then
    echo "platform is $1"
    setBulidPath
    cd build
    export LD_LIBRARY_PATH=${third_path}/$1/protobuf/lib:$LD_LIBRARY_PATH
    cmake -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=x86_64 -DCMAKE_INSTALL_PREFIX=${install_prefix} ..
    make -j8
elif [ "$1" == 'install' ]; then
    echo "install"
    cd build
    make install
    echo "source ${setup_path}" >> ~/.bashrc
    if [ ! -d "/home/$USER/cyber_data/log" ]; then
      echo "cyber log not exist, create dir /home/$USER/cyber_data/log"
      mkdir -p /home/$USER/cyber_data/log
    fi
elif [ "$1" == 'clean' ]; then
    echo "clean"
    rm -rf $build_path
else
    echo "This Platform not support:$1"
    helpDoc
    exit 1
fi
