#! /bin/bash

cwd=$(pwd)

# Download Boost
cd src/ext
wget -O boost.tgz https://github.com/giuliojiang/pbrt-v3-IISPT-extras/releases/download/v1/boost_1_66_0.tar.gz
tar -vxf boost.tgz
cd $cwd
pwd