#! /bin/bash

cwd=$(pwd)

# Download Boost and compile it
cd src/ext
wget -O boost.tgz https://github.com/giuliojiang/pbrt-v3-IISPT-extras/releases/download/v1/boost_1_66_0.tar.gz
tar -vxf boost.tgz
extdir=$(pwd)
boostbindir="${extdir}/boost_bin"
cd boost_1_66_0
./bootstrap.sh --prefix=${boostbindir}
./b2 install
cd $cwd
pwd