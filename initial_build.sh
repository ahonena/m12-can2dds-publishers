#!/bin/sh
cd src/IDL;
./build_types.py;
cd ..; cd ..;
mkdir build;
cd build;
cmake ..;
make
