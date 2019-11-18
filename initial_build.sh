#!/bin/sh
set -e
cd src/IDL;
./build_types.py;
./command_for_includes.sh;
cd ..; cd ..;
mkdir build;
cd build;
cmake ..;
make;

