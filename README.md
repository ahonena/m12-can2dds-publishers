# m12-can2dds-publishers
This project contains implementation for publishing CAN-bus level information of TUNI M12 (Wille) machine to DDS.

## Dependencies:
This software depends upon [PEAK-can basic API](https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1)
and [RTI Connext 6.0 Traditional C++ API](https://community.rti.com/documentation).

## About the hardware and software platforms:
The hardware is iEi TANK-720 (blue) rugged PC with 2xPCAN mPCIe-cards, each having 2 CAN channels.
The operating system is currently x64 Linux (Xubuntu) with preempt_rt patch (4.14.87-rt49). RTI connext needs at 
least GCC 7.3.0.
The build system is based on custom python scripts, cmake and make.

## Building:
The DDS requires automatically generated C++ code to implement our own datatypes defined in **.idl** files. This is done with 
[rtiddsgen]()https://community.rti.com/static/documentation/connext-dds/6.0.0/doc/manuals/connext_dds/code_generator/RTI_CodeGenerator_UsersManual.pdf  

The **rtiddsgen** is used within a script called 
[build_types.py](https://github.com/ahonena/m12-can2dds-publishers/blob/master/src/IDL/build_types.py)

This script generates the C++ code for the types and compiles them with the given makefile. 

After this, the build is setup with cmake and then compiled with make.
The build script also generates a list of necessary #include commands 
[here](https://github.com/ahonena/m12-can2dds-publishers/blob/master/src/IDL/command_for_includes.sh), 
if additional types are  needed or any existing types are renamed.

