#ifndef M12_CAN2DDS_H
#define M12_CAN2DDS_H
#include <iostream>
#include <stdexcept>
#include <atomic>
#include <csignal>
#include <string>
#include <sstream>

#include "PCANBasic.h"


#include "M12_EMstop.hpp"
#include "M12_EMstopPlugin.hpp"
#include "M12_IMU_acceleration.hpp"
#include "M12_IMU_accelerationPlugin.hpp"
#include "M12_IMU_gyro.hpp"
#include "M12_IMU_gyroPlugin.hpp"
#include "M12_input_and_hsd.hpp"
#include "M12_input_and_hsdPlugin.hpp"
#include "M12_RawCAN.hpp"
#include "M12_RawCANPlugin.hpp"
#include "M12_Resolver.hpp"
#include "M12_ResolverPlugin.hpp"
#include "M12_WorkHydraulicPosition.hpp"
#include "M12_WorkHydraulicPositionPlugin.hpp"
#include "M12_WorkHydraulicPressures.hpp"
#include "M12_WorkHydraulicPressuresPlugin.hpp"





//void keyboard_interrupt_handler(int);

class M12_CAN2DDS {
private:
  int fd;
  TPCANStatus Status;
  unsigned int pcan_device;
  int dds_domain = 0;
public:
  M12_CAN2DDS(TPCANBaudrate);
  ~M12_CAN2DDS();
  void listen_and_publish();
  void print_stuff();
};

#endif
