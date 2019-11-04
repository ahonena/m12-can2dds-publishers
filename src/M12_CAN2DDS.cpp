#include "M12_CAN2DDS.h"
#include <unistd.h>
#include <sys/mman.h>
#include <dds/domain/ddsdomain.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>


static volatile sig_atomic_t stop = 0;

void keyboard_interrupt_handler(int sig){
  stop = 1;
}

//------------------------------------------------------------------------------
M12_CAN2DDS::M12_CAN2DDS(TPCANBaudrate CAN_bitrate){
  //CAN LEVEL INITIALIZATION ---------------------------------------------------
  unsigned int rx_count = 0;
  pcan_device = PCAN_PCIBUS1;
  //mlockall(MCL_CURRENT | MCL_FUTURE);
  Status = CAN_Initialize(pcan_device, CAN_bitrate, 0, 0, 0);
  //Status = 17;
  if (Status){
    std::stringstream Status_stream;
    Status_stream << std::hex << Status;
    auto errormsg_ = "Could not initialize PEAK CAN device, status code: 0x" + Status_stream.str();
    throw std::logic_error(errormsg_);
  }

  //int fd;
  Status = CAN_GetValue(pcan_device, PCAN_RECEIVE_EVENT, &fd, sizeof fd);
  //Status = 100;
  if (Status){
    CAN_Uninitialize(pcan_device);
    std::stringstream Status_stream;
    Status_stream << std::hex << Status;
    auto errormsg_ = "Could not retrieve information about PEAK CAN device, status code: 0x" + Status_stream.str();
    throw std::logic_error(errormsg_);
  }


  //DDS LEVEL INITIALIZATION ---------------------------------------------------
  dds::domain::DomainParticipant participant_m12(dds_domain);
  dds::pub::Publisher publisher_m12(participant_m12);

  // Implemented types (4.11.2019)

  //M12_IMU_acceleration
  //M12_input_and_hsd
  //M12_Resolver
  //M12_IMU_gyro
  //M12_RawCAN
  //M12_EMstop
  //M12_WorkHydraulicPosition
  //M12_WorkHydraulicPressures

  dds::topic::Topic<M12_IMU_acceleration> IMU_acceleration_topic(participant_m12, "IMU_acceleration");
  dds::topic::Topic<M12_input_and_hsd> M12_input_and_hsd_topic(participant_m12, "Driver_input");
  dds::topic::Topic<M12_Resolver> M12_Resolver_topic(participant_m12, "Resolver");
  dds::topic::Topic<M12_IMU_gyro> M12_IMU_gyro_topic(participant_m12, "IMU_gyro");
  dds::topic::Topic<M12_RawCAN> M12_RawCAN_topic(participant_m12, "RawCAN");
  dds::topic::Topic<M12_EMstop> M12_EMstop_topic(participant_m12, "EMstop_status");
  dds::topic::Topic<M12_WorkHydraulicPosition> M12_WorkHydraulicPosition_topic(participant_m12, "WorkHydraulicPositions");
  dds::topic::Topic<M12_WorkHydraulicPressures> M12_WorkHydraulicPressures(participant_m12, "WorkHydraulicPressures");


}
//------------------------------------------------------------------------------
M12_CAN2DDS::~M12_CAN2DDS(){
  std::cout << "Destructor called..." << std::endl;

}




//------------------------------------------------------------------------------
void M12_CAN2DDS::listen_and_publish(){
  signal(SIGINT, keyboard_interrupt_handler);
  std::cout << "Listening to CAN bus channel 1, PCAN code: " << pcan_device << std::endl;


  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd, &fds);
  mlockall(MCL_CURRENT | MCL_FUTURE);
  while (!stop) {

    // blocks on read descriptor
    int err = select(fd+1, &fds, NULL, NULL, NULL);
    if (err != 1 || !FD_ISSET(fd, &fds)) {
      //printf("select(%xh) failure: %d\n", pcan_device, err);
      //std::cout << "FD_ISSET(fd, &fds): " << FD_ISSET(fd, &fds) << std::endl;
      auto errormsg_ = "System call \"select\" failure, error code: " + std::to_string(err);
      std::cerr << errormsg_ << std::endl;
      continue;
      //throw std::logic_error(errormsg_);
    }

    TPCANMsg Message;
    Status = CAN_Read(pcan_device, &Message, NULL);

    if (Status == PCAN_ERROR_QRCVEMPTY) {
      std::cerr << "CAN read error: PCAN_ERROR_QRCVEMPTY" << std::endl;

      if (usleep(1000000)) {
        auto errormsg_ = "System call \"usleep\" failure";
        throw std::logic_error(errormsg_);
      }
      continue;
    }

    if (Status != PCAN_ERROR_OK) {
      std::stringstream Status_stream;
      Status_stream << std::hex << Status;
      auto errormsg_ = "CAN read error, status code: " + Status_stream.str();
      throw std::logic_error(errormsg_);
    }

    // emergency exit...
    //if (Message.ID == 0xff && Message.LEN == 1 && Message.DATA[0] == 0xff)
    //	break;

    //rx_count++;
    //nterpret_message(Message);
    switch(Message.ID){
      case 0x189:{
        std::cout << "Hydraulic pressures message received..." << std::endl;
        break;
      }

      default:{
        std::cout << "Raw CAN message received..." << std::endl;
        break;
      }
    }
    printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
    (int) Message.ID, (int) Message.LEN, (int) Message.DATA[0],
    (int) Message.DATA[1], (int) Message.DATA[2],
    (int) Message.DATA[3], (int) Message.DATA[4],
    (int) Message.DATA[5], (int) Message.DATA[6],
    (int) Message.DATA[7]);


  }







}
//------------------------------------------------------------------------------
void M12_CAN2DDS::print_stuff(){
  std::cout << "PEAK CAN status value is: 0x" << std::hex << Status << std::endl;
  std::cout << "The file descriptor is: " << fd << std::endl;
}
