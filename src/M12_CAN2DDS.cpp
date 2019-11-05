#include "M12_CAN2DDS.h"
#include <unistd.h>
#include <sys/mman.h>
#include <dds/domain/ddsdomain.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/core/ListenerBinder.hpp>

#include <bitset>

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



}
//------------------------------------------------------------------------------
M12_CAN2DDS::~M12_CAN2DDS(){
  std::cout << "Destructor called..." << std::endl;

}




//------------------------------------------------------------------------------
void M12_CAN2DDS::listen_and_publish(){



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



  dds::topic::Topic<M12_IMU_acceleration> M12_IMU_acceleration_topic(participant_m12, "IMU_acceleration");
  dds::topic::Topic<M12_input_and_hsd> M12_input_and_hsd_topic(participant_m12, "Driver_input");
  dds::topic::Topic<M12_Resolver> M12_Resolver_topic(participant_m12, "Resolver");
  dds::topic::Topic<M12_IMU_gyro> M12_IMU_gyro_topic(participant_m12, "IMU_gyro");
  dds::topic::Topic<M12_RawCAN> M12_RawCAN_topic(participant_m12, "RawCAN");
  dds::topic::Topic<M12_EMstop> M12_EMstop_topic(participant_m12, "EMstop_status");
  dds::topic::Topic<M12_WorkHydraulicPosition> M12_WorkHydraulicPosition_topic(participant_m12, "WorkHydraulicPositions");
  dds::topic::Topic<M12_WorkHydraulicPressures> M12_WorkHydraulicPressures_topic(participant_m12, "WorkHydraulicPressures");

  dds::pub::Publisher publisher_M12(participant_m12);

  dds::pub::DataWriter<M12_IMU_acceleration> writer_IMU_acceleration_topic(publisher_M12, M12_IMU_acceleration_topic);
  dds::pub::DataWriter<M12_input_and_hsd> writer_M12_input_and_hsd_topic(publisher_M12, M12_input_and_hsd_topic);
  dds::pub::DataWriter<M12_Resolver> writer_M12_Resolver_topic(publisher_M12, M12_Resolver_topic);
  dds::pub::DataWriter<M12_IMU_gyro> writer_M12_IMU_gyro_topic(publisher_M12, M12_IMU_gyro_topic);
  dds::pub::DataWriter<M12_RawCAN> writer_M12_RawCAN_topic(publisher_M12, M12_RawCAN_topic);
  dds::pub::DataWriter<M12_EMstop> writer_M12_EMstop_topic(publisher_M12, M12_EMstop_topic);
  dds::pub::DataWriter<M12_WorkHydraulicPosition> writer_M12_WorkHydraulicPosition_topic(publisher_M12, M12_WorkHydraulicPosition_topic);
  dds::pub::DataWriter<M12_WorkHydraulicPressures> writer_M12_WorkHydraulicPressures_topic(publisher_M12, M12_WorkHydraulicPressures_topic);



  M12_Resolver resolver_sample;
  M12_IMU_gyro imu_gyro_sample;

  M12_EMstop emstop_sample;
  M12_WorkHydraulicPosition workpos_sample;
  M12_WorkHydraulicPressures workpres_sample;

  //std::cout << "RawCAN sample, COBID: " << rawcan_sample.COBID() << std::endl;

  signal(SIGINT, keyboard_interrupt_handler);
  std::cout << "Listening to PCAN-PCI channel 1 (first from the right), PCAN code: " << pcan_device << std::endl;


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
      auto errormsg_ = "System call \"select\" failure. Exiting program...";
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

    // CAN MESSAGE INTERPRETATION-----------------------------------------------
    // -------------------------------------------------------------------------
    switch(Message.ID){

      // IMU acceleration
      case 0x2B2:{
        double g_ = 9.82; // m/s² per 1 g
        double g_per_unit = 0.25*0.001;// From ADIS 16385 documentation
        double acc_per_unit = g_*g_per_unit;

        uint8_t byte_0 = Message.DATA[0];
        uint8_t byte_1 = Message.DATA[1];
        int unit_acceleration_x = 0;
        unit_acceleration_x = int (byte_0 << 8 | byte_1);

        uint8_t byte_2 = Message.DATA[2];
        uint8_t byte_3 = Message.DATA[3];
        int unit_acceleration_y = 0;
        unit_acceleration_y = int (byte_2 << 8 | byte_3);

        uint8_t byte_4 = Message.DATA[4];
        uint8_t byte_5 = Message.DATA[5];
        int unit_acceleration_z = 0;
        unit_acceleration_z = int (byte_4 << 8 | byte_5);

        double acc_x = double (unit_acceleration_x)*acc_per_unit;
        double acc_y = double (unit_acceleration_y)*acc_per_unit;
        double acc_z = double (unit_acceleration_z)*acc_per_unit;

        M12_IMU_acceleration imu_acc_sample;
        imu_acc_sample.x_acceleration(acc_x);
        imu_acc_sample.y_acceleration(acc_y);
        imu_acc_sample.z_acceleration(acc_z);

        std::array<uint8_t,2> diagnostic_state = {Message.DATA[6],Message.DATA[7]};
        imu_acc_sample.diagnostic_state(diagnostic_state);

        writer_IMU_acceleration_topic.write(imu_acc_sample);
        break;
      }

      // Input and HSD
      case 0x485:{

        uint8_t byte_0 = Message.DATA[0];
        uint8_t byte_1 = Message.DATA[1];
        uint unit_motor_speed = uint (byte_0 << 8 | byte_1);
        double motor_speed = 10.0*double (unit_motor_speed);

        hsd_direction_enum motor_direction;
        uint8_t byte_2 = Message.DATA[2];
        if(byte_2 == 1){
          motor_direction = hsd_direction_enum::FORWARD;
        }
        else if(byte_2 == 2){
          motor_direction = hsd_direction_enum::BACKWARD;
        }
        else{
          motor_direction = hsd_direction_enum::UNKNOWN;
        }

        //std::array<bool,4> buttons;

        uint8_t byte_5 = Message.DATA[5];
        std::bitset<8> buttons_bitset(byte_5);
        std::array<bool,4> buttons;
        buttons.at(0) = bool(buttons_bitset.test(0));
        buttons.at(1) = bool(buttons_bitset.test(1));
        buttons.at(2) = bool(buttons_bitset.test(2));
        buttons.at(3) = bool(buttons_bitset.test(3));

        int8_t joystick_x = Message.DATA[3];
        int8_t joystick_y = Message.DATA[4];

        uint8_t gas_pedal = Message.DATA[6];

        M12_input_and_hsd input_sample;
        input_sample.gas_pedal(gas_pedal);
        input_sample.joystick_x(joystick_x);
        input_sample.joystick_y(joystick_y);
        input_sample.buttons(buttons);
        input_sample.hsd_motor_speed(motor_speed);
        input_sample.hsd_direction(motor_direction);

        writer_M12_input_and_hsd_topic.write(input_sample);
        break;
      }

      case 0x189:{
        std::cout << "Hydraulic pressures message received..." << std::endl;
        break;
      }

      default:{
        std::cout << "Uninterpreted CAN message received..." << std::endl;

        dds::core::array<uint8_t, 8> tmp_array;
        for(int i = 0; i<8; i++){
          tmp_array.at(i) = Message.DATA[i];
          //std::cout << (int)tmp_array.at(i) << std::endl;
        }
        M12_RawCAN rawcan_sample;
        rawcan_sample.COBID((int32_t) Message.ID);
        rawcan_sample.data(tmp_array);
        writer_M12_RawCAN_topic.write(rawcan_sample);
        break;
      }
    }
    /*
    printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
    (int) Message.ID, (int) Message.LEN, (int) Message.DATA[0],
    (int) Message.DATA[1], (int) Message.DATA[2],
    (int) Message.DATA[3], (int) Message.DATA[4],
    (int) Message.DATA[5], (int) Message.DATA[6],
    (int) Message.DATA[7]);
    */

  }







}
//------------------------------------------------------------------------------
void M12_CAN2DDS::print_stuff(){
  std::cout << "PEAK CAN status value is: 0x" << std::hex << Status << std::endl;
  std::cout << "The file descriptor is: " << fd << std::endl;
}
