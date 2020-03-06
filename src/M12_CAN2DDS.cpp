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
#include <math.h>

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



  //M12_Resolver resolver_sample;



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
        /*
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
        */
        int16_t acc_x = 0;
        int16_t acc_y = 0;
        int16_t acc_z = 0;
        memcpy(&acc_x, Message.DATA + 0,2);
        memcpy(&acc_y, Message.DATA + 2,2);
        memcpy(&acc_z, Message.DATA + 4,2);

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
        /*
        uint8_t byte_0 = Message.DATA[0];
        uint8_t byte_1 = Message.DATA[1];
        uint unit_motor_speed = uint (byte_0 << 8 | byte_1);
        double motor_speed = 10.0*double (unit_motor_speed);
        */
        uint16_t motor_speed = 0;
        memcpy(&motor_speed, Message.DATA + 0, 2);

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

      // Resolver
      case 0x19F:{
        std::cout << "Resolver message received..." << std::endl;
        /*
        int16_t bytes_0_1 = int16_t (Message.DATA[0] << 8 | Message.DATA[1]);
        double resolver_angle = static_cast<double>(bytes_0_1);
        resolver_angle = resolver_angle*0.01; // from 0.01 deg/unit  to 1 deg/unit
        resolver_angle = (resolver_angle/180.0)*M_PIl;

        int32_t bytes_4_5_6_7 = int32_t (Message.DATA[4] << 24 | Message.DATA[5] << 16 | Message.DATA[6] << 8 | Message.DATA[7]);
        double resolver_velocity = static_cast<double>(bytes_4_5_6_7);
        resolver_velocity = 0.01*resolver_velocity;
        resolver_velocity = (resolver_velocity/180.0)*M_PIl;

        M12_Resolver resolver_sample;
        resolver_sample.position(resolver_angle);
        resolver_sample.velocity(resolver_velocity);
        */
        int16_t resolver_angle = 0;
        memcpy(&resolver_angle, Message.DATA + 0, 2);

        int32_t resolver_velocity = 0;
        memcpy(&resolver_velocity, Message.DATA + 4, 4);

        M12_Resolver resolver_sample;
        resolver_sample.position(resolver_angle);
        resolver_sample.velocity(resolver_velocity);

        //std::cout << "Computed resolver angle in radians: " <<  resolver_angle <<  std::endl;
        //std::cout << "Computed resolver velocity in radians per second: " <<  resolver_velocity <<  std::endl;
        writer_M12_Resolver_topic.write(resolver_sample);
        break;
      }

      case 0x1B2:{
        std::cout << "IMU gyro message received..." << std::endl;
        /*
        int16_t angular_velocity_x = int16_t (Message.DATA[0] << 8 | Message.DATA[1]);
        int16_t angular_velocity_y = int16_t (Message.DATA[2] << 8 | Message.DATA[3]);
        int16_t angular_velocity_z = int16_t (Message.DATA[4] << 8 | Message.DATA[5]);
        int16_t temperature = int16_t (Message.DATA[6] << 8 | Message.DATA[7]);

        double angvx = static_cast<double>(angular_velocity_x);
        angvx = 0.003125*angvx;
        angvx = (angvx/180.0)*M_PIl;

        double angvy = static_cast<double>(angular_velocity_y);
        angvy = 0.003125*angvy;
        angvy = (angvy/180.0)*M_PIl;

        double angvz = static_cast<double>(angular_velocity_z);
        angvz = 0.003125*angvz;
        angvz = (angvz/180.0)*M_PIl;

        double chiptmp = static_cast<double>(temperature);
        chiptmp = 25.0 + 0.0678*chiptmp;

        M12_IMU_gyro imu_gyro_sample;
        */
        int16_t angvx = 0;
        memcpy(&angvx, Message.DATA + 0, 2);

        int16_t angvy = 0;
        memcpy(&angvy, Message.DATA + 2, 2);

        int16_t angvz = 0;
        memcpy(&angvz, Message.DATA + 4, 2);

        int16_t chiptmp = 0;
        memcpy(&chiptmp, Message.DATA + 6, 2);

        M12_IMU_gyro imu_gyro_sample;
        imu_gyro_sample.x_rate(angvx);
        imu_gyro_sample.y_rate(angvy);
        imu_gyro_sample.z_rate(angvz);

        imu_gyro_sample.chip_temperature(chiptmp);

        writer_M12_IMU_gyro_topic.write(imu_gyro_sample);
        break;
      }

      case 0x181:{
        std::cout << "Emergency stop message received..." << std::endl;
        std::bitset<4> buttons_bitset(Message.DATA[0]);
        bool is_normal_operation = buttons_bitset.test(0);
        bool emergency_stop_activated = buttons_bitset.test(1);
        bool door_switch_bypass_activated = buttons_bitset.test(2);
        bool dead_mans_switch_pressed = !buttons_bitset.test(3);

        M12_EMstop emstop_sample;
        emstop_sample.normal_operation(is_normal_operation);
        emstop_sample.emergency_stop_activated(emergency_stop_activated);
        emstop_sample.door_switch_bypass_activated(door_switch_bypass_activated);
        emstop_sample.dead_mans_switch_pressed(dead_mans_switch_pressed);

        writer_M12_EMstop_topic.write(emstop_sample);
        break;
      }

      case 0x186:{
        std::cout << "Work hydraulic actuator positions message received..." << std::endl;
        /*
        int16_t liftpos = int16_t(Message.DATA[0] << 8 | Message.DATA[1]);
        int16_t tiltpos = int16_t(Message.DATA[2] << 8 | Message.DATA[3]);
        */
        int16_t liftpos = 0;
        memcpy(&liftpos, Message.DATA + 0,2);

        int16_t tiltpos = 0;
        memcpy(&tiltpos, Message.DATA + 2,2);

        M12_WorkHydraulicPosition workpos_sample;
        workpos_sample.lift(liftpos);
        workpos_sample.tilt(tiltpos);
        writer_M12_WorkHydraulicPosition_topic.write(workpos_sample);
        break;
      }

      case 0x189:{
        std::cout << "Hydraulic pressures message received..." << std::endl;
        /*
        uint16_t pA_lift = uint16_t(Message.DATA[0] << 8 | Message.DATA[1]);
        uint16_t pB_lift = uint16_t(Message.DATA[2] << 8 | Message.DATA[3]);
        uint16_t pA_tilt = uint16_t(Message.DATA[4] << 8 | Message.DATA[5]);
        uint16_t pB_tilt = uint16_t(Message.DATA[6] << 8 | Message.DATA[7]);

        double lift_cylA_pressure = 0.01*static_cast<double>(pA_lift);
        double lift_cylB_pressure = 0.01*static_cast<double>(pB_lift);
        double tilt_cylA_pressure = 0.01*static_cast<double>(pA_tilt);
        double tilt_cylB_pressure = 0.01*static_cast<double>(pB_tilt);
        */
        uint16_t lift_cylA_pressure = 0;
        memcpy(&lift_cylA_pressure, Message.DATA + 0, 2);

        uint16_t lift_cylB_pressure = 0;
        memcpy(&lift_cylB_pressure, Message.DATA + 2, 2);

        uint16_t tilt_cylA_pressure = 0;
        memcpy(&tilt_cylA_pressure, Message.DATA + 4, 2);

        uint16_t tilt_cylB_pressure = 0;
        memcpy(&tilt_cylB_pressure, Message.DATA + 6, 2);

        M12_WorkHydraulicPressures workpres_sample;
        workpres_sample.lift_cylA_pressure(lift_cylA_pressure);
        workpres_sample.lift_cylB_pressure(lift_cylB_pressure);
        workpres_sample.tilt_cylA_pressure(tilt_cylA_pressure);
        workpres_sample.tilt_cylB_pressure(tilt_cylB_pressure);

        writer_M12_WorkHydraulicPressures_topic.write(workpres_sample);
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
