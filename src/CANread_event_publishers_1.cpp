#include <iostream>
#include "M12_CAN2DDS.h"
//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  std::cout << "Starting the test program..." << std::endl;
  std::cout << "C++ version: " << __cplusplus << std::endl;

  try{
    M12_CAN2DDS my_object(PCAN_BAUD_500K);


    my_object.print_stuff();
    my_object.listen_and_publish();
  }
  catch(const std::logic_error e){
    std::cerr << "ERROR: "<< e.what() << std::endl;
    //std::cerr << std::endl << "exiting..." << std::endl;

    return 1;
  }
  std::cout << "Exiting the test program..." << std::endl;
  return 0;
}
