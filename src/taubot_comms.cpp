#include "serial_comms/serial_comms.hpp"
 

int main()
{

  SerialComms comms;

  //char* uart_str[] = "/dev/ttyS0";
  std::string uart_str("/dev/ttyS0");

  comms.setup(uart_str); 

  if(!comms.connected())
  { 
    std::cout << "error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
    return 1;
  }

  comms.serialConnection();
  if(!comms.connected())
  {   
    printf("Serialc connection error");
    return 1;  
  } 

  int c = 0;

  while (true)
  {
    char msg[43] = {0};
    double num1[10] = {-155,    -2.3654,   -348.265,   -0.4125, -58.65844, -69.567890, -74,     -8.624,  -9,   0.021  } ;
    double num2[10] = {0.00145, 2235.2135, 324567890, 4,        0.5,       6,          7.12549, 0.08546, 9999, 0.00001};
    char num_str[22];

    comms.dtoa(num_str, num1[c], 22);
    strncat((char*) msg, num_str, sizeof(num_str)-1);
    comms.dtoa(num_str, num2[c], 22);
    strncat((char*) msg, num_str, sizeof(num_str)-1);
    msg[sizeof(msg)-1] = '\0';

    if(++c >= 10)
      c = 0;

    comms.sendMsg(msg, sizeof(msg));  
  //  sleep(1);
    std::cout << "Write " << msg << " to ttyS0." << std::endl;

  //  usleep(32000);

    if(comms.receiveMsg(85))
    {
        std::cout << "UART error, please check." << std::endl;
    }
    else
    {
      std::cout << std::endl << "cmdVelL = " << comms.getCmdVelL() << std::endl;
      std::cout << "distL    = " << comms.getDistL() << std::endl;
      std::cout << "cmdVelR = " << comms.getCmdVelR() << std::endl;
      std::cout << "distR    = " << comms.getDistR() << std::endl;
    }
  }  
 
  comms.closeComms();
  return 0;  
 }
                                  
