#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main(int argc, char **argv)
{
    const char* port = "/dev/ttyACM0";  // default
    int baud = 1000000;
    int target_id = 1;

    if (argc > 1) port = argv[1];
    if (argc > 2) baud = std::stoi(argv[2]);
    if (argc > 3) target_id = std::stoi(argv[3]);

    std::cout << "serial: " << port << " @ " << baud << " baud" << std::endl;

    if(!sm_st.begin(baud, port)){
        std::cout<<"Failed to init sms/sts motor on "<< port <<"!"<<std::endl;
        return 0;
    }

    // Add delay after initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Attempting ping to ID " << target_id << "..." << std::endl;
    int ID = sm_st.Ping(target_id);
    std::cout << "Ping returned: " << ID << std::endl;

    if(ID!=-1){
        std::cout<<"ID:"<<ID<<std::endl;
    }else{
        std::cout<<"Ping servo ID error!"<<std::endl;
    }

    // Add delay before end
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sm_st.end();
    return 1;
}