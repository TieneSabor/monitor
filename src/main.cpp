#include <monitor/monitorNode.hpp>
#include <monitor/monitorTest.hpp>

// for crtl c event
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

monitorNode* mNPtr;

void termHandler(int s){
    std::cout << "Terminate the program" << std::endl;
    mNPtr->killMonitors();
    // delete mNPtr;
    exit(1); 

}

int main(int argc, char **argv){
    ros::init(argc, argv, "runtimeVerify");
    // monitor_thread_test();
    mNPtr = new monitorNode("");
    // set up handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = termHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    // run the node
    try {
        mNPtr->run();
    } catch (...){
        termHandler(-1);
    }
    return 0;
}