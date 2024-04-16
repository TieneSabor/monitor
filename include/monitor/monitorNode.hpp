#ifndef _MONITOR_NODE__HPP_
#define _MONITOR_NODE__HPP_

#include <monitor/monitorThread.hpp>
#include <monitor/logger.hpp>
#include <signal.h>

// ros library
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"

namespace monitor{

typedef unsigned int hash_t;

class monitorNode{
public:
    monitorNode(std::string fileName);

    ~monitorNode();

    void killMonitors();

    void run();

    // functions that help testing manually
    void newAP(std::string AP);

    void newFunction(std::string LTLf);

private:
    // dealing with what AP is in this case
    std::vector<std::string> _APNames;
    std::vector<std::string> _LTLfs;
    std::vector<monitorThread*> _monitorPtrs;

    ret metaChecker(std::vector<std::string> metaFromMsg);
    void metaCB(const std_msgs::String::ConstPtr& msg);

    hash_t getHash();
    ret hashChecker(hash_t hash);
    void eventCB(const std_msgs::Int32MultiArray::ConstPtr& msg);

    std::string _fileName;
    ret inputParser();

    std::string _statMeta;
    ret updateStatMeta();

    logger _log;

    // ros stuff
    ros::NodeHandle _n;

    // mock events
    std::queue<std::vector<int>> _testQueue;
    void setUpTest();
};

};

#endif
