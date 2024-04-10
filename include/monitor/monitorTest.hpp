#ifndef _MONITOR_TEST__HPP_
#define _MONITOR_TEST__HPP_

#include <monitor/monitor.hpp>
#include <monitor/monitorThread.hpp>

using namespace monitor;

void monitor_test(){
    monitorLTLf myMonitor("a U b");
    std::vector<std::string> APList;
    APList.push_back("a");
    APList.push_back("b");
    APList.push_back("c");
    APList.push_back("d");
    myMonitor.setEventAPList(APList);
    myMonitor.print();
    // define some event
    std::vector<int> evA;
    evA.push_back(1);
    evA.push_back(0);
    evA.push_back(0);
    evA.push_back(0);
    std::vector<int> evB;
    evB.push_back(0);
    evB.push_back(1);
    evB.push_back(0);
    evB.push_back(0);
    std::vector<int> evEmp;
    evEmp.push_back(0);
    evEmp.push_back(0);
    evEmp.push_back(0);
    evEmp.push_back(0);
    std::vector<int> evUnk;
    evUnk.push_back(-1);
    evUnk.push_back(0);
    evUnk.push_back(0);
    evUnk.push_back(0);
    // a sequence that should stay undecided
    myMonitor.newEvent(evA);
    myMonitor.newEvent(evA);
    myMonitor.newEvent(evB);
    myMonitor.newEvent(evB);
    // a sequence that should put the system to sleep
    myMonitor.restart();
    myMonitor.newEvent(evB);
    myMonitor.newEvent(evA);
    myMonitor.newEvent(evUnk);
    myMonitor.newEvent(evA);
    // a sequence that should violate something
    myMonitor.restart();
    myMonitor.newEvent(evEmp);
    myMonitor.newEvent(evB);
    myMonitor.newEvent(evA);
    myMonitor.newEvent(evA);
    return;
}

void monitor_thread_test(){
    monitorThread myMT("a U b", 1);
    std::vector<std::string> APList;
    APList.push_back("a");
    APList.push_back("b");
    APList.push_back("c");
    APList.push_back("d");
    myMT.setEventAPList(APList);
    // define some event
    std::vector<int> evA;
    evA.push_back(1);
    evA.push_back(0);
    evA.push_back(0);
    evA.push_back(0);
    std::vector<int> evB;
    evB.push_back(0);
    evB.push_back(1);
    evB.push_back(0);
    evB.push_back(0);
    std::vector<int> evEmp;
    evEmp.push_back(0);
    evEmp.push_back(0);
    evEmp.push_back(0);
    evEmp.push_back(0);
    std::vector<int> evUnk;
    evUnk.push_back(-1);
    evUnk.push_back(0);
    evUnk.push_back(0);
    evUnk.push_back(0);
    // a sequence that should stay undecided
    myMT.run();
    myMT.newEvent(evA);
    myMT.newEvent(evA);
    myMT.newEvent(evB);
    myMT.newEvent(evB);
    ret retR = OK;
    do{
        retR = myMT.processEvent();
    } while(retR != QEMPTY);
    myMT.restart();
    // sleep
    myMT.newEvent(evA);
    myMT.newEvent(evUnk);
    myMT.newEvent(evUnk);
    myMT.newEvent(evB);
    retR = OK;
    do{
        retR = myMT.processEvent();
    } while(retR != QEMPTY);
    myMT.restart();
    // violate
    myMT.newEvent(evEmp);
    myMT.newEvent(evB);
    myMT.newEvent(evA);
    myMT.newEvent(evB);
    // flush and stop
    retR = OK;
    do{
        retR = myMT.processEvent();
        status st = myMT.getStatus();
        if (st == VIOLATE){
            break;
        }
    } while(retR != QEMPTY);
    myMT.stop();
    return;
}

#endif