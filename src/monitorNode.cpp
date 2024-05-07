// monitor codes
#include <monitor/monitorNode.hpp>

using namespace monitor;

monitorNode::monitorNode(std::string fileName) : _log(3, "Main"){
    _fileName = fileName;
    ret inRet = inputParser();
    if (inRet != OK){
        _log.erro("Input file parsing failed: %s", fileName.c_str());
    }
    // for test purpose
    setUpTest();
}

monitorNode::~monitorNode(){
    // stop all monitors
    for (int i = 0; i<_monitorPtrs.size(); i++){
        ret retR = OK;
        do{
            retR = _monitorPtrs[i]->processEvent();
            if (_monitorPtrs[i]->getStatus() == VIOLATE){
                break;
            }
        } while(retR != QEMPTY);
        _monitorPtrs[i]->stop();
        // delete
        delete _monitorPtrs[i];
    }
    // free all aps and ltlfs
    for (int i=0; i<_APNames.size(); i++){
        delete &_APNames[i];
    }
    for (int i=0; i<_LTLfs.size(); i++){
        delete &_LTLfs[i];
    }
    return;
}

void monitorNode::killMonitors(){
    for (int i = 0; i<_monitorPtrs.size(); i++){
        pid_t pid = _monitorPtrs[i]->getPid();
        kill(pid, SIGKILL);
    }
}

void monitorNode::run(){
    // registor all monitors
    for (int i=0; i < _LTLfs.size(); i++){
        monitorThread* mtPtr = new monitorThread(_LTLfs[i], i);
        mtPtr->setEventAPList(_APNames);
        // run all monitors
        mtPtr->run();
        _monitorPtrs.push_back(mtPtr);
    }
    // start ros stuffs
    ros::Subscriber metaSub  = _n.subscribe("monitor/meta",  1000, &monitorNode::metaCB,  this);
    ros::Subscriber eventSub = _n.subscribe("monitor/event", 1000, &monitorNode::eventCB, this);
    ros::Publisher eventPub = _n.advertise<std_msgs::Int32MultiArray>("monitor/event", 1000);
    // status & status meta publisher
    ros::Publisher statMetaPub = _n.advertise<std_msgs::String>("monitor/statMeta",1000);
    ros::Publisher statusPub   = _n.advertise<std_msgs::Int32MultiArray>("monitor/status",1000);
    ros::Rate loop_rate(100);
    // status meta published in a much lower freq.
    int statMetaPeriod = 100;
    int statMetaCounter = 0;
    if (updateStatMeta() != OK){
        _log.erro("Status Meta Error.");
    }
    // start expo
    _exp = std::chrono::high_resolution_clock::now();
    // the while loop
    while(true){
        // process event
        // TODO: restart monitors if violated
        for (int i = 0; i<_monitorPtrs.size(); i++){
            if (_monitorPtrs[i]->processEvent() == QEMPTY){
                _log.debg("Monitor %d has empty event queue. ");
            }
        }
        // publish mock events
        if (_testQueue.size()>0){
            std_msgs::Int32MultiArray msg;
            msg.data = _testQueue.front();
            // insert fake hash
            msg.data.insert(msg.data.begin(), getHash());
            _testQueue.pop();
            eventPub.publish(msg);
        }
        // publish monitor status
        if (statMetaCounter >= statMetaPeriod){
            std_msgs::String msg;
            msg.data = _statMeta;
            statMetaPub.publish(msg);
            statMetaCounter = 0;
        }
        statMetaCounter ++;
        // TODO: check the status is correct
        std_msgs::Int32MultiArray msg;
        msg.data.push_back(getHash());
        for (int i=0; i<_monitorPtrs.size(); i++){
            int stat = 0;
            status monStat = _monitorPtrs[i]->getStatus();
            // can just transfer enum to int tho
            if (monStat == UNDECIDE){
                stat = 0;
            } else if (monStat == VIOLATE){
                stat = 1;
            } else if (monStat == SLEEP){
                stat = 2;
            } else {
                _log.warn("Unknown Monitor Status...");
            }
            msg.data.push_back(stat);
        }
        statusPub.publish(msg);
        // spin
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void monitorNode::newAP(std::string AP){
    std::string* ap = new std::string(AP);
    _APNames.push_back(* ap);
    _APNames2EventID[* ap] = _evNum;
    _evNum ++;
}

void monitorNode::newFunction(std::string LTLf){
    std::string* ltlf = new std::string(LTLf);
    _LTLfs.push_back(* ltlf);
}

void monitorNode::newTimer(std::string timerAP, float timeBound){
    std::string* ap = new std::string(timerAP);
    _APNames.push_back(* ap);
    _timerBoundMap[* ap] = timeBound;
}

ret monitorNode::inputParser(){
    return OK;
}

ret monitorNode::metaChecker(std::vector<std::string> metaFromMsg){
    if (metaFromMsg.size() != _APNames.size()){
        _log.erro("Meta from ROS message (%d) has different size than from file (%d). ", metaFromMsg.size(), _APNames.size());
        return ERR;
    }
    for (int i = 0; i<_APNames.size(); i++){
        if (0 != _APNames[i].compare(metaFromMsg[i])){
            _log.erro("Meta from ROS message has different APs or they are in different order than from file.  ");
            return ERR;
        }
    }
    return OK;
}

void monitorNode::metaCB(const std_msgs::String::ConstPtr& msg){
    if (msg->data.size() <= 0){
        return;
    }
    // parse meta
    std::vector<std::string> metaFromMsg;
    std::string delimiter = ";";
    size_t pos = 0, lastPos = 0;
    while ((pos = msg->data.find(delimiter, lastPos)) != std::string::npos) {
        std::string token = msg->data.substr(lastPos, pos);
        metaFromMsg.push_back(token);
        // msg->data.erase(0, pos + delimiter.length());
        lastPos = pos;
    }
    // check meta
    if (ERR == metaChecker(metaFromMsg)){
        _log.erro("Wrong meta:");
        _log.erro("    " + msg->data);
        return;
    }
    // TODO: do we want a flag s.t. we only receive events after meta?
}

hash_t monitorNode::getHash(){
    return 2000;
}

ret monitorNode::hashChecker(hash_t hash){
    return OK;
}

void monitorNode::eventCB(const std_msgs::Int32MultiArray::ConstPtr& msg){
    if (msg->data.size() <= 0){
        return;
    }
    // check hash
    hash_t hash = (hash_t) msg->data[0];
    if (ERR == hashChecker(hash)){
        _log.erro("Event hash is wrong: %d  ", (unsigned int) hash);
        return;
    }
    // distribute event to each monitor
    std::vector<int> event;
    // event.insert(event.begin(), ++msg->data.begin(), msg->data.end());
    for (int i=0; i<_APNames.size(); i++){
        if (_APNames2EventID.find(_APNames[i])!=_APNames2EventID.end()){
            int eid = _APNames2EventID[_APNames[i]];
            event.push_back(msg->data[eid]);
        } else {
            // create timer
            auto elapsed = std::chrono::high_resolution_clock::now() - _exp;
            long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            if (_timerBoundMap[_APNames[i]]*1000000 > microseconds){
                // have not expired
                event.push_back(0);
                _log.debg("Timer Not Expired");
            } else {
                event.push_back(1);
                _log.debg("Timer Expired");
            }
        }
    }
    for (int i = 0; i<_monitorPtrs.size(); i++){
        _monitorPtrs[i]->newEvent(event);
    }
}

ret monitorNode::updateStatMeta(){
    _statMeta.clear();
    for (int i=0; i<_monitorPtrs.size(); i++){
        if (i != 0){
            _statMeta += ";";
        }
        _statMeta += _monitorPtrs[i]->getMonitorName();
    }
    return OK;
}

void monitorNode::setUpTest(){
    newTimer("t1", 30.0);
    newTimer("t2", 30.0);
    newAP("p1");
    newAP("p2");
    newAP("p3");
    newAP("p4");
    newAP("p5");
    newAP("p6");
    newAP("p7");
    newAP("p8");
    newAP("p9");
    newFunction("G p1");
    newFunction("G p2");
    newFunction("G p3");
    newFunction("G p4");
    newFunction("(! p6) U p5");
    newFunction("(! t1) U p5");
    newFunction("(! t2) U p6");
    // Test
    // newAP("b");
    // newAP("c");
    // newAP("d");
    // newFunction("a U b");
    // newFunction("G a");
    // std::vector<int>* evA = new std::vector<int>({1,0,0,0});
    // std::vector<int>* evB = new std::vector<int>({0,1,0,0});
    // std::vector<int>* evEmp = new std::vector<int>({0,0,0,0});
    // std::vector<int>* evUnk = new std::vector<int>({-1,0,0,0});
    // healthy
    // _testQueue.push(* evA);
    // _testQueue.push(* evA);
    // _testQueue.push(* evB);
    // _testQueue.push(* evB);
    // sleep
    // _testQueue.push(* evA);
    // _testQueue.push(* evUnk);
    // _testQueue.push(* evUnk);
    // _testQueue.push(* evB);
    // violate
    // _testQueue.push(* evEmp);
    // _testQueue.push(* evB);
    // _testQueue.push(* evA);
    // _testQueue.push(* evB);
}
