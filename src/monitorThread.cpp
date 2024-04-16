#include <monitor/monitorThread.hpp>

using namespace monitor;

monitorThread::monitorThread(std::string formulaString, int monitorID) : _moni(formulaString), _id(monitorID), _log(3, formulaString + " Manager") {
    return;
}

ret monitorThread::setEventAPList(std::vector<std::string> APList){
    return _moni.setEventAPList(APList);
}

status monitorThread::getStatus(){
    return _monitorStatus;
}

pid_t monitorThread::getPid(){
    return _pid;
}

std::string monitorThread::getMonitorName(){
    return _moni.getMonitorName();
}

void monitorThread::run(){
    if (!_moni.isAPListSet()){
        _log.erro("Event AP list not set.  Monitor will not run.  ");
        return;
    }
    // build the pipes
    _linkIn[1] = memfd_create(("control_" + std::to_string(_id)).c_str(), 0);
    _linkEv[1] = memfd_create(("event_" + std::to_string(_id)).c_str(), 0);
    _linkOut[1] = memfd_create(("feedback_" + std::to_string(_id)).c_str(), 0);
    if (((pipe(_linkIn) == -1)) || (pipe(_linkEv) == -1) || (pipe(_linkOut) == -1)){
        _log.erro("Pipe creation failed. ");
        return;
    }

    // fork
    int _bufSize = _moni.getAPListSize();
    signed char _bufEv[_bufSize + 1];
    signed char _bufIn[2]; // we need one slot for control
    signed char _bufOut[2]; // we need one slot for feedback
    _pid = fork();
    if (_pid == 0){
        // This is the child
        // iteratively check for event
        bool running = true;
        while (running){
            // return feedback
            status fb = _moni.getStatus();
            switch(fb){
                case UNDECIDE:
                    _bufOut[0] = (signed char) DONE;
                    break;
                case VIOLATE:
                    _bufOut[0] = (signed char) VIOLATED;
                    break;
                case SLEEP:
                    _bufOut[0] = (signed char) SLEEPING;
                    break;
                default:
                    _log.erro("Unknown monitor status: %d", (int) fb);
                    break;
            }
            ret retOut = writePipe(_linkOut, _bufOut, 1);
            if (retOut == ERR){
                _log.erro("Feedback failed.  ");
            }
            // read from control
            ret retCon = readPipe(_linkIn, _bufIn, 1);
            if (retCon == OK){
                switch ((control) _bufIn[0]){
                    case RESTART:
                        _log.prnt("Monitor restart");
                        _moni.restart();
                        break;
                    case STOP:
                        running = false;
                        break;
                    default:
                        _log.erro("Wrong control code: %d", _bufIn[0]);
                        break;
                }
            }
            // read event
            ret retEv = readPipe(_linkEv, _bufEv, _bufSize);
            if (retEv == OK){
                // translate input to vector
                std::vector<int> newEvent;
                for (int i=0; i<_bufSize; i++){
                    newEvent.push_back((int) _bufEv[i]);
                }
                // put that into monitor
                _moni.newEvent(newEvent);
            }
        }
        _log.prnt("Monitor turns off");
        exit(0);
    } else {
        return;
    }
    // never goes here
    return;
}

ret monitorThread::newEvent(std::vector<int> newEvent){
    if (newEvent.size() != _moni.getAPListSize()){
        _log.erro("Wrong event size: %d", newEvent.size());
        return ERR;
    }
    // push this event to the queue
    _eventQueue.push(newEvent);
    // if monitor is done, pop one from queue and send it
    return processEvent();
}

ret monitorThread::processEvent(){
    // check if the queue is empty
    if (_eventQueue.size() == 0){
        return QEMPTY;
    }
    // check if we have feedback from the monitor
    signed char _bufOut[2]; // we need one slot for feedback
    ret retR = readPipe(_linkOut, _bufOut, 1);
    if (retR == ERR){
        _log.debg("Monitor not ready");
        // not ready
        return OK;
    } else {
        feedback fb = (feedback) _bufOut[0];
        // if ((fb == DONE) || (fb == VIOLATED) || (fb == SLEEPING)){
        if ((fb == DONE) || (fb == SLEEPING)){
            // pop one and send
            std::vector<int> event = _eventQueue.front();
            signed char _bufEv[_moni.getAPListSize() + 1];
            for (int i=0; i<event.size(); i++){
                _bufEv[i] = (signed char) event[i];
            }
            ret retW = writePipe(_linkEv, _bufEv, _moni.getAPListSize());
            if (retW == ERR){
                return ERR;
            }
            _eventQueue.pop();
            _log.debg("Event sent");
            // maintain status
            if (fb == SLEEPING){
                _monitorStatus = SLEEP;
            } else if (fb == DONE) {
                _monitorStatus = UNDECIDE;
            }
        } else if (fb == VIOLATED) {
            _log.debg("Monitor is violated.  Waiting for restart. ");
            _monitorStatus = VIOLATE;
        } else {
            _log.erro("Unknown monitor status: %d", _bufOut[0]);
        }
    }
    return OK;
}

ret monitorThread::stop(){
    // wait until monitor is ready before stop it
    signed char _bufOut[2]; // we need one slot for feedback
    ret retR = ERR;
    do {
        retR = readPipe(_linkOut, _bufOut, 1);
    } while (retR != OK);
    // stop it
    signed char _bufIn[2];
    _bufIn[0] = (signed char) STOP;
    ret retW = writePipe(_linkIn, _bufIn, 1);
    if (retW == ERR){
        return ERR;
    }
    return OK;
}

ret monitorThread::restart(){
    // wait until monitor is ready before stop it
    signed char _bufOut[2]; // we need one slot for feedback
    ret retR = ERR;
    do {
        retR = readPipe(_linkOut, _bufOut, 1);
    } while (retR != OK);
    // restart it
    signed char _bufIn[2];
    _bufIn[0] = (signed char) RESTART;
    ret retW = writePipe(_linkIn, _bufIn, 1);
    if (retW == ERR){
        return ERR;
    }
    return OK;
}

ret monitorThread::readPipe(int* link, signed char * buf, int size){
    // check if the file is there
    fd_set set;
    FD_ZERO(&set); /* clear the set */
    FD_SET(link[0], &set); /* add our file descriptor to the set */
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;
    int rv = select(link[0] + 1, &set, NULL, NULL, &timeout);
    if(rv == -1){
        _log.erro("Read pipe select() error.  ");
        return ERR;
    } else if(rv == 0){
        _log.debg("Read pipe time out");
        return ERR;
    }

    // prepare for read
    buf[size] = '\0';
    int retr = 0;
    // read once
    retr = read(link[0], buf, size);
    // the read character number should be equal to ap list size
    if (retr != size){
        _log.erro("Read (via pipe) size is wrong: %d", retr);
        return ERR;
    } else if (retr == 0){
        return ERR;
    } else {
        return OK;
    }
}

ret monitorThread::writePipe(int* link, signed char * buf, int size){
    buf[size] = '\0';
    int retw = 0;
    // write once
    retw = write(link[1], buf, size);
    // the read character number should be equal to ap list size
    if (retw != size){
        _log.erro("Write (via pipe) size is wrong: %d", retw);
        return ERR;
    } else if (retw == 0){
        _log.erro("Write pipe failed.  ");
        return ERR;
    } else {
        return OK;
    }
}