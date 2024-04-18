#ifndef _MONITOR_THREAD__HPP_
#define _MONITOR_THREAD__HPP_

#include <monitor/monitor.hpp>
#include <monitor/logger.hpp>
#include <queue>

// for c forking stuffs
#include <errno.h>
// #include <ext/stdio_filebuf.h>
#include <cstdio>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <unistd.h>

namespace monitor{
    enum control{
        RESTART, STOP,
    };

    enum feedback {
        DONE, VIOLATED, SLEEPING, 
    };

    class monitorThread{
    public:
        monitorThread(std::string formulaString, int monitorID);
        
        // ~monitorThread();

        ret setEventAPList(std::vector<std::string> APList);

        status getStatus();

        pid_t getPid();
        
        std::string getMonitorName();

        void run();

        ret newEvent(std::vector<int> newEvent);

        ret processEvent();

        ret stop();

        ret restart();

    private:
        monitorLTLf _moni;
        int _APListSize = 0;
        int _id = -1;
        status _monitorStatus = SLEEP;

        int _linkIn[2], _linkOut[2], _linkEv[2];
        pid_t _pid;
        // int _bufSize = 200;

        std::queue<std::vector<int>> _eventQueue;

        ret readPipe(int* link, signed char * buf, int size);

        ret writePipe(int* link, signed char * buf, int size);

        logger _log;
    };
}

#endif