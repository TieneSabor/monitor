#ifndef _MONITOR__HPP_
#define _MONITOR__HPP_

#include <monitor/logger.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

// spot headers
#include <spot/tl/parse.hh>
#include <spot/tl/ltlf.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/remprop.hh>
#include <spot/twaalgos/powerset.hh>
#include <spot/twaalgos/isdet.hh>
#include <spot/tl/apcollect.hh>
#include <spot/twa/bddprint.hh>

namespace monitor
{
    enum ret{
        OK, ERR, QEMPTY, 
    };

    enum status{
        UNDECIDE, VIOLATE, SLEEP
    };

    class monitorLTLf{
    public:
        monitorLTLf(std::string LTLfFormulaString);

        // ~monitor();

        ret restart();

        // a list of all APs that a single event contains
        ret setEventAPList(std::vector<std::string> APList);

        bool isAPListSet(){ return _setEvAps; }

        int getAPListSize(){  return _eventAPList.size();  }

        status getStatus(){  return _monitorStatus;  }

        // a list of true (1), false (0) or unknown (-1) following the order of _APList.
        ret newEvent(std::vector<int> newEvent);

        void print();

    private:
        status _monitorStatus;
        unsigned _curStateId;

        std::string _formulaString;
        spot::parsed_formula _pf;
        spot::twa_graph_ptr _daut;

        // list of APs of an incoming event
        bool _setEvAps;
        std::vector<std::string> _eventAPList;
        std::map<std::string, int> _APName2EventID;

        std::map<std::string, bdd> _APName2BDD;
        void updateAPName2BDD();

        std::vector<std::string> _APNames;
        void updateAPNames();

        logger _log;
    };
} // namespace monitor


#endif