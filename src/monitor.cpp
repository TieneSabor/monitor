#include <monitor/monitor.hpp>

using namespace monitor;

monitorLTLf::monitorLTLf(std::string LTLfFormulaString): _log(3, LTLfFormulaString){
    _pf = spot::parse_infix_psl(LTLfFormulaString);
    if (_pf.format_errors(std::cerr)){
        return;
    }
    _formulaString = LTLfFormulaString;

    // build the DFA
    spot::translator trans;
    trans.set_type(spot::postprocessor::Buchi);
    trans.set_pref(spot::postprocessor::SBAcc
                    | spot::postprocessor::Small);
    spot::twa_graph_ptr aut = trans.run(spot::from_ltlf(_pf.f));
    _daut = spot::to_finite(aut);
    _daut = spot::tba_determinize_check(_daut, 0, 0, _pf.f);
    if (!spot::is_deterministic(_daut)){
        _log.erro("Determinize failed");
        return;
    }
    // maintain AP list kand APP to BDD map
    updateAPName2BDD();
    updateAPNames();
    // initialize the state
    _curStateId = _daut->get_init_state_number();
    _monitorStatus = SLEEP;
}

ret monitorLTLf::restart(){
    if(_monitorStatus == VIOLATE){
        _log.warn("Restart a violated monitor");
    }
    _curStateId = _daut->get_init_state_number();
    _monitorStatus = SLEEP;
    return OK;
}

ret monitorLTLf::setEventAPList(std::vector<std::string> APList){
    if (APList.size() == 0){
        _log.erro("AP List for event input is empty.  Please check again.  ");
        return ERR;
    }
    _eventAPList = APList;
    ret ret = OK;
    // TODO: APList should be a unique list of names
    for (std::string & name : _APNames){
        bool found = false;
        for (int i=0; i<APList.size(); i++){
            if (name.compare(APList[i])==0){
                _APName2EventID[name] = i;
                found = true;
                break;
            }
        }
        if (!found){
            _log.erro("Cannot find %s from the provided AP list of an event. ", name);
            ret = ERR;
        }
    }
    if (ret == OK){
        _setEvAps = true;
    }
    return ret;
}

ret monitorLTLf::newEvent(std::vector<int> newEvent){
    if (!_setEvAps){
        _log.erro("Event APs not set.  ");
        return ERR;
    }
    if (_monitorStatus == VIOLATE){
        return OK;
    }
    if (newEvent.size() == 0){
        _log.erro("New event is empty.  ");
        return ERR;
    }
    if (newEvent.size() != _eventAPList.size()){
        _log.erro("New event size is not right.  ");
        return ERR;
    }
    // build a bdd to check the condition for an edge
    bdd event = bdd_true();
    std::vector<bdd> unknowns;
    for(std::string & name: _APNames){
        int nEvIndex = _APName2EventID[name];
        // theoretically we don't need to check "if nEvIndex is in the size of newEvent".
        int val = newEvent[nEvIndex];
        if (val == 1){
            event = bdd_and(event, _APName2BDD[name]);
        } else if (val == 0){
            event = bdd_and(event, bdd_not(_APName2BDD[name]));
        } else if (val == -1){
            unknowns.push_back(_APName2BDD[name]);
        }
        else {
            _log.erro("Unknown new event value: %d.  Should be -1 to 1. ");
            return ERR;
        }
    }
    // currently, the monitor starts in SLEEP mode.  If unknowns is empty, it turns to UNDECIDE
    //  If unknowns is not empty, it turns to SLEEP again.
    // In the future, we can do exist/forall abstraction with unknowns
    if ((_monitorStatus==SLEEP) && (unknowns.size() == 0)){
        _monitorStatus = UNDECIDE;
        _log.prnt("Awakes");
    } else if ((_monitorStatus==UNDECIDE) && (unknowns.size() > 0)){
        _monitorStatus = SLEEP;
        _log.prnt("Falls asleep");
    }
    // dont check event if in SLEEP
    if (_monitorStatus == SLEEP){
        return OK;
    }
    // find the next transition
    // TODO: check if two edges accept current event
    bool decided = false;
    const spot::bdd_dict_ptr& dict = _daut->get_dict();
    // spot::bdd_print_formula(std::cout, dict, event) << std::endl;
    for (auto & e: _daut->out(_curStateId)){
        bdd decide = bdd_and(e.cond, event);
        if (decide != bdd_false()){
            // accept
            decided = true;
            _curStateId = e.dst;
        }
    }
    if (!decided){
        // monitor is violated
        std::stringstream ss;
        ss << std::endl;
        ss << "========<print start>=========" << std::endl;
        ss << "monitor violation detected: " << _formulaString << std::endl; // << std::endl;
        // _pf.f.dump(std::cout) << std::endl;
        // violation information
        ss << "monitor dfa: " << std::endl;
        print_hoa(ss, _daut) << '\n';
        ss << "current state: " << _curStateId << std::endl;
        ss << "current event: " << std::endl;
        spot::bdd_print_formula(ss, dict, event) << std::endl;
        ss << "========<print end>=========" << std::endl;
        std::string errPrint = ss.str();
        _log.erro(errPrint);
        _monitorStatus = VIOLATE;
    }
    return OK;
}

void monitorLTLf::print(){
    std::cout << "Monitor DFA: " << std::endl;
    print_hoa(std::cout, _daut) << '\n';
    std::cout << "Current state: " << _curStateId << std::endl;
}

void monitorLTLf::updateAPNames(){
    spot::atomic_prop_set* collect = spot::atomic_prop_collect(_pf.f);
    spot::atomic_prop_set::iterator it;
    for (it = collect->begin(); it != collect->end(); ++it) {
        _APNames.push_back(it->ap_name());
    }
}

void monitorLTLf::updateAPName2BDD(){
    spot::atomic_prop_set* collect = spot::atomic_prop_collect(_pf.f);
    spot::atomic_prop_set::iterator it;
    for (it = collect->begin(); it != collect->end(); ++it) {
        // u_long f = *it;
        std::string name = it->ap_name();
        bdd bddAP = spot::atomic_prop_collect_as_bdd(*it, _daut);
        // const spot::bdd_dict_ptr& dict = aut->get_dict();
        // spot::bdd_print_formula(std::cout, dict, bddAP);
        _APName2BDD[name] = bddAP;
    }
}