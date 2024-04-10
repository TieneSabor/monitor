#include <iostream>
#include <spot/tl/parse.hh>
#include <spot/tl/ltlf.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/remprop.hh>
#include <spot/twaalgos/powerset.hh>
#include <spot/twaalgos/isdet.hh>
#include <spot/tl/apcollect.hh>
#include <spot/twa/bddprint.hh>

#include <string>
#include <map>

std::map<std::string, bdd> getFormulaAPName2BDD(spot::formula f, spot::twa_graph_ptr aut){
  std::map<std::string, bdd> ret;
  spot::atomic_prop_set* collect = spot::atomic_prop_collect(f);
  spot::atomic_prop_set::iterator it;
  for (it = collect->begin(); it != collect->end(); ++it) {
    // u_long f = *it;
    std::string name = it->ap_name();
    bdd bddAP = spot::atomic_prop_collect_as_bdd(*it, aut);
    // const spot::bdd_dict_ptr& dict = aut->get_dict();
    // spot::bdd_print_formula(std::cout, dict, bddAP);
    ret[name] = bddAP;
  }
  return ret;
}

int main()
{
  // spot::parsed_formula pf = spot::parse_infix_psl("(a U b) & Fc");
  // spot::parsed_formula pf = spot::parse_infix_psl("GFa | G(b <-> Xa)");
  spot::parsed_formula pf = spot::parse_infix_psl("(G e) & (G f)");
  if (pf.format_errors(std::cerr))
    return 1;

  spot::translator trans;
  trans.set_type(spot::postprocessor::Buchi);
  trans.set_pref(spot::postprocessor::SBAcc
                 | spot::postprocessor::Small);
  spot::twa_graph_ptr aut = trans.run(spot::from_ltlf(pf.f));
  // spot::twa_graph_ptr daut = spot::tba_determinize_check(aut, 0, 0, pf.f);
  aut = spot::to_finite(aut);
  spot::twa_graph_ptr daut = spot::to_finite(aut);
  daut = spot::tba_determinize_check(daut, 0, 0, pf.f);
  // print_hoa(std::cout, aut) << '\n';
  std::cout << "=====<DFA>=======" << std::endl;
  bool isd = spot::is_deterministic(daut);
  std::cout << isd << std::endl;
  print_hoa(std::cout, daut) << '\n';
  // find the set of ap
  auto map = getFormulaAPName2BDD(pf.f, daut);
  // test a&b
  bdd anb = bdd_and(map["a"], map["b"]);
  const spot::bdd_dict_ptr& dict = aut->get_dict();
  spot::bdd_print_formula(std::cout, dict, anb) << std::endl;
  // check for conditions of edges
  const spot::state* s = aut->get_init_state();
  for (auto i: daut->succ(s))
  {
      const spot::state* dst = i->dst();
      std::cout << aut->format_state(s) << "->"
                << aut->format_state(dst) << '\n';
      bdd condition = i->cond();
      bdd decide = bdd_and(condition, anb);
      spot::bdd_print_formula(std::cout, dict, condition) << std::endl;
      spot::bdd_print_formula(std::cout, dict, decide) << std::endl;
      if (decide == bdd_false()){
        std::cout << "reject" << std::endl;
      } else {
        std::cout << "accept" << std::endl;
      }
      // dfs_rec(aut, dst, seen);
      // Do not destroy dst, as it is either destroyed by dfs_rec()
      // or stored in seen.
  }
  return 0;
}