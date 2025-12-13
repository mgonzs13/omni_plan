#include "easy_plan/pddl/expression.hpp"

namespace easy_plan {
namespace pddl {

Predicate::Predicate(const std::string &name,
                     const std::vector<std::string> &args, bool negated)
    : name_(name), args_(args), negated_(negated) {}

std::string Predicate::get_name() const { return this->name_; }

std::vector<std::string> Predicate::get_args() const { return this->args_; }

bool Predicate::is_negated() const { return this->negated_; }

std::string Predicate::to_pddl() const {
  std::string s = "(" + this->name_;
  for (const auto &arg : this->args_)
    s += " " + arg;
  s += ")";
  if (this->negated_)
    s = "(not " + s + ")";
  return s;
}

} // namespace pddl
} // namespace easy_plan