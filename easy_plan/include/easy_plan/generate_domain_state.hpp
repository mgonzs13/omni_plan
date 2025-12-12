#ifndef EASY_PLAN__GENERATE_DOMAIN_STATE_HPP
#define EASY_PLAN__GENERATE_DOMAIN_STATE_HPP

#include <memory>
#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"

namespace easy_plan {
class GenerateDomainState : public yasmin::State {
public:
  GenerateDomainState();

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) override;
};

} // namespace easy_plan

#endif // EASY_PLAN__GENERATE_DOMAIN_STATE_HPP