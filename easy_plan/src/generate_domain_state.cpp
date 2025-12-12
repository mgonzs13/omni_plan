
#include "yasmin_ros/basic_outcomes.hpp"

#include "easy_plan/generate_domain_state.hpp"

using namespace easy_plan;

GenerateDomainState::GenerateDomainState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

std::string
GenerateDomainState::execute(std::shared_ptr<yasmin::Blackboard> blackboard) {
  return yasmin_ros::basic_outcomes::SUCCEED;
}