// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <yasmin/state.hpp>

#include "easy_plan/domain_generator.hpp"
#include "easy_plan/states/outcomes.hpp"

class GenerateDomainState : public yasmin::State {

public:
  GenerateDomainState()
      : yasmin::State({easy_plan::states::outcomes::SUCCEED}),
        state_loader_(std::make_unique<
                      pluginlib::ClassLoader<easy_plan::DomainGenerator>>(
            "easy_plan", "DomainGenerator")) {}

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {
    if (!this->domain_generator_) {
      std::string domain_plugin = blackboard->get<std::string>("domain_plugin");
      this->domain_generator_ =
          this->state_loader_->createUniqueInstance(domain_plugin);
    }

    blackboard->set<std::string>("domain",
                                 this->domain_generator_->get_domain());
    return easy_plan::states::outcomes::SUCCEED;
  }

private:
  std::unique_ptr<pluginlib::ClassLoader<easy_plan::DomainGenerator>>
      state_loader_;
  pluginlib::UniquePtr<easy_plan::DomainGenerator> domain_generator_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GenerateDomainState, yasmin::State)