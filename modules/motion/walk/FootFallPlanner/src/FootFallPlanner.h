/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_WALK_FOOTFALLPLANNER_H
#define MODULES_MOTION_WALK_FOOTFALLPLANNER_H

#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "messages/motion/walk/WalkCommand.h"
#include "messages/behaviour/Action.h"
#include "messages/support/Configuration.h"

namespace modules {
namespace motion {
namespace walk {

    class FootFallPlanner : public NUClear::Reactor {
    private:
        static constexpr size_t UPDATE_FREQUENCY = 60;

        enum class State {
            STOPPED,
            STOP_REQUEST,
            WALKING,
            WALKING_INTERRUPTED
        };

        ReactionHandle updateHandle;

        /// Subsumption ID key to access motors
        const size_t id;

        // begin walk settings
        messages::behaviour::LimbID swingLegInitial = messages::behaviour::LimbID::LEFT_LEG;
        NUClear::clock::duration stepTime = std::chrono::seconds(4);
        // end walk settings

        // begin walk state
        State state;
        messages::motion::walk::WalkCommand currentWalkCommand;
        messages::behaviour::LimbID swingLeg = swingLegInitial;
        NUClear::clock::time_point beginStepTime;
        // end walk state

        void configure(const messages::support::Configuration<FootFallPlanner>& config);
        void resetState();
        void startWalking();
        void requestStopWalking();
        void stopWalking();
        void update();
        void newStep();
        messages::motion::walk::WalkCommand::SE2 calculateFootTarget(messages::behaviour::LimbID leg);
    public:
        static constexpr const char* CONFIGURATION_PATH = "walk/FootFallPlanner.yaml";
        /// @brief Called by the powerplant to build and setup the FootFallPlanner reactor.
        explicit FootFallPlanner(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}


#endif