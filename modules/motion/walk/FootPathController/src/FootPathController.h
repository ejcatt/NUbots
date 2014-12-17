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

#ifndef MODULES_MOTION_WALK_FOOTPATHCONTROLLER_H
#define MODULES_MOTION_WALK_FOOTPATHCONTROLLER_H

#include <nuclear>
#include <armadillo>

#include "messages/motion/walk/WalkCommand.h"
#include "messages/behaviour/Action.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"

namespace modules {
namespace motion {
namespace walk {

    class FootPathController : public NUClear::Reactor {
    private:
        /**
         * The number of servo updates performnced per second
         */
        static constexpr size_t UPDATE_FREQUENCY = 60;

        ReactionHandle updateHandle;

        /// Subsumption ID key to access motors
        const size_t id;

        // start walk settings
        double phaseStart;
        double phaseEnd;
        double stepHeight;
        // end walk settings

        // start walk state
        messages::behaviour::LimbID swingLeg;
        NUClear::clock::time_point beginStepTime;
        NUClear::clock::time_point endStepTime;
        messages::motion::walk::WalkCommand::SE2 target;
        // end walk state

        void configure(const messages::support::Configuration<FootPathController>& config);
        void update(const messages::input::Sensors& sensors);
        arma::vec3 phaseEasing(double phase, double phaseStart, double phaseEnd);
        std::unique_ptr<std::vector<messages::behaviour::ServoCommand>> motionLegs(std::vector<std::pair<messages::input::ServoID, float>> qLegs);

    public:
        static constexpr const char* CONFIGURATION_PATH = "walk/FootPathController.yaml";
        /// @brief Called by the powerplant to build and setup the FootPathController reactor.
        explicit FootPathController(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}


#endif