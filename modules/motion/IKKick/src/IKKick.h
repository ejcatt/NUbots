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

#ifndef MODULES_MOTION_IKKICK_H
#define MODULES_MOTION_IKKICK_H

#include <nuclear>

namespace modules {
namespace motion {

    class IKKick : public NUClear::Reactor {

    private:
        /// Subsumption ID key to access motors
        const size_t id;
        
    	float KICK_PRIORITY;
    	float EXECUTION_PRIORITY;
    	void updatePriority(const float& priority);

    	static constexpr uint UPDATE_FREQUENCY = 90;

        ReactionHandle updater;

    public:
        static constexpr const char* CONFIGURATION_PATH = "IKKick.yaml";
        /// @brief Called by the powerplant to build and setup the IKKick reactor.
        explicit IKKick(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif