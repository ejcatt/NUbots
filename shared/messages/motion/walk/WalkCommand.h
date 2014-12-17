/*
 * This file is part of the NUbots Codebase.
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

#ifndef MESSAGES_MOTION_WALK_WALKCOMMAND_H
#define MESSAGES_MOTION_WALK_WALKCOMMAND_H

#include <armadillo>

namespace messages {
namespace motion {
namespace walk {

    /**
     * TODO document
     *
     * @author Trent Houliston
     * @author Brendan Annable
     */
    struct WalkCommand {
        /**
         * SE2 notation vector: [x: m/s, y: m/s, angle: radians/s]
         */
        using SE2 = arma::vec3;

        SE2 command;

        auto velocity() -> decltype(command.rows(0,1)) {
            return command.rows(0,1);
        }

        auto rotationalSpeed() -> decltype(command[2]) {
            return command[2];
        }
    };

    struct WalkStartCommand {
    };

    struct WalkStopCommand {
    };

    struct WalkStarted {
    };

    struct WalkStopped {
    };

}  // walk
}  // motion
}  // messages

#endif  // MESSAGES_MOTION_WALK_WALKCOMMAND_H