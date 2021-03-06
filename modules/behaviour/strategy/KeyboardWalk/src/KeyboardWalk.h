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

#ifndef MODULES_BEHAVIOUR_STRATEGY_CONTROLLABLEDARWIN_H
#define MODULES_BEHAVIOUR_STRATEGY_CONTROLLABLEDARWIN_H

#include <nuclear>
#include <armadillo>

namespace modules {
namespace behaviour {
namespace strategy {

    class KeyboardWalk : public NUClear::Reactor {
    private:
        static constexpr const double DIFF = 0.01;
        static constexpr const double ROT_DIFF = 0.01;

        bool running = true;

        bool moving = false;
        arma::vec2 velocity;
        float rotation = 0;

        void run();
        void kill();

        void forward();
        void left();
        void back();
        void right();
        void turnLeft();
        void turnRight();
        void getUp();
        void reset();
        void walkToggle();
        void quit();

        void updateCommand();
        void printStatus();
    public:
        /// @brief Called by the powerplant to build and setup the KeyboardWalk reactor.
        explicit KeyboardWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}


#endif