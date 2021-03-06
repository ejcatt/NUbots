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

#include "NUbugger.h"

#include <random>

#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Sensors.h"
#include "utility/nubugger/NUhelpers.h"
#include "messages/support/nubugger/proto/Message.pb.h"

namespace modules {
namespace debug {

    using NUClear::DEBUG;
    using utility::nubugger::graph;
    using utility::nubugger::drawArrow;
    using utility::nubugger::drawSphere;
    using messages::platform::darwin::DarwinSensors;
    using std::chrono::milliseconds;
    using messages::input::Sensors;
    using messages::input::ServoID;
    using messages::support::nubugger::proto::DrawObjects;

    NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Every<50, milliseconds>>>([this](const time_t&) {

            double period = 10;
            double freq = 1 / period;
            double t = NUClear::clock::now().time_since_epoch().count() / double(NUClear::clock::period::den);
            float sine = sin(2 * M_PI * freq * t);
            float cosine = cos(2 * M_PI * freq * t);
            float dsine = 2 * sine;
            float dcosine = 4 * cosine;

            emit(graph("Debug Waves", sine, cosine, dsine, dcosine));

        });

        on<Trigger<DarwinSensors>>([this](const DarwinSensors& sensors) {
            //Includes change to our standard coordinate system
            emit(graph(
                "Accelerometer",
                -sensors.accelerometer.y,
                sensors.accelerometer.x,
                -sensors.accelerometer.z

            ));

            emit(graph(
                "Gyro",
                -sensors.gyroscope.x,
                -sensors.gyroscope.y,
                sensors.gyroscope.z
            ));

        });

        on<Trigger<Sensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Sensors& sensors) {

            for (const auto& s : sensors.servos) {
                if (s.id == ServoID::L_HIP_ROLL){
                    emit(graph("Servo " + messages::input::stringFromId(s.id), s.presentPosition));
                }
            }

        });

        on<Trigger<Every<1, std::chrono::seconds>>>([this] (const time_t&) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(-2, 2);
            double x = dis(gen);
            double y = dis(gen);
            double z = dis(gen);

            double period = 10;
            double freq = 1 / period;
            double t = NUClear::clock::now().time_since_epoch().count() / double(NUClear::clock::period::den);
            float sine = sin(2 * M_PI * freq * t);
            float cosine = cos(2 * M_PI * freq * t);

            emit(drawArrow("arrow", {x, y, std::abs(z)}, {sine, cosine, 0}, sine));
            emit(drawSphere("sphere", {x, z, std::abs(z)}, std::abs(sine)));

        });

    }

} // debug
} // modules
