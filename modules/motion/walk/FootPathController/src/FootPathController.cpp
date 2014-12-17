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

#include "FootPathController.h"

#include "messages/motion/walk/FootTarget.h"

#include "utility/nubugger/NUhelpers.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/motion/RobotModels.h"

namespace modules {
namespace motion {
namespace walk {

    using messages::input::Sensors;
    using messages::input::ServoID;
    using messages::behaviour::LimbID;
    using messages::behaviour::ServoCommand;
    using messages::motion::walk::FootTarget;
    using messages::motion::walk::FootTargetComplete;
    using messages::support::Configuration;

    using utility::nubugger::graph;
    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::motion::kinematics::DarwinModel;

    FootPathController::FootPathController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        on<Trigger<Configuration<FootPathController>>>(std::bind(&FootPathController::configure, this, std::placeholders::_1));

        updateHandle = on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, With<Sensors>, Options<Single, Priority<NUClear::HIGH>>>([this](const time_t&, const Sensors& sensors) {
            update(sensors);
        });
        updateHandle.disable();

        on<Trigger<FootTarget>, With<Optional<Sensors>>>([this](const FootTarget& footTarget, const std::shared_ptr<const Sensors>& sensors) {
            auto now = NUClear::clock::now();

            if (footTarget.time <= now) {
                log<NUClear::ERROR>("FootPathController:: Foot Target time in the future!");
                return;
            }

            beginStepTime = now;
            endStepTime = footTarget.time;
            swingLeg = footTarget.leg;
            target = footTarget.target;
            updateHandle.enable();
        });

    }
    void FootPathController::configure(const Configuration<FootPathController>& config) {
        phaseStart = config["phase_single"][0].as<double>();
        phaseEnd = config["phase_single"][1].as<double>();
        stepHeight = config["step_height"].as<double>();
    }

    void FootPathController::update(const Sensors& sensors) {
        auto now = NUClear::clock::now();

        if (endStepTime <= beginStepTime) {
            log<NUClear::ERROR>("FootPathController:: Foot Target time is not in the future!");
            return;
        }

        double phase = (now - beginStepTime).count() / double((endStepTime - beginStepTime).count());


        phase = std::min(phase, 1.0);

        auto easing = phaseEasing(phase, phaseStart, phaseEnd);

        emit(graph("phase", phase));
        emit(graph("easing", easing));

        // update feet
        // update torso
        arma::mat44 leftFoot = arma::eye(4,4);
        arma::mat44 rightFoot = arma::eye(4,4);

        if (swingLeg == LimbID::LEFT_LEG) {
            leftFoot(2,3) = stepHeight * easing[2];
        }
        else {
            rightFoot(2,3) = stepHeight * easing[2];
        }

        auto joints = calculateLegJointsTeamDarwin<DarwinModel>(leftFoot, rightFoot);

        emit(motionLegs(joints));

        if (phase == 1) {
            // target complete
            phase = 1;
            updateHandle.disable();
            emit(std::make_unique<FootTargetComplete>());
        }
    }

    std::unique_ptr<std::vector<ServoCommand>> FootPathController::motionLegs(std::vector<std::pair<ServoID, float>> joints) {

        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        NUClear::clock::time_point time = NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den/UPDATE_FREQUENCY);

        for (auto& joint : joints) {
            waypoints->push_back({id, time, joint.first, joint.second, 30}); // TODO: config gains
        }

        return std::move(waypoints);
    }

    arma::vec3 FootPathController::phaseEasing(double phase, double phaseStart, double phaseEnd) {
        // Computes relative x,z motion of foot during single support phase
        // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
        double phaseSingle = std::min(std::max(phase - phaseStart, 0.0) / (phaseEnd - phaseStart), 1.0);
        double phaseSingleSkew = std::pow(phaseSingle, 0.8) - 0.17 * phaseSingle * (1 - phaseSingle);
        double xf = 0.5 * (1 - std::cos(M_PI * phaseSingleSkew));
        double zf = 0.5 * (1 - std::cos(2 * M_PI * phaseSingleSkew));

        return {xf, 0, zf};
    }

}
}
}

