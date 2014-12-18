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
#include "utility/math/matrix.h"
#include "utility/support/YamlExpression.h"
#include "utility/support/YamlArmadillo.h"

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
    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::DarwinModel;
    using utility::math::matrix::translationMatrix;
    using utility::math::matrix::yRotationMatrix;
    using utility::math::matrix::orthonormal44Inverse;
    using utility::support::Expression;

    FootPathController::FootPathController(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

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
            controlId = footTarget.controlId;
            endStepTime = footTarget.time;
            swingLeg = footTarget.leg;
            target = footTarget.target;
            updateHandle.enable();
        });

    }
    void FootPathController::configure(const Configuration<FootPathController>& config) {
        phaseStart = config["walk_cycle"]["phase_single"][0].as<double>();
        phaseEnd = config["walk_cycle"]["phase_single"][1].as<double>();
        stepHeight = config["walk_cycle"]["step_height"].as<double>();
        bodyHeight = config["stance"]["body_height"].as<double>();
        bodyTilt = config["stance"]["body_tilt"].as<Expression>();
        footOffset = config["stance"]["foot_offset"].as<arma::vec>();
    }

    void FootPathController::update(const Sensors& sensors) {
        auto now = NUClear::clock::now();

        if (endStepTime <= beginStepTime) {
            log<NUClear::ERROR>("FootPathController:: Foot Target time is not in the future!");
            return;
        }

        double phase = (now - beginStepTime).count() / double((endStepTime - beginStepTime).count());

        // cap phase at 1
        phase = std::min(phase, 1.0);

        // calculate how far through the step we should go
        auto easing = phaseEasing(phase, phaseStart, phaseEnd);

        arma::mat44 leftFoot = arma::eye(4,4);
        arma::mat44 rightFoot = arma::eye(4,4);
        arma::mat44 torso = arma::eye(4,4);

        torso *= yRotationMatrix(bodyTilt, 4);

        leftFoot *= translationMatrix(arma::vec({footOffset[0], DarwinModel::Leg::HIP_OFFSET_Y + footOffset[1], -bodyHeight}));
        rightFoot *= translationMatrix(arma::vec({footOffset[0], -DarwinModel::Leg::HIP_OFFSET_Y - footOffset[1], -bodyHeight}));

        if (swingLeg == LimbID::LEFT_LEG) {
            leftFoot *= translationMatrix(arma::vec({0, 0, easing[2] * stepHeight}));
        }
        else {
            rightFoot *= translationMatrix(arma::vec({0, 0, easing[2] * stepHeight}));
        }

        auto torsoInv = orthonormal44Inverse(torso);

        leftFoot = torsoInv * leftFoot;
        rightFoot = torsoInv * rightFoot;

        auto joints = calculateLegJoints<DarwinModel>(leftFoot, rightFoot);

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
            waypoints->push_back({controlId, time, joint.first, joint.second, 30}); // TODO: config gains
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

