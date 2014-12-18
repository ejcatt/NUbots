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

#include "FootFallPlanner.h"

#include "messages/input/ServoID.h"
#include "messages/motion/walk/FootTarget.h"

#include "utility/support/YamlExpression.h"

namespace modules {
namespace motion {
namespace walk {

    using messages::input::ServoID;
    using messages::behaviour::LimbID;
    using messages::behaviour::ServoCommand;
    using messages::behaviour::ActionPriorites;
    using messages::behaviour::RegisterAction;
    using messages::motion::walk::WalkCommand;
    using messages::motion::walk::WalkStartCommand;
    using messages::motion::walk::WalkStopCommand;
    using messages::motion::walk::WalkStopped;
    using messages::motion::walk::WalkStarted;
    using messages::motion::walk::FootTarget;
    using messages::motion::walk::FootTargetComplete;
    using messages::support::Configuration;

    using utility::support::Expression;

    /**
     * SE2 notation vector: [x: m/s, y: m/s, angle: radians/s]
     */
    using SE2 = WalkCommand::SE2;

    FootFallPlanner::FootFallPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , id(size_t(this) * size_t(this) - size_t(this)) {

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
            id,
            "Walk Engine",
            {
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this] (const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // legs are available, start
                    if (state == State::WALKING_INTERRUPTED) {
                        state = State::WALKING;
                    }
                }
            },
            [this] (const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // legs are no longer available, reset walking (too late to stop walking)
                    if (state == State::WALKING) {
                        state = State::WALKING_INTERRUPTED;
                    }
                }
            },
            [this] (const std::set<ServoID>&) {
                // nothing
            }
        }));

        on<Trigger<Configuration<FootFallPlanner>>>(std::bind(&FootFallPlanner::configure, this, std::placeholders::_1));

        on<Trigger<WalkCommand>>([this](const WalkCommand& walkCommand) {
            currentWalkCommand = walkCommand;
        });

        on<Trigger<WalkStartCommand>>([this](const WalkStartCommand&) {
            startWalking();
        });

        on<Trigger<WalkStopCommand>>([this](const WalkStopCommand&) {
            requestStopWalking();
        });

        on<Trigger<FootTargetComplete>>([this](const FootTargetComplete&) {
            if (state == State::WALKING) {
                // swap legs
                swingLeg = swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG;
                newStep();
            }
        });

        resetState();
    }

    void FootFallPlanner::configure(const Configuration<FootFallPlanner>& config) {
        // uses milliseconds as std::chrono::durations must take an integer value
        // and the step time is likely to be less than 1
        stepTime = std::chrono::milliseconds(int(round(1000 * config["step_time"].as<Expression>())));
    }

    void FootFallPlanner::resetState() {
        state = State::STOPPED;
        swingLeg = swingLegInitial;
        beginStepTime = NUClear::clock::now();
    }

    void FootFallPlanner::startWalking() {
        if (state != State::WALKING) {
            resetState();
            state = State::WALKING;
            emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 25, 10 }})); // TODO: config
            emit(std::make_unique<WalkStarted>());
            newStep();
            log<NUClear::TRACE>("Walk Engine:: Start Walking");
        }
    }

    void FootFallPlanner::requestStopWalking() {
        state = State::STOP_REQUEST;
        log<NUClear::TRACE>("Walk Engine:: Stop Walk Requested");
    }

    void FootFallPlanner::stopWalking() {
        state = State::STOPPED;
        emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { 0, 0 }})); // TODO: config
        emit(std::make_unique<WalkStopped>());
        emit(std::make_unique<std::vector<ServoCommand>>()); // TODO: is this needed?
        log<NUClear::TRACE>("Walk Engine:: Stopped Walking");
    }

    void FootFallPlanner::newStep() {
        auto now = NUClear::clock::now();
        auto endStepTime = now + stepTime;
        SE2 footTarget = calculateFootTarget(swingLeg);
        emit(std::make_unique<FootTarget>(FootTarget{id, swingLeg, footTarget, endStepTime}));
        beginStepTime = now;
    }

    SE2 FootFallPlanner::calculateFootTarget(LimbID leg) {
        auto footTarget = arma::zeros(3);
        // TODO
        return footTarget;
    }
}
}
}

