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

#include "WalkPathPlanner.h"

#include <cmath>
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"
#include "messages/localisation/FieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "utility/nubugger/NUhelpers.h"


namespace modules {
    namespace behaviour {
        namespace planning {

            using messages::support::Configuration;
            using messages::input::Sensors;
            using messages::motion::WalkCommand;
            using messages::behaviour::WalkTarget;
            using messages::behaviour::WalkApproach;
            using messages::motion::WalkStartCommand;
            using messages::motion::WalkStopCommand;
            using messages::motion::KickFinished;
            using utility::nubugger::graph;

            using LocalisationBall = messages::localisation::Ball;
            using Self = messages::localisation::Self;
            using VisionBall = messages::vision::Ball;
            using VisionObstacle = messages::vision::Obstacle;

            //using namespace messages;

            //using messages::input::ServoID;
            //using messages::motion::ExecuteScriptByName;
            //using messages::behaviour::RegisterAction;
            //using messages::behaviour::ActionPriorites;
            //using messages::behaviour::LimbID;

            WalkPathPlanner::WalkPathPlanner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                //we will initially stand still
                planType = messages::behaviour::WalkApproach::StandStill;

                //do a little configurating
                on<Trigger<Configuration<WalkPathPlanner>>>([this] (const Configuration<WalkPathPlanner>& file){

                    turnSpeed = file.config["turnSpeed"].as<float>();
                    forwardSpeed = file.config["forwardSpeed"].as<float>();
                    footSeparation = file.config["footSeparation"].as<float>();

                    footSize = file.config["footSize"].as<float>();

                    //timers for starting turning and walking
                    walkStartTime = file.config["walkStartTime"].as<double>();
                    walkTurnTime = file.config["walkTurnTime"].as<double>();

                    //walk accel/deccel controls
                    accelerationTime = file.config["accelerationTime"].as<double>();
                    accelerationFraction = file.config["accelerationFraction"].as<float>();

                    //approach speeds
                    closeApproachSpeed = file.config["closeApproachSpeed"].as<float>();
                    closeApproachDistance = file.config["closeApproachDistance"].as<float>();
                    midApproachSpeed = file.config["midApproachSpeed"].as<float>();
                    midApproachDistance = file.config["midApproachDistance"].as<float>();

                    //turning values
                    turnDeviation = file.config["turnDeviation"].as<float>();

                    //hystereses
                    distanceHysteresis = file.config["distanceHysteresis"].as<float>();
                    turningHysteresis = file.config["turningHysteresis"].as<float>();
                    positionHysteresis = file.config["positionHysteresis"].as<float>();

                    //ball lineup
                    //XXX: add these once we know what they do
                    //vector<float> ballApproachAngle = file.config["ballApproachAngle"];
                    //svector<int> ballKickFoot = file.config["ballKickFoot"];
                    ballLineupDistance = file.config["ballLineupDistance"].as<float>();
                    ballLineupMinDistance = file.config["ballLineupMinDistance"].as<float>();

                    //extra config options
                    useAvoidance = file.config["useAvoidance"].as<bool>();
                    assumedObstacleWidth = file.config["assumedObstacleWidth"].as<float>();
                    avoidDistance = file.config["avoidDistance"].as<float>();

                    bearingSensitivity = file.config["bearingSensitivity"].as<float>();
                    ApproachCurveFactor = file.config["ApproachCurveFactor"].as<float>();
                });


                on<Trigger<KickFinished>>([this] (const KickFinished&) {
                    emit(std::move(std::make_unique<WalkStartCommand>()));
                });

                on<Trigger<Every<20, Per<std::chrono::seconds>>>,
                    With<messages::localisation::Ball>,
                    With<std::vector<messages::localisation::Self>>,
                    With<Optional<std::vector<messages::vision::Obstacle>>>,
                    With<std::vector<messages::vision::Ball>>,
                    Options<Sync<WalkPathPlanner>>
                   >([this] (
                     const time_t&,
                     const LocalisationBall& ball,
                     const std::vector<Self>& selfs,
                     const std::shared_ptr<const std::vector<VisionObstacle>>& robots,
                     const std::vector<VisionBall>&
                    ) {

                    if(planType == messages::behaviour::WalkApproach::StandStill){
                        emit(std::make_unique<WalkStopCommand>());
                        return;
                     } else if(planType == messages::behaviour::WalkApproach::DirectCommand){
                        std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
                        command->velocity = currentTargetPosition;
                        command->rotationalSpeed = currentTargetHeading[0];
                        emit(std::move(command));
                        emit(std::move(std::make_unique<WalkStartCommand>()));
                        return;
                    }

                    //std::cout << "starting path planning" << std::endl;
                    arma::vec normed_heading = arma::normalise(selfs.front().heading);
                    arma::mat robotToWorldRotation;
                    robotToWorldRotation << normed_heading[0] << -normed_heading[1] << arma::endr
                                         << normed_heading[1] <<  normed_heading[0];
                    arma::vec ballPos = robotToWorldRotation * arma::vec(ball.position) + arma::vec(selfs.front().position);
                    //std::cout << "ball pos found" << std::endl;
                    arma::vec targetPos,targetHead;
                    //work out where we're going
                    if (targetPosition == messages::behaviour::WalkTarget::Robot) {
                        //XXX: check if robot is visible
                    } else if (targetPosition == messages::behaviour::WalkTarget::Ball) {
                        targetPos = ballPos;
                    } else { //other types default to position/waypoint location
                        targetPos = currentTargetPosition;
                    }
                    //work out where to face when we get there
                    if (targetHeading == messages::behaviour::WalkTarget::Robot) {
                        //XXX: check if robot is visible
                    } else if (targetHeading == messages::behaviour::WalkTarget::Ball) {
                        targetHead = arma::normalise(ballPos-targetPos);
                    } else { //other types default to position/waypoint bearings
                        targetHead = arma::normalise(arma::vec(currentTargetHeading)-targetPos);
                    }
                    //calculate the basic movement plan
                    arma::vec movePlan = arma::vec({0.0,0.0,0.0});

                    if(planType == messages::behaviour::WalkApproach::ApproachFromDirection) {
                        movePlan = approachFromDirection(selfs.front(),targetPos,targetHead);
                    } else if(planType == messages::behaviour::WalkApproach::WalkToPoint) {
                        movePlan = goToPoint(selfs.front(),targetPos,targetHead);
                    } else if(planType == messages::behaviour::WalkApproach::OmnidirectionalReposition) {
                        movePlan = goToPoint(selfs.front(),targetPos,targetHead);
                    }

                    //work out if we have to avoid something
                    if (useAvoidance && (robots != NULL) && robots->size() > 0) {
                        //this is a vision-based temporary for avoidance
                        movePlan = avoidObstacles(*robots,movePlan);
                    }

                    //this applies limits and hysteresis to movement
                    movePlan = generateWalk(movePlan,
                                            planType == messages::behaviour::WalkApproach::OmnidirectionalReposition);
                    std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>();
                    command->velocity = arma::vec({movePlan[0],movePlan[1]});
                    command->rotationalSpeed = movePlan[2];
                    emit(graph("Walk command:", command->velocity[0], command->velocity[1], command->rotationalSpeed));
                    emit(std::move(std::make_unique<WalkStartCommand>()));
                    emit(std::move(command));//XXX: emit here
                });

                on<Trigger<messages::behaviour::WalkStrategy>, Options<Sync<WalkPathPlanner>>>([this] (const messages::behaviour::WalkStrategy& cmd) {
                    //reset hysteresis variables when a command changes
                    turning = 0;
                    distanceIncrement = 3;

                    //save the plan
                    planType = cmd.walkMovementType;
                    targetHeading = cmd.targetHeadingType;
                    targetPosition = cmd.targetPositionType;
                    currentTargetPosition = cmd.target;
                    currentTargetHeading = cmd.heading;
                });

                //Walk planning testing: Walk to ball face to goal
/*
                auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
                approach->targetPositionType = WalkTarget::Ball;
                approach->targetHeadingType = WalkTarget::WayPoint;
                approach->walkMovementType = WalkApproach::ApproachFromDirection;
                approach->heading = arma::vec({-3,0});
                approach->target = arma::vec({0,0});
                emit(std::move(approach));
*/

            }

            arma::vec WalkPathPlanner::generateWalk(const arma::vec& move, bool omniPositioning) {

                //this uses hystereses to provide a stable, consistent positioning and movement
                double walk_speed = 0.0;
                double walk_bearing = 0.0;
                double walk_rotation = 0.0;

                //check what distance increment we're in and swap/set speed accordingly:
                if ((move[0] > midApproachDistance + distanceHysteresis and distanceIncrement < 3)
                    or
                    (move[0] > midApproachDistance and distanceIncrement >= 3)) {

                    distanceIncrement = 3;
                    walk_speed = forwardSpeed;
                }
                else if ((move[0] > closeApproachDistance+distanceHysteresis and distanceIncrement < 2)
                         or
                         (move[0] > closeApproachDistance and distanceIncrement >= 2)) {

                    distanceIncrement = 2;
                    walk_speed = midApproachSpeed;
                }
                else if ((move[0] > ballLineupDistance+distanceHysteresis and distanceIncrement < 1)
                         or
                         (move[0] > ballLineupDistance and distanceIncrement >= 1)) {

                    distanceIncrement = 1;
                    walk_speed = closeApproachSpeed;
                }
                else if (move[0] < ballLineupDistance + distanceHysteresis) {
                    distanceIncrement = 0;
                    walk_speed = 0.f;
                }

                //decide between heading and bearing
                if ((distanceIncrement > 1) && (!omniPositioning)) {
                    walk_rotation = move[1];
                } else {
                    walk_bearing = move[1];
                    walk_rotation = move[2];
                }

                //make sure our rotation is normalised to our turning limits
                walk_rotation = fmin(turnSpeed,fmax(walk_rotation,-turnSpeed));

                //Replacing turn hysteresis with a unicorn equation
                float g = 1./(1.+std::exp(-4.*walk_rotation*walk_rotation));

                return arma::vec({walk_speed*(1.-g)*cos(walk_bearing),walk_speed*(1.-g)*sin(walk_bearing),walk_rotation*g});
            }

            arma::vec WalkPathPlanner::approachFromDirection(const Self& self,
                                                             const arma::vec2& target,
                                                             const arma::vec2& direction) {

                //this method calculates the possible ball approach commands for the robot
                //and then chooses the lowest cost action
                std::vector<arma::vec2> waypoints(3);

                //calculate the values we need to set waypoints
                const double ballDistance = arma::norm(target - self.position, 2);
                const double selfHeading = atan2(self.heading[1], self.heading[0]);

                //create our waypoints - these are approach vectors offset from the ball
                waypoints[0] = -direction * ballDistance * ApproachCurveFactor;
                waypoints[1] = arma::vec2({waypoints[0][1], -waypoints[0][0]});
                waypoints[2] = arma::vec2({-waypoints[0][1], waypoints[0][0]});


                //offset the waypoints by the foot separation so we are aiming at the ball with the correct foot
                std::vector<arma::vec2> footOffsets(3);
                footOffsets[1] = footOffsets[2] = arma::normalise(waypoints[0]) * (footSeparation + footSize) * 0.5;
                //do a foot offset for the straight approach case
                if (arma::dot(waypoints[1], waypoints[1]) < arma::dot(waypoints[1], waypoints[1])) {
                    footOffsets[0] = arma::normalise(waypoints[1]) * (footSeparation + footSize) * 0.5;
                } else {
                    footOffsets[0] = arma::normalise(waypoints[2]) * (footSeparation + footSize) * 0.5;
                }
                //fill target headings and distances, and movement bearings and costs
                std::vector<double> headings(3);
                std::vector<double> distances(3);
                std::vector<double> bearings(3);
                std::vector<double> costs(3);

                for (size_t i = 0; i < 3; ++i) {
                    //store the directions we want to face when we get to the ball in headings
                    const double waypointHeading = atan2(-waypoints[i][1], -waypoints[i][0]) - selfHeading;
                    headings[i] = atan2(sin(waypointHeading), cos(waypointHeading));

                    //calculate the estimated distance to destination - include the foot offset for ball lineup at this point
                    arma::vec waypointPos = waypoints[i] + target - self.position + footOffsets[i];
                    distances[i] = arma::norm(waypointPos, 2);

                    //calculate the angle between the current direction and the bearing of the destination
                    const double waypointBearing = atan2(waypointPos[1], waypointPos[0]) - selfHeading;
                    bearings[i] = atan2(sin(waypointBearing), cos(waypointBearing));

                    //costs defines which move plan is the most appropriate - the sensitivity gives us control over whether the robots prefers translation or rotation
                    costs[i] = bearings[i] * bearings[i] * bearingSensitivity + distances[i] * distances[i];

                }

                //decide which approach to the ball is cheapest
                size_t selected;
                if (costs[0] < costs[1] and costs[0] < costs[2]) {
                    selected = 0;
                } else if (costs[1] < costs[2]) {
                    selected = 1;
                } else {
                    selected = 2;
                }

		// Disable sideways approaches.
		selected = 0;

                return arma::vec({distances[selected], bearings[selected], headings[selected]});
            }

            arma::vec WalkPathPlanner::goToPoint(const Self& self,
                                                  const arma::vec2& target,
                                                  const arma::vec2& direction) {
                //quick and dirty go to point calculator
                //gets position and heading difference and returns walk params for it
                const arma::vec2 posdiff = target - self.position;
                const double targetDistance = arma::norm(posdiff);
                const double selfHeading = std::atan2(self.heading[1],self.heading[0]);
                const double targetHeading = std::atan2(posdiff[1],posdiff[0])-selfHeading;
                const double targetBearing = std::atan2(direction[1],direction[0])-selfHeading;

                arma::vec result(3);
                result[0] = targetDistance;
                result[1] = atan2(sin(targetHeading),cos(targetHeading));
                result[2] = atan2(sin(targetBearing),cos(targetBearing));

                //apply turning hysteresis
                return result;
            }

            arma::vec WalkPathPlanner::avoidObstacles(const std::vector<VisionObstacle>&,
                                                  const arma::vec3& movePlan) {
                return movePlan; //XXX:unimplemented
                //double heading = 0.0;
                /*float new_bearing = relative_bearing;
                float avoid_distance = min(m_avoid_distance,distance);
                vector<Object> obstacles;



                //use either localised or visual avoidance
                if (m_use_localisation_avoidance) {
                    //XXX: localisation based avoidance not implemented


                } else {
                    obstacles = NavigationLogic::getVisibleObstacles();
                    for(unsigned int i=0; i < obstacles.size(); i++) { //for each object
                        if (obstacles[i].measuredDistance() < avoid_distance) { //if we are an obstacle
                            if (obstacles[i].measuredBearing() > relative_bearing and obstacles[i].measuredBearing()-obstacles[i].arc_width < relative_bearing) {
                                //if we are on the right and occluding
                                new_bearing = mathGeneral::normaliseAngle(obstacles[i].measuredBearing()-obstacles[i].arc_width);
                            } else if (obstacles[i].measuredBearing() < relative_bearing and obstacles[i].measuredBearing()+obstacles[i].arc_width > relative_bearing) {
                                //if we are on the left and occluding
                                new_bearing = mathGeneral::normaliseAngle(obstacles[i].measuredBearing()+obstacles[i].arc_width);
                            }
                        }
                    }
                }*/
            }



        }  // planning
    }  // behaviours
}  // modules
