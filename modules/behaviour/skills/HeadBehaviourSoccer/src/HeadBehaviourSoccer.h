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

#ifndef MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_H
#define MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_H

#include <nuclear>
#include <armadillo>
#include <set>
#include "messages/vision/VisionObjects.h"
#include "messages/motion/HeadCommand.h"
#include "messages/input/Sensors.h"
#include "messages/input/CameraParameters.h"
#include "Searcher.h"

namespace modules {
    namespace behaviour{
        namespace skills {

            /**
             * Executes a HeadBehaviourSoccer action.
             *
             * @author Jake Fountain
             */
            class HeadBehaviourSoccer : public NUClear::Reactor {
            public:
                enum SearchType {
                    LOST = 0,
                    FIND_ADDITIONAL_OBJECTS = 1,
                    OTHER = 2
                };
                SearchType searchTypeFromString(std::string s){

                    if(s.compare("LOST") == 0) {
                        return SearchType::LOST;
                    }
                    else if(s.compare("FIND_ADDITIONAL_OBJECTS") == 0){
                        return SearchType::FIND_ADDITIONAL_OBJECTS;
                    }
                    else if(s.compare("OTHER") == 0){
                        return SearchType::OTHER;
                    } else {
                        throw std::domain_error("HeadBehaviourSoccer - searchTypeFromString: NO SEARCH TYPE FOUND!");
                    }

                }

            private:
                
                /*! @brief Updates the search plan when something has changed
                */
                void updateHeadPlan(const std::vector<messages::vision::VisionObject>& fixationObjects, const bool& search, const messages::input::Sensors& sensors, const utility::math::matrix::Rotation3D& headToIMUSpace);
                
                /*! @brief Converts from camera space direction to IMU space direction
                */
                arma::vec2 getIMUSpaceDirection(const arma::vec2& screenAngles, const utility::math::matrix::Rotation3D& headToIMUSpace);
                
                /*! @brief Gets points which allow for simultaneous search and viewing of key objects
                */
                std::vector<arma::vec2> getSearchPoints(std::vector<messages::vision::VisionObject> fixationObjects, SearchType sType);
                
                /*! @brief Combines a collection of vision objects. The screen resulting screen angular region is the bounding box of the objects
                */
                messages::vision::VisionObject combineVisionObjects(const std::vector<messages::vision::VisionObject>& obs);
                
                /*! @brief Gets a bounding box in screen angular space of a set of vision objects
                */
                utility::math::geometry::Quad getScreenAngularBoundingBox(const std::vector<messages::vision::VisionObject>& obs);


                //CONFIG - loaded elsewhere
                float max_yaw;
                float min_yaw;
                float max_pitch;
                float min_pitch;

                messages::input::CameraParameters cam;

                //CONFIG from HeadBehaviourSoccer.yaml
                double fractional_view_padding;
                float search_timeout_ms;
                float fractional_angular_update_threshold;

                std::map<SearchType, std::vector<arma::vec2>> searches;

                //State variables
                Searcher<arma::vec2> headSearcher;

                int ballPriority;
                int goalPriority;

                NUClear::clock::time_point lastPlanUpdate;
                NUClear::clock::time_point timeLastObjectSeen;
                
                arma::vec2 lastCentroid;

                bool lostAndSearching;
                bool lostLastTime;

                int lastBallPriority;
                int lastGoalPriority;

            public:
                explicit HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment);
                static constexpr const char* CONFIGURATION_PATH = "HeadBehaviourSoccer.yaml";
            };

        }  // skills
    } //behaviour
}  // modules

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADBEHAVIOURSOCCER_H

