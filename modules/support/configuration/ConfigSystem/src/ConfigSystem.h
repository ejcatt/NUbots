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

#ifndef MODULES_UTILITY_CONFIGURATION_CONFIGSYSTEM_H_
#define MODULES_UTILITY_CONFIGURATION_CONFIGSYSTEM_H_

#include <nuclear>
#include <yaml-cpp/yaml.h>

namespace modules {
    namespace support {
        namespace configuration {

            /**
             * Handles configuration objects for the rest of the system.
             *
             * @author Trent Houliston
             * @author Michael Burton
             */
            class ConfigSystem : public NUClear::Reactor {

            private:
                using HandlerFunction = std::function<void (NUClear::Reactor*, const std::string&, const YAML::Node&)>;

                std::set<std::type_index> loaded;
                std::map<std::string, std::vector<HandlerFunction>> handler;
                std::map<int, std::string> watchPath;
                std::map<std::string, NUClear::clock::time_point> timestamp;
                int watcherFd;
                int killFd;

                volatile bool running;

                void run();
                void kill();
                void loadDir(const std::string& path, HandlerFunction emit);
                void watchDir(const std::string& path);
                YAML::Node buildConfigurationNode(const std::string& filePath);

                // Lots of space for events (definitely more then needed)
                static constexpr size_t MAX_EVENT_LEN = 20 * 1024;
                static constexpr const char* BASE_CONFIGURATION_PATH = "config/";

            public:
                explicit ConfigSystem(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // configuration
    }  // support
}  // modules

#endif  // MODULES_UTILITY_CONFIGURATION_CONFIGSYSTEM_H_
