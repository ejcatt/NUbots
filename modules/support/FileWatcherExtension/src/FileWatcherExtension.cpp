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

#include "FileWatcherExtension.h"

#include <sys/inotify.h>

namespace modules {
namespace support {

    FileWatcherExtension::FileWatcherExtension(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , watcherFd(inotify_init()) {

        struct WatchTask {
            int wd;
            bool directory;
            std::string path;
            std::vector<std::shared_ptr<NUClear::threading::Reaction>> reactions;
        };

        on<IO>(fd).then([this] (int fd, int set) {

            // Our buffer to hold the events
            uint8_t buffer[MAX_EVENT_LEN];

            // Read our buffer
            int length = read(fd, buffer, sizeof(buffer));

            // Inspect our events
            for(int i = 0; i < length;) {
                // Extract our event
                inotify_event* event = reinterpret_cast<inotify_event*>(&buffer[i]);

                // Get the task for this descriptor
                auto& task = tasks[event->wd];

                // General flags
                IN_ACCESS;
                IN_ATTRIB;
                IN_CLOSE_WRITE;
                IN_CLOSE_NOWRITE;
                IN_DELETE;
                IN_DELETE_SELF;
                IN_MODIFY;
                IN_MOVE_SELF;
                IN_MOVED_FROM;
                IN_MOVED_TO;
                IN_OPEN;

                // Grouped flags
                IN_MOVE;
                IN_CLOSE;

                // Mutation flags
                IN_DONT_FOLLOW
                IN_EXCL_UNLINK;
                IN_MASK_ADD;
                IN_ONESHOT;
                IN_ONLYDIR;

                // Other events
                IN_IGNORED;
                IN_ISDIR;
                IN_Q_OVERFLOW;
                IN_UNMOUNT;

                // If it's a directory then append the name to it to get the full file path

                // If this wd corresponds to a task
                if(tasks.find(event->wd) != std::end(task)) {

                }
                else {
                    // Maybe we should remove this wd? (how did this happen!)
                }
                // Lookup the reactions, type and path for this wd
                event->wd;

                // Extract all the mask options
                event->mask;

                // Need to link move events by this cookie
                event->cookie;

                // The filename that was returned
                event->name;

                // Next event
                i += sizeof(inotify_event) + event->len;
            }

            // Do the things we need to do
        });

        // How to add a watch wd is a unique identifier that will be returned
        int wd = inotify_add_watch(watcherFd, path.c_str(), IN_ATTRIB | IN_MODIFY | IN_CREATE | IN_DELETE_SELF | IN_MOVE);

    }

}
}

