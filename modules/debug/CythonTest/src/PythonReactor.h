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

#ifndef MODULES_DEBUG_PYTHONREACTOR_H
#define MODULES_DEBUG_PYTHONREACTOR_H

#include <nuclear>

namespace modules {
namespace debug {

    class PythonReactor : public NUClear::Reactor {

    public:
        template <typename... TList>
        struct Builder {

            template <typename T>
            Builder<TList..., Trigger<T>> trigger() {
                return Builder<TList..., Trigger<T>>();
            }

            template <typename T>
            Builder<TList..., With<T>> also() {
                return Builder<TList..., With<T>>();
            }

            template <typename T>
            Builder<TList..., Options<T>> option() {
                return Builder<TList..., Options<T>>();
            }

            template <typename Callback>
            void then(Reactor* context, Callback c) {
                context->on<TList...>(c);
            }

            template <typename Callback>
            void then(Reactor* context, std::string name, Callback c) {
                context->on<TList...>(name, c);
            }
        };

        static inline Builder<> on() {
            return Builder<>();
        };

    };

}
}


#endif