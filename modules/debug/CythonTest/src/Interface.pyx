# This file is part of NUbots Codebase.
#
# The NUbots Codebase is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# The NUbots Codebase is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
#
# Copyright 2013 NUBots <nubots@nubots.net>

cdef extern from "<nuclear>" namespace "NUClear":
    cdef cppclass Reactor:
        pass

# Dirty hack that makes cython think auto is a thing (hehe) allowing us to construct a complex template
cdef extern from "":
    cdef cppclass auto:
        auto trigger[T]()
        auto also[T]()
        auto option[T]()
        void then[T](Reactor*, T)
        void then[T](Reactor*, string, T)

cdef extern from "PythonReactor.h" namespace "modules::debug::PythonReactor":
    auto on()

cdef void doSomething(int i, double d):
    print "Int:", i, "Double:", d

cdef class Interface:
    cdef Reactor* reactor

    cdef bind(self, Reactor* reactor):
        self.reactor = reactor
        on().trigger[int]().also[double]().then(reactor, doSomething)

cdef public buildCythonTest(Reactor* reactor):
    interface = Interface()
    interface.bind(reactor)
    return interface
