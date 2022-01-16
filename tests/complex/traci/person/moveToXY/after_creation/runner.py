#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2022 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Jakob Erdmann
# @date    2015-02-06


from __future__ import print_function
from __future__ import absolute_import
import os
import sys
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci  # noqa
import sumolib  # noqa

sumoBinary = sumolib.checkBinary('sumo')
cmd = [
    sumoBinary,
    "-n", "input_net2.net.xml",
    "-r", "input_routes.rou.xml",
    "--fcd-output", "fcd.xml",
    "--vehroute-output", "vehroutes.xml",
    "--no-step-log"]
traci.start(cmd)


def step():
    s = traci.simulation.getTime()
    traci.simulationStep()
    return s


p = "p0"
t = traci.simulation.getTime()
x, y = traci.person.getPosition(p)
print("t=%s x=%s y=%s" % (t, x, y))
print("jumping backwards on the same edge")
traci.person.moveToXY(p, "", x, y - 10)
t = traci.simulation.getTime()
traci.simulationStep()
x, y = traci.person.getPosition(p)
print("t=%s x=%s y=%s" % (t, x, y))

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
traci.close()
