/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2020-2022 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    sumo2fmi_bridge.c
/// @author  Robert Hilbrich
/// @date    Tue, 03 Mar 2020
///
// Implementation of the FMI to SUMO bridge features
/****************************************************************************/

#ifdef _MSC_VER
// Avoid warnings in windows build because of strcpy instead of strcpy_s,
// because the latter is not available on all platforms
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <foreign/fmi/fmi2Functions.h>
#include <string.h>
#include "libsumocpp2c.h"
#include "sumo2fmi_bridge.h"

/* Explicit definition of unused parameters to avoid compiler warnings */
#define UNREFERENCED_PARAMETER(P) (P)

void
sumo2fmi_set_startValues(ModelInstance *comp) {
    comp->freeMemory(comp->libsumoCallOptions);

    char* defaultCallOptions = "-c tools/game/grid6.sumocfg";
    comp->libsumoCallOptions = (char *)comp->allocateMemory(1 + strlen(defaultCallOptions), sizeof(char));
    strcpy((char *)comp->libsumoCallOptions, (char *)defaultCallOptions);
}

void
sumo2fmi_logError(ModelInstance *comp, const char *message, ...) {
    if (!comp->logErrors) return;

    va_list args;
    va_start(args, message);
    sumo2fmi_logMessage(comp, fmi2Error, "logStatusError", message, args);
    va_end(args);
}

void
sumo2fmi_logMessage(ModelInstance *comp, int status, const char *category, const char *message, va_list args) {
    va_list args1;
    size_t len = 0;
    char *buf = "";

    va_copy(args1, args);
    len = vsnprintf(buf, len, message, args1);
    va_end(args1);

    va_copy(args1, args);
    buf = comp->allocateMemory(len + 1, sizeof(char));
    vsnprintf(buf, len + 1, message, args);
    va_end(args1);

    comp->logger(comp->componentEnvironment, comp->instanceName, status, category, buf);

    comp->freeMemory(buf);
}

// Retrieve the integer value for a single variable
fmi2Status
sumo2fmi_getInteger(ModelInstance* comp, const fmi2ValueReference vr, int* value) {
    UNREFERENCED_PARAMETER(comp);

    // Do we need the pointer to comp here?
    switch (vr) {
        case 1:
            *value = libsumo_vehicle_getIDCount();
            return fmi2OK;
        default:
            return fmi2Error;
    }
}

fmi2Status
sumo2fmi_getString(ModelInstance* comp, const fmi2ValueReference vr, const char* value) {
    switch (vr) {
        case 0:
            value = comp->libsumoCallOptions;
            return fmi2OK;
        default:
            return fmi2Error;
    }
}

fmi2Status
sumo2fmi_setString(ModelInstance* comp, fmi2ValueReference vr, const char* value) {
    switch (vr) {
        case 0:
            comp->freeMemory(comp->libsumoCallOptions);
            comp->libsumoCallOptions = (char *)comp->allocateMemory(1 + strlen(value), sizeof(char));
            strcpy(comp->libsumoCallOptions, value);
            return fmi2OK;
        default:
            return fmi2Error;
    }
}

fmi2Status
sumo2fmi_step(ModelInstance *comp, double tNext) {
    UNREFERENCED_PARAMETER(comp);

    libsumo_step(tNext);
    return fmi2OK;
}
