/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2022 German Aerospace Center (DLR) and others.
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
/// @file    NEMAController.cpp
/// @author  Tianxin Li
/// @author  Qichao Wang
/// @date    August 2020
///
// An actuated NEMA-phase-compliant traffic light logic
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <utility>
#include <vector>
#include <bitset>
#include <sstream>
#include <iostream>
#include <utils/common/FileHelpers.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/output/MSE2Collector.h>
#include <microsim/output/MSInductLoop.h>
#include <netload/NLDetectorBuilder.h>
#include "NEMAController.h"


// ===========================================================================
// parameter defaults definitions
// ===========================================================================
#define INVALID_POSITION std::numeric_limits<double>::max() // tl added

#define DEBUG_NEMA

// ===========================================================================
// method definitions
// ===========================================================================
NEMALogic::NEMALogic(MSTLLogicControl& tlcontrol,
                     const std::string& id, const std::string& programID,
                     const SUMOTime _offset,
                     const Phases& phases,
                     int step, SUMOTime delay,
                     const std::map<std::string, std::string>& parameter,
                     const std::string& basePath) :
    MSSimpleTrafficLightLogic(tlcontrol, id, programID, _offset, TrafficLightType::NEMA, phases, step, delay, parameter),
    myPhase(phases[0]->duration, phases[0]->getState()) {
    myDetectorLength = StringUtils::toDouble(getParameter("detector-length", "20"));
    myDetectorLengthLeftTurnLane = StringUtils::toDouble(getParameter("detector-length-leftTurnLane", "20"));
    myCycleLength = TIME2STEPS(StringUtils::toDouble(getParameter("total-cycle-length", getParameter("cycle-length", getParameter(toString(SUMO_ATTR_CYCLETIME), "60")))));
    myNextCycleLength = myCycleLength;
    myDefaultCycleTime = TIME2STEPS(myCycleLength);
    myShowDetectors = StringUtils::toBool(getParameter("show-detectors", toString(OptionsCont::getOptions().getBool("tls.actuated.show-detectors"))));
    myFile = FileHelpers::checkForRelativity(getParameter("file", "NUL"), basePath);
    myFreq = TIME2STEPS(StringUtils::toDouble(getParameter("freq", "300")));
    myVehicleTypes = getParameter("vTypes", "");
    myCabinetType = parseCabinetType(getParameter("cabinetType", "Type170"));
    ignoreErrors = StringUtils::toBool(getParameter("ignore-errors", "false"));
    
    // TODO: Create a parameter for this
    cycleRefPoint = TIME2STEPS(0);
    

    // for (int i = 0; i < (int)VecMaxRecall.size(); i++) {
    //     maxRecalls[VecMaxRecall[i] - 1] = true;
    //     recall[VecMaxRecall[i] - 1] = true;
    // }

    std::string barriers = getParameter("barrierPhases", "");
    std::string coordinates = getParameter("coordinatePhases", getParameter("barrier2Phases", ""));
    std::string ring1 = getParameter("ring1", "");
    std::string ring2 = getParameter("ring2", "");
    
    fixForceOff = StringUtils::toBool(getParameter("fixForceOff", "false"));
    offset = _offset;
    myNextOffset = offset;
    whetherOutputState = StringUtils::toBool(getParameter("whetherOutputState", "false"));
    coordinateMode = StringUtils::toBool(getParameter("coordinate-mode", "false"));
    greenTransfer = StringUtils::toBool(getParameter("greenTransfer", "true"));

    //missing parameter error
    error_handle_not_set(ring1, "ring1");
    error_handle_not_set(ring2, "ring2");
    error_handle_not_set(barriers, "barrierPhases");
    error_handle_not_set(coordinates, "barrier2Phases or coordinatePhases");

    //print to check
#ifdef DEBUG_NEMA
    std::cout << "JunctionID = " << myID << std::endl;
    std::cout << "All parameters after calling constructor are: " << std::endl;
    std::cout << "myDetectorLength = " << myDetectorLength << std::endl;
    std::cout << "cycleLength = " << STEPS2TIME(myCycleLength) << std::endl;
    std::cout << "ring1 = " << ring1 << std::endl;
    std::cout << "ring2 = " << ring2 << std::endl;
    std::cout << "barriers = " << barriers << std::endl;
    std::cout << "coordinates = " << coordinates << std::endl;
    std::cout << "offset = " << offset << std::endl;
    std::cout << "whetherOutputState = " << whetherOutputState << std::endl;
    std::cout << "myShowDetectors = " << myShowDetectors << std::endl;
    std::cout << "coordinateMode = " << coordinateMode << std::endl;
    std::cout << "fixForceOff = " << fixForceOff << std::endl;
    std::cout << "greenTransfer = " << greenTransfer << std::endl;
    std::cout << "You reach the end of constructor" << std::endl;
    std::cout << "****************************************\n";
#endif
    // Construct the NEMA specific timing data types and initial phases
    constructTimingAndPhaseDefs(barriers, coordinates, ring1, ring2);
}

NEMALogic::~NEMALogic() { 
    for (auto p: myPhaseObjs){
        delete p;
    }
}

void
NEMALogic::constructTimingAndPhaseDefs(std::string &barriers, std::string &coordinates, std::string &ring1, std::string &ring2){
    //init barriers
    std::vector<int> barrierPhases = readParaFromString(barriers);
    std::vector<int> coordinatePhases = readParaFromString(coordinates);
    
    // create the rings object. This is a holdover from the old codebase  
    rings.push_back(readParaFromString(ring1));
    rings.push_back(readParaFromString(ring2));

    
    #ifdef DEBUG_NEMA
    //print to check
    for (int i = 0; i < (int)rings.size(); i++) {
        int count = 0;
        std::cout << "Ring" << i + 1 << " includes phases: \t";
        for (auto j : rings[i]) {
            count++;
            std::cout << j << " ";
            if (count == 2 || count == 4) {
                std::cout << " | ";
            }
        }
        std::cout << std::endl;
    }
    #endif


    // load the recalls
    std::vector<int> VecMinRecall = readParaFromString(getParameter("minRecall", "1,2,3,4,5,6,7,8"));
    std::vector<int> VecMaxRecall = readParaFromString(getParameter("maxRecall", ""));

    #ifdef DEBUG_NEMA
        std::cout << "minRecall: ";
        for (int i = 0; i < 8; i++) {
            std::cout << vectorContainsPhase(VecMinRecall, i + 1) << '\t';
        }
        std::cout << std::endl;

        std::cout << "maxRecall: ";
        for (int i = 0; i < 8; i++) {
            std::cout << vectorContainsPhase(VecMaxRecall, i + 1) << '\t';
        }
        std::cout << std::endl;
    #endif

    // loop through the rings and construct the augmented phases. 
    // This relies on the phase being in order in the rings parameter in the configuration file 
    int ringNum = 0;
    int lastPhaseIter = 0;
    int phaseIter = 0;
    for (const auto &r: rings){
        int ringIter = 0;
        lastPhaseIter = phaseIter;
        phaseIter = 0;
        for (const auto &p: r){
            if (p != 0){
                // find the phase definition matching the phase integer
                MSPhaseDefinition *tempPhase = nullptr;
                for (const auto &pDef: myPhases){
                    if (string2int(pDef->getName()) == p){
                        tempPhase = pDef;
                        break;
                    }
                }
                // there must be a matching MSPhaseDefinition
                assert(tempPhase != nullptr);

                // create lane specific objects 
                std::string state = tempPhase->getState();
                std::set<std::string> laneIDs = getLaneIDsFromNEMAState(state);
                std::vector<std::string> laneIDs_vector;
                for (std::string laneID : laneIDs) {
                    laneIDs_vector.push_back(laneID);
                    myLanePhaseMap[laneID] = p;
                }
                phase2ControllerLanesMap[p] = laneIDs_vector;

                // Create the Phase Object
                // find if it is at a barrier
                bool barrierPhase = vectorContainsPhase(barrierPhases, p) || vectorContainsPhase(coordinatePhases, p);
                bool coordinatePhase = vectorContainsPhase(coordinatePhases, p) && coordinateMode;
                bool minRecall = vectorContainsPhase(VecMinRecall, p);
                bool maxRecall = vectorContainsPhase(VecMaxRecall, p);
                // TODO: Determine if this is the proper logic.
                bool greenRest = vectorContainsPhase(coordinatePhases, p) && (myCabinetType == TS2 || !coordinateMode);
                // could add per-phase fixforceoff here
                // barrierNum is either 0 or 1, depending on mainline side or sidestreet
                int barrierNum = ringIter / 2;

                // now ready to create the phase
                myPhaseObjs.push_back(
                    new NEMAPhase(p, barrierPhase, greenRest, coordinatePhase, minRecall, maxRecall, fixForceOff, barrierNum, ringNum, tempPhase)
                );
                
                // Add a reference to the  
                if (phaseIter > 0){
                    myPhaseObjs.back()->setSequentialPriorPhase(myPhaseObjs[lastPhaseIter + (phaseIter - 1)]);
                }
                phaseIter++;
            }
            ringIter++;
        }
        // Set the first to point to the last, wrapping around the ring.
        myPhaseObjs[lastPhaseIter]->setSequentialPriorPhase(myPhaseObjs[lastPhaseIter + phaseIter - 1]);
        // index the ring counter
        ringNum++;
    }

    // set the default phases. These should be set according to the b
    for (int i = 0; i < 2; i++){
        // create the coordinate phase ptr
        coordinatePhaseObjs[i] = getPhaseObj(coordinatePhases[i], i);
        defaultBarrierPhases[i][coordinatePhaseObjs[i]->barrierNum] = coordinatePhaseObjs[i];
        // create the other barrier phase ptr
        PhasePtr b = getPhaseObj(barrierPhases[i], i);
        defaultBarrierPhases[i][b->barrierNum] = b;
        assert(b->barrierNum != coordinatePhaseObjs[i]->barrierNum);
    }

    // Initialize the created phases with their detector objects
    IntVector latchingDetectors = readParaFromString(getParameter("latchingDetectors", ""));

    // create a vector of pairs of cross phase switching
    std::vector<std::pair<int, int>> cp;
    for (auto &p : myPhaseObjs){
        std::string cps = "crossPhaseSwitching:";
        int crossPhase = StringUtils::toInt(getParameter(cps.append(std::to_string(p->phaseName)), "0"));
        if (crossPhase > 0){
            cp.push_back({p->phaseName, crossPhase});
        }
    }

    for (auto &p : myPhaseObjs){
        bool latching = vectorContainsPhase(latchingDetectors, p->phaseName);
        int cpTarget = 0;
        int cpSource = 0;
        for (auto &cp_pair : cp){
            if (cp_pair.first == p->phaseName || cp_pair.second == p->phaseName){
                cpTarget = cp_pair.first;
                cpSource = cp_pair.second;
            } 
        }
        p->init(this, cpTarget, cpSource, latching);
    }    
    
    // Calculate Force offs Based on Timing
    calculateForceOffs();

    if (coordinateMode){
        // Calculate the Initial Phases in coordinated operation only.
        // Otherwise they have already been calculated above
        calculateInitialPhases();
    } else {
        // default to starting in the "first" phase with an active call (detector or recall), 
        // with the fall back being the barrier 0 default phases
        // NEMAPhase* defaultP[2] = {defaultBarrierPhases[0][0], defaultBarrierPhases[1][0]};
        NEMAPhase* defaultP[2] = {getPhasesByRing(0).front(), getPhasesByRing(1).front()};
        // for (int i = 0; i < 2; i++){
        //     for (auto &p: getPhasesByRing(i)){
        //         if (p->callActive()){
        //             defaultP[i] = p;
        //             break;
        //         }
        //     }
        //     if (defaultP[i]->barrierNum != defaultP[!i]->barrierNum){
        //         // this means that the most recent set phase's barrier is not equal to the others
        //         defaultP[!i] = defaultBarrierPhases[!i][defaultP[i]->barrierNum];
        //     }
        // }
        // Forcefully enter these phases
        defaultP[0]->forceEnter(this);
        defaultP[1]->forceEnter(this);
    }



#ifdef DEBUG_NEMA
    //print to check the rings and barriers active phase
    std::cout << "After init, active ring1 phase is " << myActivePhaseObjs[0]->phaseName << std::endl;
    std::cout << "After init, active ring2 phase is " << myActivePhaseObjs[1]->phaseName << std::endl;


    //print to check the phase definition is correct
    std::cout << "Print to check NEMA phase definitions\n";
    for (auto &p : myPhaseObjs) {
        std::cout << "index = " << p->phaseName << "; ";
        std::cout << "minDur = " << std::to_string(p->minDuration) << "; ";
        std::cout << "maxDur = " << std::to_string(p->maxDuration) << "; ";
        std::cout << "vehext = " << std::to_string(p->vehExt) << "; ";
        std::cout << "yellow = " << std::to_string(p->yellow) << "; ";
        std::cout << "red = " << std::to_string(p->red) << "; ";
        std::cout << "state = " << std::to_string((int)p->getCurrentState()) << std::endl;
    }
#endif

#ifdef DEBUG_NEMA
    std::cout << "After init, r1/r2 barrier phase = " << defaultBarrierPhases[0][1]->phaseName << " and " << defaultBarrierPhases[1][1]->phaseName << std::endl;
    std::cout << "After init, r1/r2 coordinate phase = " << defaultBarrierPhases[0][0]->phaseName << " and " << defaultBarrierPhases[1][0]->phaseName << std::endl;
#endif


#ifdef DEBUG_NEMA
    std::cout << "R1State = " << myActivePhaseObjs[0]->phaseName << " and its state = " << std::to_string((int)myActivePhaseObjs[0]->getCurrentState()) << std::endl;
    std::cout << "R2State = " << myActivePhaseObjs[1]->phaseName << " and its state = " << std::to_string((int)myActivePhaseObjs[0]->getCurrentState()) << std::endl;
#endif
    
    // Set the light state
    myPhase.setState(composeLightString());
    myPhase.setName(toString(myActivePhaseObjs[0]->phaseName) + "+" + toString(myActivePhaseObjs[1]->phaseName));
    myStep = 0;

    //validating timing
    validate_timing();
}

const bool
NEMALogic::vectorContainsPhase(std::vector<int> v, int phaseNum){
    if(std::find(v.begin(), v.end(), phaseNum) != v.end()) {
        return true;
    } 
    return false;  
}

void
NEMALogic::init(NLDetectorBuilder& nb) {
    //init the base path for output state
    outputStateFilePath = outputStateFilePath + "/" + myID + "_state_output";
    // std::cout << "outputStaetFilePath = " << outputStateFilePath << std::endl;
    //init cycleRefPoint

    //init outputStateFile
    if (whetherOutputState) {
        outputStateFile.open(outputStateFilePath);
        outputStateFile << "Output state changes:\n";
        outputStateFile.close();
    }



    //init the traffic light
    MSTrafficLightLogic::init(nb);
    assert(myLanes.size() > 0);
    //iterate through the lanes and build one E2 detector for each lane associated with the traffic light control junction
    for (const LaneVector& lanes : myLanes) {
        for (MSLane* const lane : lanes) {
            //decide the detector length
            double detector_length = 0;
            if (isLeftTurnLane(lane)) {
                detector_length = myDetectorLengthLeftTurnLane;
            } else {
                detector_length = myDetectorLength;
            }
            if (noVehicles(lane->getPermissions())) {
                // do not build detectors on green verges or sidewalks
                continue;
            }
            // Build detector and register them in the detector control
            if (myLaneDetectorMap.find(lane) == myLaneDetectorMap.end()) {
                MSE2Collector* det = nullptr;
                const std::string customID = getParameter(lane->getID());
                if (customID != "") {
                    det = dynamic_cast<MSE2Collector*>(MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_LANE_AREA_DETECTOR).get(customID));
                    if (det == nullptr) {
                        WRITE_ERROR("Unknown laneAreaDetector '" + customID + "' given as custom detector for NEMA tlLogic '" + getID() + "', program '" + getProgramID() + ".");
                        continue;
                    }
                    //set the detector to be visible in gui
                    det->setVisible(myShowDetectors);
                } else {
                    int phaseNumber = 0;
                    if (myLanePhaseMap.find(lane->getID()) != myLanePhaseMap.end()){    
                         phaseNumber = myLanePhaseMap.find(lane->getID()) -> second;
                    }
                    std::string id = myID + "_" + myProgramID + "_D" + toString(phaseNumber) + "." + toString(lane->getIndex());
                    // std::cout << "The detectorID = " << id << std::endl;
                    //createE2Detector() method will lead to bad detector showing in sumo-gui
                    //so it is better to use build2Detector() rather than createE2Detector()
                    // det = nb.createE2Detector(id, DU_TL_CONTROL, lane, INVALID_POSITION, lane->getLength(), myDetectorLength, 0, 0, 0, myVehicleTypes, myShowDetectors);
                    // MSNet::getInstance()->getDetectorControl().add(SUMO_TAG_LANE_AREA_DETECTOR, det, myFile, myFreq);
                    nb.buildE2Detector(id, //detectorID
                                       lane, //lane to build this detector
                                       INVALID_POSITION, // set the detector location by end point and length, so this one is set to invalue value so this parameter can be passed
                                       lane->getLength(), // set the end position of the detector at the end of the lane, which is right at the position of stop bar of a junction
                                       detector_length, //detector length
                                       myFile, // detector information output file
                                       myFreq, // detector reading interval
                                       0, // time-based threshold that decribes how much time has to pass until a vehicle is considerred as halting
                                       0, // speed threshold as halting
                                       0, // minimum dist to the next standing vehicle to make this vehicle count as a participant to the jam
                                       myVehicleTypes, //vehicle types to consider, if it is empty, meaning consider all types of vehicles
                                       false, // detector position check. More details could be found on SUMO web
                                       myShowDetectors, // whether to show detectors in sumo-gui
                                       0, //traffic light that triggers aggregation when swithing
                                       0); // outgoing lane that associated with the traffic light

                    //get the detector to be used in the lane detector map loading
                    det = dynamic_cast<MSE2Collector*>(MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_LANE_AREA_DETECTOR).get(id));
                }
                // print to check
                // std::cout << "E2Detector " << det->getID() << " is built on laneID = " << lane->getID() << std::endl;

                //map the detector to lane and lane to detector
                myLaneDetectorMap[lane] = det;
                myDetectorLaneMap[det] = lane;
                myDetectorInfoVector.push_back(DetectorInfo(det, (int)myPhases.size()));

            }
        }
    }
    for (auto item : phase2ControllerLanesMap) {
        int NEMAPhaseIndex = item.first;
        std::vector<std::string> laneIDs = item.second;
        std::vector<MSE2Collector*> detectors;
        MSE2Collector* detector = nullptr;
        for (std::string laneID : laneIDs) {
            MSLane* lane = MSLane::dictionary(laneID);
            detector = myLaneDetectorMap[lane];
            detectors.push_back(detector);
        }
        // have to try this on both rings, because of the case where both rings have the same phase
        // See Basic NEMA test
        for (int i = 0; i < 2; i++){
            if (vectorContainsPhase(rings[i], NEMAPhaseIndex)){
                getPhaseObj(NEMAPhaseIndex, -1)->setDetectors(detectors);
            }
        }
    }

    //Do not delete. SUMO traffic logic check.
    //SUMO check begin
    const SVCPermissions motorized = ~(SVC_PEDESTRIAN | SVC_BICYCLE);
    std::map<int, std::set<MSE2Collector*>> linkToDetectors;
    std::set<int> actuatedLinks;

    const int numLinks = (int)myLinks.size();
    std::vector<bool> neverMajor(numLinks, true);
    for (const MSPhaseDefinition* phase : myPhases) {
        const std::string& state = phase->getState();
        for (int i = 0; i < numLinks; i++) {
            if (state[i] == LINKSTATE_TL_GREEN_MAJOR) {
                neverMajor[i] = false;
            }
        }
    }
    std::vector<bool> oneLane(numLinks, false);
    for (int i = 0; i < numLinks; i++) {
        for (MSLane* lane : getLanesAt(i)) {
            int numMotorized = 0;
            for (MSLane* l : lane->getEdge().getLanes()) {
                if ((l->getPermissions() & motorized) != 0) {
                    numMotorized++;
                }
            }
            if (numMotorized == 1) {
                oneLane[i] = true;
                break;
            }
        }
    }

    for (const MSPhaseDefinition* phase : myPhases) {
        const int phaseIndex = (int)myDetectorForPhase.size();
        std::set<MSE2Collector*> detectors;
        if (phase->isActuated()) {
            const std::string& state = phase->getState();
            std::set<int> greenLinks;
            std::map<MSE2Collector*, std::set<int>> detectorLinks;

            for (int i = 0; i < numLinks; i++)  {
                if (state[i] == LINKSTATE_TL_GREEN_MAJOR
                        || (state[i] == LINKSTATE_TL_GREEN_MINOR
                            && ((neverMajor[i]  // check1a
                                 && hasMajor(state, getLanesAt(i))) // check1b
                                || oneLane[i])) // check1c
                   ) {
                    greenLinks.insert(i);
                    actuatedLinks.insert(i);
                }

                for (MSLane* lane : getLanesAt(i)) {
                    if (myLaneDetectorMap.count(lane) != 0) {
                        detectorLinks[myLaneDetectorMap[lane]].insert(i);
                    }
                }
            }
            for (auto& item : detectorLinks) {
                MSE2Collector* det = item.first;
                MSLane* detectorLane = myDetectorLaneMap[det];
                bool usable = true;
                // check 1
                for (int j : item.second) {
                    if (greenLinks.count(j) == 0) {
                        usable = false;
                    }
                }

                //check 2
                if (usable) {
                    for (MSLink* link : detectorLane->getLinkCont()) {
                        MSLane* next = link->getLane();
                        if (myLaneDetectorMap.count(next) != 0) {
                            MSE2Collector* nextDet = myLaneDetectorMap[next];
                            for (int j : detectorLinks[nextDet]) {
                                if (greenLinks.count(j) == 0) {
                                    usable = false;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (usable) {
                    detectors.insert(item.first);
                    for (int j : item.second) {
                        linkToDetectors[j].insert(item.first);
                    }
                }
            }
            if (detectors.size() == 0) {
                WRITE_WARNINGF("At NEMA tlLogic '%', actuated phase % has no controlling detector", getID(), toString(phaseIndex));
            }
        }
        std::vector<DetectorInfo*> detectorInfos;
        myDetectorForPhase.push_back(detectorInfos);
        for (MSE2Collector* det : detectors) {
            for (DetectorInfo& detInfo : myDetectorInfoVector) {
                if (detInfo.det == det) {
                    myDetectorForPhase.back().push_back(&detInfo);
                    detInfo.servedPhase[phaseIndex] = true;
                }
            }
        }
    }

    for (int i : actuatedLinks) {
        if (linkToDetectors[i].size() == 0 && myLinks[i].size() > 0
                && (myLinks[i].front()->getLaneBefore()->getPermissions() & motorized) != 0) {
            WRITE_WARNINGF("At NEMA tlLogic '%, linkIndex % has no controlling detector", getID(), toString(i));
        }
    }

#ifdef DEBUG_NEMA
    //std::cout << "reach the end of init()\n";
#endif
}

void
NEMALogic::validate_timing() {
    //check cycle length
    for (int ringIndex = 0; ringIndex < 2; ringIndex++){
        // TS2 Force Offs don't go in order, so using a different method to check cycle time
        SUMOTime cycleLengthCalculated = 0;
        for (auto &p : getPhasesByRing(ringIndex)){
            cycleLengthCalculated += (p->maxDuration + p->yellow + p->red);
        }
        if (coordinateMode && cycleLengthCalculated != myCycleLength){
            int ringNumber = ringIndex + 1;
            const std::string error = "At NEMA tlLogic '" + getID() + "', Ring " + toString(ringNumber) + " does not add to cycle length.";
            if (ignoreErrors) {
                WRITE_WARNING(error);
            } else {
                throw  ProcessError(error);
            }
        }
    }
    // check that the barriers sum together
    SUMOTime cycleLengths[2][2] = {{0, 0}, {0, 0}};
    for (int ringIndex = 0; ringIndex < 2; ringIndex++){
        // TS2 Force Offs don't go in order, so using a different method to check cycle time
        for (auto &p : getPhasesByRing(ringIndex)){
            cycleLengths[ringIndex][p->barrierNum] += p->maxDuration + p->yellow + p->red;
        }
    }
    // Write warnings if the barrers do not sum
    for (int barrierNum = 0; barrierNum < 2; barrierNum++){
        if (cycleLengths[0][barrierNum] != cycleLengths[1][barrierNum]) {
            const std::string error = "At NEMA tlLogic '" + getID() + "', the phases before barrier " + toString(barrierNum) + " from both rings do not add up. (ring1="
                                    + toString(cycleLengths[0][barrierNum]) + ", ring2=" + toString(cycleLengths[1][barrierNum]) + ")";
            if (coordinateMode && !ignoreErrors) {
                throw  ProcessError(error);
            } else {
                WRITE_WARNING(error);
            }
        }
    }
    // no offset for non coordinated
    if (!coordinateMode && offset != 0) {
        WRITE_WARNINGF("NEMA tlLogic '%' is not coordinated but an offset was set.", getID());
    }
}

void
NEMALogic::setNewSplits(std::vector<double> newSplits) {
    assert(newSplits.size() == 8);
    for (auto &p : myPhaseObjs){
        if (newSplits[p->phaseName - 1] > 0){
            p->nextMaxDuration = TIME2STEPS(newSplits[p->phaseName - 1]) - p->yellow - p->red;
        }
    }
}


void
NEMALogic::setNewMaxGreens(std::vector<double> newMaxGreens) {
    for (auto &p : myPhaseObjs){
        if (newMaxGreens[p->phaseName - 1] > 0){
            p->nextMaxDuration = TIME2STEPS(newMaxGreens[p->phaseName - 1]);
        }
    }
}


void
NEMALogic::setNewCycleLength(double newCycleLength) {
    myNextCycleLength = TIME2STEPS(newCycleLength);
}


void
NEMALogic::setNewOffset(double newOffset) {
    myNextOffset = TIME2STEPS(newOffset);
}


std::vector<int> NEMALogic::readParaFromString(std::string s) {
    std::vector<int> output;
    for (char c : s) {
        if (c >= '0' && c <= '9') {
            int temp = c - '0';
            output.push_back(temp);
        }
    }
    return output;
}

std::vector<std::string> NEMALogic::string2vector(std::string s) {
    std::vector<std::string> output;
    std::stringstream ss(s);
    while (ss.good()) {
        std::string substr;
        std::getline(ss, substr, ',');
        output.push_back(substr);
    }
    return output;
}

std::string NEMALogic::combineStates(std::string state1, std::string state2) {
    std::string output = "";
    if (state1.size() != state2.size()) {
        throw ProcessError("At NEMA tlLogic '" + getID() + "', different sizes of NEMA phase states. Please check the NEMA XML");
    }
    for (int i = 0; i < (int)state1.size(); i++) {
        char ch1 = state1[i];
        char ch2 = state2[i];

        // check through this order. 'G' overwrite 'g'.
        if (ch1 == 'G' || ch2 == 'G') {
            output += 'G';
        } else if (ch1 == 'g' || ch2 == 'g') {
            output += 'g';
        } else if (ch1 == 's' || ch2 == 's') {
            output += 's';
        } else if (ch1 == 'y' || ch2 == 'y') {
            output += 'y';
        } else if (ch1 == 'u' || ch2 == 'u') {
            output += 'u';
        } else if (ch1 == 'O' || ch2 == 'O') {
            output += 'O';
        } else if (ch1 == 'o' || ch2 == 'o') {
            output += 'o';
        } else {
            output += 'r';
        }
    }
    return output;
}

const MSPhaseDefinition&
NEMALogic::getCurrentPhaseDef() const {
    return myPhase;
}

int NEMALogic::measureRingDistance(int p1, int p2, int ringNum){
        int length = (int)rings[ringNum].size();
        int d = 0;
        bool found = false;        
        for (int i = 0; i < (length * 2); i++){
            if (rings[ringNum][i % length] != 0){ 
                if (found){
                   d++;
                   if (rings[ringNum][i % length] == p2){
                       break;
                   } 
                }
                else if (rings[ringNum][i % length] == p1){
                   found = true;
                } 
            }
        }
        assert(d > 0);
        return d;
}


//b should be the base of mode
SUMOTime 
NEMALogic::ModeCycle(SUMOTime a, SUMOTime b) {
    SUMOTime c = a - b;
    while (c >= b) {
        c = c - b;
    }
    while (c < 0) { //should be minimum green (or may be  not)
        c += b;
    }
    return c;
}


std::set<std::string> NEMALogic::getLaneIDsFromNEMAState(std::string state) {
    std::set<std::string> output;
    const MSTrafficLightLogic::LinkVectorVector& linkV = MSNet::getInstance()->getTLSControl().get(myID).getActive()->getLinks();
    for (int i = 0; i < (int)state.size(); i++) {
        char ch = state[i];
        if (ch == 'G') {
            for (auto link : linkV[i]) {
                output.insert(link->getLaneBefore()->getID());
            }
        }
    }
    return output;
}

bool NEMALogic::isLeftTurnLane(const MSLane* const lane) const {
    const std::vector<MSLink*> links = lane->getLinkCont();
    if (links.size() == 1 && links.front()->getDirection() == LinkDirection::LEFT) {
        return true;
    }
    return false;
}

bool
NEMALogic::hasMajor(const std::string& state, const LaneVector& lanes) const {
    for (int i = 0; i < (int)state.size(); i++) {
        if (state[i] == LINKSTATE_TL_GREEN_MAJOR) {
            for (MSLane* cand : getLanesAt(i)) {
                for (MSLane* lane : lanes) {
                    if (lane == cand) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


void
NEMALogic::activateProgram() {
    MSTrafficLightLogic::activateProgram();
    for (auto& item : myLaneDetectorMap) {
        item.second->setVisible(true);
    }
}

void
NEMALogic::deactivateProgram() {
    MSTrafficLightLogic::deactivateProgram();
    for (auto& item : myLaneDetectorMap) {
        item.second->setVisible(false);
    }
}

void
NEMALogic::setShowDetectors(bool show) {
    myShowDetectors = show;
    for (auto& item : myLaneDetectorMap) {
        item.second->setVisible(myShowDetectors);
    }
}

int NEMALogic::string2int(std::string s) {
    std::stringstream ss(s);
    int ret = 0;
    ss >> ret;
    return ret;
}


const std::string
NEMALogic::getParameter(const std::string& key, const std::string defaultValue) const {
    if (StringUtils::startsWith(key, "NEMA.")) {
        if (key == "NEMA.phaseCall") {
            int i = 0;
            std::string out_str = "";
            for (auto &p: myPhaseObjs){
                if (i < 1){
                    out_str += std::to_string(p->detectActive());
                }else{
                    out_str+=",";
                }
                i++;
            }
            return out_str;
        } else {
            throw InvalidArgument("Unsupported parameter '" + key + "' for NEMA controller '" + getID() + "'");
        }
    } else {
        return Parameterised::getParameter(key, defaultValue);
    }
}


void
NEMALogic::setParameter(const std::string& key, const std::string& value) {
    if (StringUtils::startsWith(key, "NEMA.")) {
        if (key == "NEMA.splits" || key == "NEMA.maxGreens") {
            //splits="2.0 3.0 4.0 5.0 2.0 3.0 4.0 5.0"
            const std::vector<std::string>& tmp = StringTokenizer(value).getVector();
            if (tmp.size() != 8) {
                throw InvalidArgument("Parameter '" + key + "' for NEMA controller '" + getID() + "' requires 8 space-separated values");
            }
            std::vector<double> timing;
            for (const std::string& s : tmp) {
                timing.push_back(StringUtils::toDouble(s));
            }
            if (key == "NEMA.maxGreens") {
                setNewMaxGreens(timing);
            } else {
                setNewSplits(timing);
            }
        } else if (key == "NEMA.cycleLength") {
            setNewCycleLength(StringUtils::toDouble(value));
        } else if (key == "NEMA.offset") {
            setNewOffset(StringUtils::toDouble(value));
        } else {
            throw InvalidArgument("Unsupported parameter '" + key + "' for NEMA controller '" + getID() + "'");
        }
    }
    Parameterised::setParameter(key, value);
}

void
NEMALogic::error_handle_not_set(std::string param_variable, std::string param_name) {
    if (param_variable == "") {
        throw InvalidArgument("Please set " + param_name + " for NEMA tlLogic '" + getID() + "'");
    }
}

void
NEMALogic::calculateForceOffs170(){
    SUMOTime zeroTime[2] = {TIME2STEPS(0), TIME2STEPS(0)};
    for (int i = 0; i < 2; i++){
        SUMOTime runningTime = 0;
        for (auto &p: getPhasesByRing(i)){
            // this will loop the phases in order.
            // in 170, the cycle "starts" when the coordinated phase goes to yellow. 
            // See https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter6.htm 
            if (p->coordinatePhase){   
                zeroTime[i] = runningTime + p->maxDuration + p->getTransitionTime(this);
            }
            runningTime += p->maxDuration + p->getTransitionTime(this); 
            p->forceOffTime = runningTime - p->getTransitionTime(this);
            // TODO: Implement this completely.
            // p->greatestStartTime = p->forceOffTime - (p->coordinatePhase? p->maxDuration : p->minDuration);
            p->greatestStartTime = p->forceOffTime - p->minDuration;
        }
    }
    // find the minimum offset time and then subtract from everything, modecycling where negative
    SUMOTime minCoordYellow = MIN2(zeroTime[0], zeroTime[1]);
    for (auto &p : myPhaseObjs){
        p->forceOffTime = ModeCycle(p->forceOffTime - minCoordYellow, myCycleLength);
        p->greatestStartTime = ModeCycle(p->greatestStartTime - minCoordYellow, myCycleLength);
    }

    #ifdef DEBUG_NEMA
        std::ios_base::fmtflags oldflags = std::cout.flags();
        std::streamsize oldprecision = std::cout.precision();
        for (int i = 0; i < 2; i++){
            std::cout << "Ring" << i + 1 << " force offs: \t";
            for (auto &p : rings[i]){
                if (p > 0){
                    PhasePtr pObj = getPhaseObj(p, i);
                    std::cout << std::fixed << std::setprecision(2) << STEPS2TIME(pObj->forceOffTime) << "\t";
                }
                else{
                     std::cout << std::to_string(0) << "\t";
                }
            }
            std::cout << std::endl;
        }
        std::cout.flags (oldflags);
        std::cout.precision (oldprecision);
    #endif
}

void
NEMALogic::calculateForceOffsTS2(){
    // TS2 "0" cycle time is the start of the "first" coordinated phases.
    // We can find this "0" point by first constructing the forceOffs in sequential order via the 170 method 
    calculateForceOffs170();

    // Switch the Force Off Times to align with TS2 Cycle.
    // The coordinate phases will always be the defaultBarrierPhases[i][0]
    NEMAPhase* cPhase[2] = {defaultBarrierPhases[0][0], defaultBarrierPhases[1][0]};
    SUMOTime minCoordTime = MIN2(cPhase[0]->forceOffTime - cPhase[0]->maxDuration, cPhase[1]->forceOffTime - cPhase[1]->maxDuration);

    // loop through all the phases and subtract this minCoordTime to move the 0 point to the start of the first coordinated phase
    for (auto &p : myPhaseObjs){
        if ((p->forceOffTime - minCoordTime) >= 0){
            p->forceOffTime -= minCoordTime;
        }else {
            p->forceOffTime = (myCycleLength + (p->forceOffTime - minCoordTime));
        }
        p->greatestStartTime = ModeCycle(p->greatestStartTime - minCoordTime, myCycleLength);   
    }
}

void
NEMALogic::calculateInitialPhases170(){
    // This function sorts the phases by their max start time
    SUMOTime cycleTime = ModeCycle(getTimeInCycle(), myCycleLength);
    
    NEMAPhase* activePhases[2]; 

    for (int i = 0; i < 2; i++){
        std::vector<NEMAPhase*> ringCopy = getPhasesByRing(i);
        // sort by the minimum start time
        std::sort(ringCopy.begin(), ringCopy.end(), 
            [](NEMAPhase *p, NEMAPhase *p1) { return p->greatestStartTime <= p1->greatestStartTime; }
        );
        // try to find the hole that the current time fits in.
        bool found = false;
        for (auto &p: ringCopy){
            // This handles the wrap around
            SUMOTime syntheticPriorStart =  p->getSequentialPriorPhase()->greatestStartTime < p->greatestStartTime ? 
                p->getSequentialPriorPhase()->greatestStartTime : p->getSequentialPriorPhase()->greatestStartTime - myCycleLength;
            if (cycleTime <= p->greatestStartTime && cycleTime > syntheticPriorStart){
                found = true;
                activePhases[i] = p;
                break;
            }
        }
        if (!found){
            const std::string error = "I can't find the correct phase for NEMA tlLogic '" + getID() + "' Ring " + toString(i) + " to start in.";
            if (ignoreErrors) {
                WRITE_WARNING(error);
                WRITE_WARNING("I am starting in the coordinated phases");
                activePhases[0] = coordinatePhaseObjs[0];
                activePhases[1] = coordinatePhaseObjs[1];        
                break;
            } else {
                throw ProcessError(error);
            }
        }
    }

    // ensure that the two found phases are on the same side of the barrier
    if (activePhases[0]->barrierNum != activePhases[1]->barrierNum){
        // give preference to whatever is on the coordinate side of the barrier, one must be if they aren't equal to each other
        activePhases[0] = activePhases[0]->barrierNum == 0? activePhases[0] : defaultBarrierPhases[0][0];
        activePhases[1] = activePhases[1]->barrierNum == 0? activePhases[1] : defaultBarrierPhases[1][0]; 
    }

    // update the expected duration to be the time until their force off
    activePhases[0]->forceEnter(this);
    activePhases[1]->forceEnter(this);
}

void
NEMALogic::calculateInitialPhasesTS2(){
    // Modifications where made to 170 algorithm so that it works with both.
    calculateInitialPhases170();
}

SUMOTime
NEMALogic::coordModeCycle170(NEMAPhase* phase){
    return ModeCycle(myCycleLength - getTimeInCycle() - phase->yellow - phase->red, myCycleLength);  
}

SUMOTime
NEMALogic::coordModeCycleTS2(NEMAPhase* phase){
    // This puts the phase green for the rest of the cycle, plus the first bit in which it must be green
    // We don't need the yellow and red here because the force off already incorporates that.
    return ModeCycle((myCycleLength + phase->yellow) - getTimeInCycle(), myCycleLength);
}

std::vector<NEMAPhase *>
NEMALogic::getPhasesByRing(int ringNum){
    std::vector<NEMAPhase *> phases;
    for (auto &p : myPhaseObjs){
        if (p->ringNum == ringNum){
            phases.push_back(p);
        } 
    }
    return phases;
}

void 
NEMALogic::setActivePhase(PhasePtr phase){ 
    myActivePhaseObjs[phase->ringNum] = phase; 
};

std::map<std::string, double>
NEMALogic::getDetectorStates() const {
    std::map<std::string, double> result;
    for (auto item : myDetectorLaneMap) {
        result[item.first->getID()] = item.first->getCurrentVehicleNumber();
    }
    return result;
}

// ===========================================================================
// Phase Definitions
// =========================================================================== 

NEMAPhase::NEMAPhase(int phaseName, bool isBarrier, bool isGreenRest, bool isCoordinated,
                     bool minRecall, bool maxRecall, bool fixForceOff, int barrierNum, int ringNum,
                     MSPhaseDefinition* phase):
                phaseName(phaseName),
                isAtBarrier(isBarrier),
                isGreenRest(isGreenRest),
                barrierNum(barrierNum),
                coordinatePhase(isCoordinated),
                fixForceOff(fixForceOff),
                ringNum(ringNum),
                minRecall(minRecall),
                maxRecall(maxRecall),
                myCorePhase(phase)
                {
    // Public
    readyToSwitch = false;
    greenRestTimer = 0;
    forceOffTime = 0;

    // Private
    myInstance = this;
    myLastPhaseInstance = nullptr;
    sequentialPriorPhase = nullptr;
    myLightState = LightState::Red;
    transitionActive = false;
    
    // Timing Parameters
    maxGreenDynamic = myCorePhase->maxDuration;
    myStartTime = TIME2STEPS(0.);
    myExpectedDuration = myCorePhase->minDuration;
    myLastEnd = TIME2STEPS(0.);
}

// TODO: Do I need this?
NEMAPhase*
NEMAPhase::getInstance(void) {
    if (this != nullptr) {
        return this;
    }
    throw ProcessError("The phase has not been constructed yet");
}

NEMAPhase::~NEMAPhase(){
    // Delete the transitions from their alloc
    for (auto t : myTransitions) {
        delete t;
    }
}


void
NEMAPhase::init(NEMALogic* controller, int crossPhaseTarget, int crossPhaseSource, bool latching){
    // switch the durations from steps2time 
    recalculateTiming();
    
    for (auto p: controller->getPhasesByRing(ringNum)){
        // construct transitions for all potentail movements, including back to myself
        myTransitions.push_back(new PhaseTransitionLogic(this, p));
        myTransitions.back()->distance = controller->measureRingDistance(phaseName, p->phaseName, ringNum);
    }

    // sort the transitions by distance for speed later
    std::sort(myTransitions.begin(), myTransitions.end(), [&](const auto *i, const auto *j) { return i->distance < j->distance; });

    // create the phase detector info
    myDetectorInfo = phaseDetectorInfo(latching, 
        crossPhaseTarget > 0 ? controller->getPhaseObj(crossPhaseTarget) : nullptr, 
        crossPhaseSource > 0 ? controller->getPhaseObj(crossPhaseSource) : nullptr
    );
}

void
NEMAPhase::recalculateTiming(void){
    // This could be extended in the future to allow for traci manipulation
    yellow = myCorePhase->yellow;
    red = myCorePhase->red;
    minDuration = myCorePhase->minDuration;
    maxDuration = myCorePhase->maxDuration;
    nextMaxDuration = myCorePhase->maxDuration;
    maxGreenDynamic = myCorePhase->maxDuration;
    vehExt = myCorePhase->vehext;
}


std::string 
NEMAPhase::getNEMAState() {
    std::string newState = "";
    std::string curState = myCorePhase->getState();
    if (myLightState >= LightState::Green) {
        newState = curState;
    } else if (myLightState <= LightState::Red) {
        for (char ch : curState) {
            UNUSED_PARAMETER(ch);
            newState += 'r';
        }
    } else {
        // yellow
        for (char ch : curState) {
            if (ch == 'G' || ch == 'g') {
                newState += 'y';
            } else {
                newState += ch;
            }
        }
    }
    return newState;
}

void
NEMAPhase::clearMyDetectors(){
    if ((!myDetectorInfo.latching) || (myLightState < LightState::Green)){
        myDetectorInfo.detectActive = false;
    } 
}

void
NEMAPhase::checkMyDetectors(){
    // Check my Detectors, only necessary if it isn't currently marked as on
    if (!myDetectorInfo.detectActive){
        // If I have a cross phase target and it is active and I am not, save my detector as not active
        if (myDetectorInfo.cpdTarget != nullptr){
            if (myDetectorInfo.cpdTarget->getCurrentState() >= LightState::Green){
                if (myLightState < LightState::Green){
                    myDetectorInfo.detectActive = false;
                    return;
                }
            } 
        }
        // If we make it to this point, check my detector like normal.
        for (auto& d: myDetectorInfo.detectors){
            if (d->getCurrentVehicleNumber() > 0){
                myDetectorInfo.detectActive = true;
                return;
            }
        }
        // If my detector is not active, check my cross phase 
        if ((myDetectorInfo.cpdSource != nullptr) && (myLightState >= LightState::Green)){
            if (myDetectorInfo.cpdSource->getCurrentState() < LightState::Green){
                for (auto& d: myDetectorInfo.cpdSource->getDetectors()){
                    if (d->getCurrentVehicleNumber() > 0){
                        myDetectorInfo.detectActive = true;
                        return;
                    }
                }       
            }
        }
    }
}

void
NEMAPhase::enter(NEMALogic* controller, NEMAPhase* lastPhase){
    myStartTime = controller->getCurrentTime();
    myLightState = LightState::Green;
    myLastPhaseInstance = lastPhase;
    readyToSwitch = false;

    // Handle Green Rest Peculiarities
    if  (!controller->coordinateMode && isGreenRest){
        // If the controller is in free mode and the phase is a green rest phase, then it should enter as "green rest"
        myLightState = LightState::GreenRest;
        // if the phase has "green rest" capabilities, set it's timer to the dynamic maxGreen  
        greenRestTimer = maxDuration * isGreenRest;
    }

    // clear the last transition decision
    lastTransitionDecision = nullptr;

    // Calculate the Max Green Time & Expected Duration here:
    if (controller->coordinateMode){
        if (coordinatePhase){
            myExpectedDuration = controller->ModeCycle(forceOffTime - controller->getTimeInCycle(), controller->getCurrentCycleLength());
            // assert(myExpectedDuration >= maxDuration);
        } else {
            maxGreenDynamic = controller->ModeCycle(forceOffTime - controller->getTimeInCycle(), controller->getCurrentCycleLength());
            if (!fixForceOff){
                maxGreenDynamic = MIN2(maxDuration, maxGreenDynamic);
            }
            myExpectedDuration = minDuration;
        }
    } 
    else {
        myExpectedDuration = minDuration;
    }
    // Implements the maxRecall functionality
    if (maxRecall && !coordinatePhase){
        myExpectedDuration = maxGreenDynamic;
    }
    // Set the controller's active phase
    controller -> setActivePhase(this);
}

void
NEMAPhase::exit(NEMALogic* controller, PhaseTransitionLogic* nextPhases[2]){
    // At the first entry to this transition, the phase will be in green
    if (nextPhases[ringNum]->getToPhase() != this){
        lastTransitionDecision = nextPhases[ringNum];
        if (myLightState >= LightState::Green){
            myLastEnd = controller->getCurrentTime();
            myLightState = LightState::Yellow;
            transitionActive = true;
        }
        else {
            if (controller->getCurrentTime() - myLastEnd >= (yellow + red)){
                // triggers the entry to the next target phase.
                readyToSwitch = false;
                transitionActive = false;
                // Enter into the next phase, setting it to Green!
                nextPhases[ringNum]->getToPhase()->enter(controller, this);
            } else if (controller->getCurrentTime() - myLastEnd >= yellow){
                myLightState = LightState::Red;
            }
        }
    } else {
        // This is the entry to green rest or green transfer
        NEMAPhase* otherPhase = controller->getActivePhase(!ringNum);
        readyToSwitch = false;
        if ((nextPhases[!ringNum]->getToPhase() == otherPhase && otherPhase->readyToSwitch)
            // if the other phase is already in green rest and I am in green transfer but there are no cars on the side streets, 
            // I should default to being in green rest. Side street check is done by looking at the green rest timer. 
            || (otherPhase->greenRestTimer >= otherPhase->maxDuration && otherPhase->getCurrentState() == LightState::GreenRest)){
            assert(isGreenRest);
            myLightState = LightState::GreenRest;
            // set the start time to be current time - the minimum timer
            myStartTime = controller->getCurrentTime() - minDuration;
            myExpectedDuration = minDuration;
            // if the phase has "green rest" capabilities, set it's timer to the dynamic maxGreen  
            greenRestTimer = maxDuration * isGreenRest;
        } else {
            myLightState = LightState::GreenXfer;
            // In green transfer, the phase will last as long as the other phase.
            myExpectedDuration = otherPhase->myExpectedDuration;
        }
    }
}

SUMOTime
NEMAPhase::getTransitionTime(NEMALogic* controller){
    if (!transitionActive){
        return (yellow + red);
    } 
    return MAX2(TIME2STEPS(0), ((controller->getCurrentTime() - myLastEnd) - (yellow + red)));  
}

SUMOTime
NEMAPhase::calcVehicleExtension(SUMOTime duration){
    if (myExpectedDuration < maxGreenDynamic && myDetectorInfo.detectActive){
        return MIN2(MAX2(duration + vehExt, minDuration), maxGreenDynamic);
    }
    return myExpectedDuration;
}

void
NEMAPhase::update(NEMALogic* controller){
    // If I am in a transition, the rest of the update logic does not matter
    if (myLightState < LightState::Green){
        readyToSwitch = true;
        return ;
    }
    // Continuation Logic
    // readyToSwitch = false;
    SUMOTime duration = controller->getCurrentTime() - myStartTime;
    // Check the vehicle extension timer as long as not in green transfer and not a coordinated phase
    if (myLightState != LightState::GreenXfer && !coordinatePhase){
        myExpectedDuration = calcVehicleExtension(duration);
    }
    // Special Logic for Green Rest, which behaves uniquely
    if (myLightState == LightState::GreenRest){
        // check all other detectors and see if anything else is active. If so, 
        // start the green rest timer countdown, which is == to the max duration of the phase
        bool vehicleActive = false;
        for (auto &p : controller->getPhaseObjs()){
            if ((p->phaseName != phaseName) 
                && (p->phaseName != controller->getActivePhase(!ringNum)->phaseName) 
                && p->myDetectorInfo.detectActive){
                greenRestTimer -= DELTA_T;
                vehicleActive = true;
                break;
            }
        }
        if (!vehicleActive){
            greenRestTimer = maxDuration;
            if (duration >= minDuration){
                myStartTime = controller->getCurrentTime() - minDuration;
                maxGreenDynamic = minDuration + maxDuration;
                myExpectedDuration = minDuration + MAX2(TIME2STEPS(0), myExpectedDuration - duration);
            }
        }

        if (greenRestTimer < DELTA_T){
            readyToSwitch = true;
            // force the counterparty to be ready to switch too. This needs to be latching....
            controller->getActivePhase(!ringNum)->readyToSwitch = true;
        }

        // Special Behavior when the Green Rest Circles all the way around in coordinated mode
        if (coordinatePhase){
            // This means that we have green rested until I should "start" again. Just call my entry function again.
            if (controller->getTimeInCycle() <= (forceOffTime - maxDuration + DELTA_T / 2)){
                enter(controller, this);
            }
        }
    }
    // Check to see if a switch is desired
    if (duration >= myExpectedDuration){
        readyToSwitch = true;
    } 
}


PhaseTransitionLogic*
NEMAPhase::getTransition(int toPhase){
    for (auto t: myTransitions){
        if (t->getToPhase()->phaseName == toPhase){
            return t;
        }
    }
    // This point should never be reached
    assert(0);
    // To satisfy the compiler and return value from all control paths
    return myTransitions.front();
}

std::vector<PhaseTransitionLogic*>
NEMAPhase::trySwitch(NEMALogic* controller){
    std::vector<PhaseTransitionLogic *> nextTransitions;
    if (readyToSwitch){
        // only try to switch if I am ready to switch
        for (auto &t: myTransitions){
            if (t->okay(controller)){
                nextTransitions.push_back(t);
                // break once there is a valid option (they have already been sorted)
                break;
            }
        }   
    }
    // Give preference to the last transition decision, 
    // but only need to add it if it is not in the list AND if nothing in the list is the same barrier as it was.
    if (lastTransitionDecision != nullptr){
        bool found = false;
        bool sameBarrier = false;
        for (auto &t: nextTransitions){
            if (t == lastTransitionDecision){
                found = true;
                break;
            }
            if (t->getToPhase()->barrierNum == lastTransitionDecision->getToPhase()->barrierNum){
                sameBarrier = true;
            }
        }
        if (!found && !sameBarrier){
            nextTransitions.push_back(lastTransitionDecision);
        }
    }
    // Add the transition back to myself, but only in the case when no others have been added
    if (nextTransitions.size() < 1){
        nextTransitions.push_back(myTransitions.back());
    }

    return nextTransitions;
}

// ===========================================================================
// PhaseTransitionLogic Definitions
// =========================================================================== 

PhaseTransitionLogic::PhaseTransitionLogic (
    NEMAPhase* fromPhase, NEMAPhase* toPhase): 
        distance(0),
        fromPhase(fromPhase),
        toPhase(toPhase)
{} 

PhaseTransitionLogic::~PhaseTransitionLogic(){};

bool
PhaseTransitionLogic::okay(NEMALogic* controller){
    if (fromPhase == toPhase){
        // for green rest or green transfer, it cannot return to itself if a transition is active
        return fromPhase->getCurrentState() >= LightState::Green;
    }
    else if (fromPhase->coordinatePhase){
        return fromCoord(controller);
    } 
    else if (fromPhase->isAtBarrier){
        return fromBarrier(controller);
    }
    else if (controller->coordinateMode){
        // typical coordinate mode transition
        return coordBase(controller);
    }
    else {
        return freeBase(controller);
    }
}

bool
PhaseTransitionLogic::freeBase(NEMALogic* controller){
    // Simplest transition logic. Just check if a detector (or recall) is active on that phase and 
    bool okay = false;
    if (toPhase->callActive()){
        if (fromPhase->barrierNum != toPhase->barrierNum){
            PhasePtr otherPhase = controller->getActivePhase(!fromPhase->ringNum);
            // If it is a barrier cross, the cross is only allowed if the transition times are ==
            if (otherPhase->readyToSwitch && otherPhase->getTransitionTime(controller) == fromPhase->getTransitionTime(controller)){
                okay = true;
            }        
        } else {
            okay = true;
        }
    }
    return okay;
}

bool
PhaseTransitionLogic::coordBase(NEMALogic* controller){
    if (toPhase->coordinatePhase && 
        (controller->getActivePhase(!fromPhase->ringNum)->readyToSwitch || fromPhase->barrierNum == toPhase->barrierNum)){
        // transitions TO the coordinated phase may always happen, as long as the other phase is okay to switch too
        return true;
    }
    // first check if the free logic is upheld 
    else if (freeBase(controller)){
        // Then check if the "to phase" can fit, which means that there is enough time to fit the current transition + the minimum time of the next phase 
        SUMOTime transitionTime = fromPhase->getTransitionTime(controller);
        SUMOTime timeTillForceOff = controller->ModeCycle(toPhase->forceOffTime - controller->getTimeInCycle(), controller->getCurrentCycleLength());
        if (toPhase->minDuration + transitionTime <= timeTillForceOff){
            return true;
        }
    }
    return false;
}


bool
PhaseTransitionLogic::fromBarrier(NEMALogic* controller){
    if (freeBase(controller)){
        if (fromPhase->barrierNum == toPhase->barrierNum){
            // same barrier side so we are good.
            // Check if green transfer is active. If so, we need to make sure that there are no calls on the other side of the barrier
            if (fromPhase->getCurrentState() >= LightState::Green){
                for (auto &p : controller->getPhasesByRing(fromPhase->ringNum)){
                    if (p->barrierNum != fromPhase->barrierNum && p->callActive()){
                        return false;
                    }
                }
            } 
            return true;
        } else {
            // This is now a barrier cross and we need to make sure that the other phase is also ready to transition
            if (fromPhase->readyToSwitch && controller->getActivePhase(!(fromPhase->ringNum))->readyToSwitch){
                return true;
            }
        }
    }
    return false;
}


bool
PhaseTransitionLogic::fromCoord(NEMALogic* controller){
    if (coordBase(controller)){
        // Determine if the other phase is also ready to switch
        // TODO: Do I need to ensure that they have ended at the same time here?  
        if (controller->getActivePhase(!(fromPhase->ringNum))->readyToSwitch){
            // Dr. Wang had the Type-170 code setup in a way that it could transition whenever - meaning that it didn't matter if the prior phase could fit or not
            if (controller->isType170()){
                return true;
            }
            // now determine if there my prior phase can fit or not. We already know that I can fit.
            NEMAPhase* priorPhase = toPhase->getSequentialPriorPhase();
            SUMOTime timeTillForceOff = controller->ModeCycle(priorPhase->forceOffTime - controller->getTimeInCycle(), controller->getCurrentCycleLength());
            SUMOTime transitionTime = fromPhase->getTransitionTime(controller);
            // if the time till the force off is less than the min duration || 
            // if it is greater than the cycle length minus the length of the coordinate phase (which the fromPhase automatically is)
            if ((priorPhase->minDuration + transitionTime) > timeTillForceOff || timeTillForceOff > (controller->getCurrentCycleLength() - fromPhase->minDuration)){
                return true;
            }
        }
    }
    return false;
}


// ===========================================================================
// NEMALogic Definitions
// =========================================================================== 
NEMAPhase*
NEMALogic::getPhaseObj(int phaseNum, int ringNum){
    // This satisfies the case where there is a "duplicate" phase on both ring
    // see basic example in NEMA tests
    std::vector<PhasePtr> iterRing = ringNum >= 0 ? getPhasesByRing(ringNum) : myPhaseObjs;  
    for (auto &p : iterRing){
        if (p->phaseName == phaseNum){
            return p;
        }
    }
    assert(0);
    // To satisfy the compiler
    return myPhaseObjs.front();
}


void
NEMALogic::getNextPhases(TransitionPairs &transitions){
        std::vector<std::vector<PhaseTransitionLogic* >> potentialPhases;
        // Get an Array of Potential Transitions
        for (const auto &p : myActivePhaseObjs){
            potentialPhases.push_back(p->trySwitch(this));
        }
        // Loop through all combination of transitions, keeping only the valid ones.
        for (const auto &r1_t : potentialPhases[0]){
            for (const auto &r2_t : potentialPhases[1]){
                if (r1_t->getToPhase()->barrierNum == r2_t->getToPhase()->barrierNum){
                    transitions.push_back({r1_t, r2_t, (float)(r1_t->distance + r2_t->distance) / 2});
                } 
                else {
                    // If the rings are different, add a choice where one of them is the default choice for whatever ring it is
                    // create two choices, one for each of the phase as they are on  different rings
                    if (r1_t->getFromPhase()->readyToSwitch){
                        // get the r2 default 
                        PhaseTransitionLogic* r2_t_temp = r2_t->getFromPhase()->readyToSwitch? 
                            myActivePhaseObjs[1]->getTransition(defaultBarrierPhases[1][r1_t->getToPhase()->barrierNum]->phaseName) : r2_t;

                        // only add it if it does not cross a barrier!
                        if (r2_t_temp->getToPhase()->barrierNum == r1_t->getToPhase()->barrierNum){
                            transitions.push_back({r1_t, r2_t_temp, (float)(r2_t_temp->distance + r1_t->distance) / 2});
                        }
                    }

                    if (r2_t->getFromPhase()->readyToSwitch){
                        // R1 default 
                        PhaseTransitionLogic* r1_t_temp = r1_t->getFromPhase()->readyToSwitch? 
                            myActivePhaseObjs[0]->getTransition(defaultBarrierPhases[0][r2_t->getToPhase()->barrierNum]->phaseName) : r1_t;

                        // only add it if it does not cross a barrier!
                        if (r1_t_temp->getToPhase()->barrierNum == r2_t->getToPhase()->barrierNum){
                            transitions.push_back({r1_t_temp, r2_t, (float)(r2_t->distance + r1_t_temp->distance) / 2});
                        }
                    }
                    // If the distances are <= 1, this means that this is the shortest transition possible 
                    // and we should break without considering the other options.  
                    if (!transitions.empty()){
                        if (transitions.back().distance < 1){
                            return;
                        }   
                    }
                }
        } 
    }
}


std::string
NEMALogic::composeLightString(){
    // Construct the Phase String
    std::string state[2] = {"", ""};
    for (int i = 0; i < 2; i ++) {
        state[i] = myActivePhaseObjs[i]->getNEMAState();
    }
    return combineStates(state[0], state[1]);
}


SUMOTime
NEMALogic::trySwitch(){
    PhaseTransitionLogic* nextPhases[2] = {nullptr, nullptr};

    // update the internal time. This is a must.
    setCurrentTime();
    
    // Check the Detectors
    for (auto &p : myPhaseObjs){
        p->checkMyDetectors();
    }

    // Update the timing parameters
    for (const auto& p : myActivePhaseObjs){
        p->update(this);
    }

    // Calculate the Next Phases, but only if one or the other is ready to transition
    if (myActivePhaseObjs[0]->readyToSwitch || myActivePhaseObjs[1]->readyToSwitch){
        TransitionPairs transitions;
        getNextPhases(transitions);
        // Sort the next phases and select the closest.
        if (transitions.size() > 0){
            std::sort(transitions.begin(), transitions.end(), 
            [](const auto &i, const auto &j) { return i.distance < j.distance; });
        }
        // Set the Next Phases = to the shortest distance
        nextPhases[0] = transitions.front().p1;
        nextPhases[1] = transitions.front().p2;
        // Try the exit logic. This doesn't necessarily mean that the phase will exit, 
        // as it could go into green rest or green transfer, but this is considered an "exit"
        for (const auto& p : myActivePhaseObjs){
            if (p->readyToSwitch){
                p->exit(this, nextPhases);
            }
        }
        // Implement Traci Updates on the start of ring1 coordinated phase
        if (myActivePhaseObjs[0]->phaseName == defaultBarrierPhases[0][0]->phaseName){
            implementTraciChanges();
        }
        // This is the only time when something might have happened, so we update the phase strings here
        std::string newState = composeLightString(); 
        if (newState != myPhase.getState()) {
            myPhase.setState(newState);
            myPhase.setName(toString(myActivePhaseObjs[0]->phaseName) + "+" + toString(myActivePhaseObjs[1]->phaseName));
            // ensure that SwitchCommand::execute notices a change
            myStep = 1 - myStep;
        }
    }

    // Clear my detectors
    for (auto &p : myPhaseObjs){
        p->clearMyDetectors();
    }

    // Basic Assertion to ensure that the Barrier is not crossed
    assert(myActivePhaseObjs[0]->barrierNum == myActivePhaseObjs[1]->barrierNum);
    return DELTA_T;
}


void
NEMALogic::implementTraciChanges(void){
    for (auto &p : myPhaseObjs){
        p->maxDuration = p->nextMaxDuration;
    }
    offset = myNextOffset;
    myCycleLength = myNextCycleLength;
}