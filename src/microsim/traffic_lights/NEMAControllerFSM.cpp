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
#include "NEMAControllerFSM.h"


// ===========================================================================
// parameter defaults definitions
// ===========================================================================
#define INVALID_POSITION std::numeric_limits<double>::max() // tl added

// #define DEBUG_NEMA

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
    myCycleLength = (StringUtils::toDouble(getParameter("total-cycle-length", getParameter("cycle-length", getParameter(toString(SUMO_ATTR_CYCLETIME), "60")))));
    myNextCycleLength = myCycleLength;
    myDefaultCycleTime = TIME2STEPS(myCycleLength);
    myShowDetectors = StringUtils::toBool(getParameter("show-detectors", toString(OptionsCont::getOptions().getBool("tls.actuated.show-detectors"))));
    myFile = FileHelpers::checkForRelativity(getParameter("file", "NUL"), basePath);
    myFreq = TIME2STEPS(StringUtils::toDouble(getParameter("freq", "300")));
    myVehicleTypes = getParameter("vTypes", "");
    myCabinetType = parseCabinetType(getParameter("cabinetType", "Type170"));
    ignoreErrors = StringUtils::toBool(getParameter("ignore-errors", "false"));
    
    // TODO: Create a parameter for this
    cycleRefPoint = 0;
    

    // for (int i = 0; i < (int)VecMaxRecall.size(); i++) {
    //     maxRecalls[VecMaxRecall[i] - 1] = true;
    //     recall[VecMaxRecall[i] - 1] = true;
    // }

#ifdef DEBUG_NEMA
    std::cout << "minRecall: ";
    for (int i = 0; i < 8; i++) {
        std::cout << minRecalls[i] << '\t';
    }
    std::cout << std::endl;

    std::cout << "maxRecall: ";
    for (int i = 0; i < 8; i++) {
        std::cout << maxRecalls[i] << '\t';
    }
    std::cout << std::endl;
#endif
    std::string barriers = getParameter("barrierPhases", "");
    std::string coordinates = getParameter("coordinatePhases", getParameter("barrier2Phases", ""));
    std::string ring1 = getParameter("ring1", "");
    std::string ring2 = getParameter("ring2", "");
    
    fixForceOff = StringUtils::toBool(getParameter("fixForceOff", "false"));
    offset = STEPS2TIME(_offset);
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
    std::cout << "cycleLength = " << myCycleLength << std::endl;
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

NEMALogic::~NEMALogic() { }

void
NEMALogic::constructTimingAndPhaseDefs(std::string &barriers, std::string &coordinates, std::string &ring1, std::string &ring2){
    //init barriers
    std::vector<int> barrierPhases = readParaFromString(barriers);
    std::vector<int> coordinatePhases = readParaFromString(coordinates);
    
    // create the rings object with the 
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
            std::cout << minRecalls[i] << '\t';
        }
        std::cout << std::endl;

        std::cout << "maxRecall: ";
        for (int i = 0; i < 8; i++) {
            std::cout << maxRecalls[i] << '\t';
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
                // TODO: Determine if this is the proper logic
                bool greenRest = vectorContainsPhase(coordinatePhases, p) && myCabinetType == TS2;
                // could add per-phase fixforceoff here
                // barrierNum is either 0 or 1, depending on mainline side or sidestreet
                int barrierNum = ringIter % 2;

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
            // Set the first to point to the last, wrapping around the ring.
            myPhaseObjs[lastPhaseIter == 0? 0 : lastPhaseIter + 1]->setSequentialPriorPhase(myPhaseObjs[lastPhaseIter + phaseIter]);
            // index the ring counter
            ringIter++;
        }
        ringNum++;
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
    } 



#ifdef DEBUG_NEMA
    //print to check the rings and barriers active phase
    std::cout << "After init, active ring1 phase is " << activeRing1Phase << std::endl;
    std::cout << "After init, active ring2 phase is " << activeRing2Phase << std::endl;


    //print to check the phase definition is correct
    std::cout << "Print to check NEMA phase definitions\n";
    for (auto p : myPhases) {
        std::cout << "index = " << p->getName() << "; ";
        std::cout << "duration (useless) = " << time2string(p->duration) << "; ";
        std::cout << "minDur = " << time2string(p->minDuration) << "; ";
        std::cout << "maxDur = " << time2string(p->maxDuration) << "; ";
        std::cout << "vehext = " << time2string(p->vehext) << "; ";
        std::cout << "yellow = " << time2string(p->yellow) << "; ";
        std::cout << "red = " << time2string(p->red) << "; ";
        std::cout << "state = " << p->getState() << std::endl;
    }
#endif

    R1State = activeRing1Phase;
    R2State = activeRing2Phase;

    // set the next phase to current for initialization
    myNextPhaseR1 = R1State;
    myNextPhaseR2 = R2State;
    myNextPhaseR1Distance = 0;
    myNextPhaseR2Distance = 0;

    // std::cout << "After init, R1State = " << R1State << std::endl;
    // std::cout << "After init, R2State = " << R2State << std::endl;

    R1RYG = GREEN;
    R2RYG = GREEN;
    
    wait4R1Green = false;
    wait4R2Green = false;


#ifdef DEBUG_NEMA
    std::cout << "After init, r1/r2 barrier phase = " << r1barrier << " and " << r2barrier << std::endl;
    std::cout << "After init, r1/r2 coordinate phase = " << r1coordinatePhase << " and " << r2coordinatePhase << std::endl;
#endif


    currentState = "";
    // currentR1State = myPhases[R1State - 1]->getState();
    // currentR2State = myPhases[R2State - 1]->getState();
    for (const MSPhaseDefinition* const p : myPhases) {
        if (R1State == string2int(p->getName())) {
            currentR1State = p->getState();
        }
        if (R2State == string2int(p->getName())) {
            currentR2State = p->getState();
        }
    }

#ifdef DEBUG_NEMA
    std::cout << "R1State = " << R1State << " and its state = " << currentR1State << std::endl;
    std::cout << "R2State = " << R2State << " and its state = " << currentR2State << std::endl;
#endif
        // Initial Phases
    std::string state1 = transitionState(currentR1State, GREEN);
    std::string state2 = transitionState(currentR2State, GREEN);
    myPhase.setState(combineStates(state1, state2));
    myPhase.setName(toString(activeRing1Phase) + "+" + toString(activeRing2Phase));
    myNextPhaseR1 = 0;
    myNextPhaseR2 = 0;

    // myStep Should Start at 0
    myStep = 0;

    //validating timing
    validate_timing();
}

void
NEMALogic::createDetectorObjects(){
    // Create vector of latching detectors
    IntVector latchingDetectors = readParaFromString(getParameter("latchingDetectors", ""));
    
    // construct the phaseDetectorMapping. In the future this could hold more parameters, such as lock in time or delay
    for (int i = 0; i < 2; i++){
        auto local_ring = i < 1? ring1: ring2;
        for (auto p: readParaFromString(local_ring)){
            if (p > 0){
                bool latching = false;
                std::string cps = "crossPhaseSwitching:";
                int crossPhase = StringUtils::toInt(getParameter(cps.append(std::to_string(p)), "0"));
                if (std::find(latchingDetectors.begin(), latchingDetectors.end(), p) != latchingDetectors.end()) {
                    latching = true;
                }
                phase2DetectorMap[p] = phaseDetectorInfo(crossPhase, latching);
            }
        }
    }

    // Construct the Cross Mapping
    for (auto &phaseDetectInfo : phase2DetectorMap) {
        if (phaseDetectInfo.second.cpdSource > 0) {
            // WRITE_WARNING(error);
            // TODO: Handle 
            if (phase2DetectorMap.find(phaseDetectInfo.second.cpdSource) != phase2DetectorMap.end()){    
                phase2DetectorMap.find(phaseDetectInfo.second.cpdSource) -> second.cpdTarget = phaseDetectInfo.first;
            } else {
                phaseDetectInfo.second.cpdSource = 0;
                std::string msg = "At NEMA tlLogic '" + getID() + "', the cross phase switching for phase " + toString(phaseDetectInfo.first) 
                                    + " is not enabled because phase " + toString(phaseDetectInfo.second.cpdSource) + " does not exist"; 
                if (!ignoreErrors){
                    throw ProcessError(msg);
                } else {
                    WRITE_WARNING(msg)
                }
            }
        }
    }
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
    //map NEMA phase to detectors
    // std::cout << "start of NEMA phase to detector map building " << std::endl;
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
        phase2DetectorMap.find(NEMAPhaseIndex) -> second.detectors = detectors;
    }
#ifdef DEBUG_NEMA
    // print to check phase2DetectorMap
    std::cout << "Print to check phase2DetectorMap" << std::endl;
    for (auto item : phase2DetectorMap) {
        std::cout << "The NEMA phase index = " << item.first << " has detectors: \n";
        for (auto det : item.second.detectors) {
            std::cout << '\t' << det->getID() << std::endl;
        }
    }
#endif

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
    for (int ringIndex = 0; ringIndex <= 1; ringIndex++){
        // TS2 Force Offs don't go in order, so using a different method to check cycle time
        double cycleLengthCalculated = 0;
        for (int p : rings[ringIndex]){
            if (p > 0){
                cycleLengthCalculated += (maxGreen[p - 1] + yellowTime[p - 1] + redTime[p - 1]);
            }
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
    // check barriers
    double ring1barrier1_length = forceOffs[r1barrier - 1] + yellowTime[r1barrier - 1] + redTime[r1barrier - 1];
    double ring2barrier1_length = forceOffs[r2barrier - 1] + yellowTime[r2barrier - 1] + redTime[r2barrier - 1];
    if (ring1barrier1_length != ring2barrier1_length) {
        const std::string error = "At NEMA tlLogic '" + getID() + "', the phases before barrier 1 from both rings do not add up. (ring1="
                                  + toString(ring1barrier1_length) + ", ring2=" + toString(ring2barrier1_length) + ")";
        if (coordinateMode && !ignoreErrors) {
            throw  ProcessError(error);
        } else {
            WRITE_WARNING(error);
        }
    }
    double ring1barrier2_length = forceOffs[r2coordinatePhase - 1] + yellowTime[r2coordinatePhase - 1] + redTime[r2coordinatePhase - 1];
    double ring2barrier2_length = forceOffs[r1coordinatePhase - 1] + yellowTime[r1coordinatePhase - 1] + redTime[r1coordinatePhase - 1];
    if (ring1barrier2_length != ring2barrier2_length) {
        const std::string error = "At NEMA tlLogic '" + getID() + "', the phases before barrier 2 from both rings do not add up. (ring1="
                                  + toString(ring1barrier2_length) + ", ring2=" + toString(ring2barrier2_length) + ")";
        if (coordinateMode && !ignoreErrors) {
            throw  ProcessError(error);
        } else {
            WRITE_WARNING(error);
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
    for (int i = 0; i < 8; i++) {
        nextMaxGreen[i] = newSplits[i] - yellowTime[i] - redTime[i];
    }
}


void
NEMALogic::setNewMaxGreens(std::vector<double> newMaxGreens) {
    for (int i = 0; i < 8; i++) {
        nextMaxGreen[i] = newMaxGreens[i];
    }
}


void
NEMALogic::setNewCycleLength(double newCycleLength) {
    myNextCycleLength = newCycleLength;
}


void
NEMALogic::setNewOffset(double newOffset) {
    myNextOffset = newOffset;
}

//helper methods

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
#ifdef DEBUG_NEMA
    //print to check
    for (auto i : output) {
        std::cout << i << std::endl;
    }
#endif
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

int NEMALogic::measureRingDistance(int currentPhase, int nextPhase, int ringNum){
        int length = (int)rings[ringNum].size();
        int d = 0;
        bool found = false;        
        for (int i = 0; i < (length * 2); i++){
            if (rings[ringNum][i % length] != 0){ 
                if (found){
                   d++;
                   if (rings[ringNum][i % length] == nextPhase){
                       break;
                   } 
                }
                else if (rings[ringNum][i % length] == currentPhase){
                   found = true;
                } 
            }
        }
        assert(d > 0);
        return d;
}


//b should be the base of mode
double NEMALogic::ModeCycle(double a, double b) {
    double c = a - b;
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
            std::string out_str=std::to_string(isDetectorActivated(1, phase2DetectorMap.find(1) -> second));
            for (int i = 2; i<=8; i++)
            {
                out_str+=",";
                if (phase2DetectorMap.find(i) != phase2DetectorMap.end()) {
                    out_str+=std::to_string(isDetectorActivated(i, phase2DetectorMap.find(i) -> second));
                } else {
                    out_str+=std::to_string(false);
                }
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
NEMALogic::calculateForceOffs170(int r1StartIndex, int r2StartIndex){
    int initialIndexRing[2] = {r1StartIndex, r2StartIndex};
    // calculate force offs with the rings in order
    for (int ringNumber = 0; ringNumber<2;ringNumber++){
        int length = (int)rings[ringNumber].size();
        int aPhaseNumber = rings[ringNumber][initialIndexRing[ringNumber]];
        int aPhaseIndex = aPhaseNumber - 1;
        int nPhaseIndex = aPhaseIndex; //next phase
        int nPhaseNumber = aPhaseNumber;
        forceOffs[aPhaseNumber-1]=maxGreen[aPhaseNumber-1];
        #ifdef DEBUG_NEMA
        std::cout << "Phase  "<<aPhaseNumber <<": force off "<<forceOffs[aPhaseNumber-1]<<std::endl;
        #endif
        for (int i = initialIndexRing[ringNumber]+1; i < length; i++) {
            nPhaseNumber = rings[ringNumber][i];
            nPhaseIndex = nPhaseNumber -1;
            // std::cout <<" ring "<<ringNumber <<" i: "<<i<< " phase: "<<nPhaseNumber<< std::endl;
            if (nPhaseNumber != 0){
                forceOffs[nPhaseIndex] = forceOffs[aPhaseIndex] + maxGreen[nPhaseIndex] + yellowTime[aPhaseIndex]+redTime[aPhaseIndex];
                aPhaseNumber = nPhaseNumber;
                aPhaseIndex = nPhaseIndex;

                #ifdef DEBUG_NEMA
                std::cout << "- Phase "<<aPhaseNumber <<": force off "<<forceOffs[aPhaseIndex]<<std::endl;
                #endif
            }
        }
    }
}

void
NEMALogic::calculateForceOffsTS2(){
    // TS2 "0" cycle time is the start of the "first" coordinated phases.
    // We can find this "0" point by first constructing the forceOffs in sequential order via the 170 method 
    calculateForceOffs170(0, 0);

    // Switch the Force Off Times to align with TS2 Cycle. 
    double minCoordTime = MIN2(forceOffs[r1coordinatePhase - 1] - maxGreen[r1coordinatePhase - 1], forceOffs[r2coordinatePhase - 1] - maxGreen[r2coordinatePhase - 1]);

    // loop rings individually
    for (int i = 0; i < 2; i++){
        for (int p : rings[i]){
            if (p > 0){
                if ((forceOffs[p - 1] - minCoordTime) >= 0){
                    forceOffs[p - 1] -= minCoordTime;
                }else {
                    forceOffs[p - 1] = (myCycleLength + (forceOffs[p - 1] - minCoordTime));
                }
            }
        }
    } 
}

void
NEMALogic::calculateInitialPhases170(){
    int initialIndexRing [2] = {0, 0};
    // calculate initial phases based on in cycle clock
    for (int ringNumber = 0; ringNumber<2;ringNumber++){
        int length = (int)rings[ringNumber].size();
        for (int i = initialIndexRing[ringNumber]; i < length; i++) {
            int aPhaseIndex = rings[ringNumber][i]-1;
            if (aPhaseIndex != -1){
                if (forceOffs[aPhaseIndex] - minGreen[aPhaseIndex] > 0){
                    phaseCutOffs[aPhaseIndex] = forceOffs[aPhaseIndex] - minGreen[aPhaseIndex];
                } else {
                    phaseCutOffs[aPhaseIndex] = myCycleLength - forceOffs[aPhaseIndex] - minGreen[aPhaseIndex];
                }
                #ifdef DEBUG_NEMA
                std::cout << "Phase "<<aPhaseIndex+1<<" cut off is "<<phaseCutOffs[aPhaseIndex]<<std::endl;
                #endif
            }
        }
    }

    // sort phaseCutOffs in order, this is to adapt it to the TS2 algorithm. 
    // Type 170 should already be sorted.
    // Slice Phase Cutoffs into Ring1 & Ring 2
    std::vector<IntVector> localRings = rings;
    for (int ringNumber = 0; ringNumber < 2; ringNumber++){
        std::sort(localRings[ringNumber].begin(), localRings[ringNumber].end(), [&](int i, int j) 
        { return phaseCutOffs[i - 1] < phaseCutOffs[j - 1]; });
    }

    // find the current in cycle time.
    SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    double currentTimeInSecond = STEPS2TIME(now);
    double currentInCycleTime = ModeCycle(currentTimeInSecond - cycleRefPoint - offset, myCycleLength);

    // find the initial phases
    bool found[2] = {false, false};
    for (int ringNumber = 0; ringNumber < 2; ringNumber++){
        int aPhaseIndex = -1;
        // This searches sorted
        for (int p: localRings[ringNumber]) {
            if (p > 0){
                aPhaseIndex = p - 1;
                // #TODO: Fix this logic intelligently.
                if ((myCabinetType == Type170 && (currentInCycleTime + minGreen[p - 1] < phaseCutOffs[p - 1]))
                    || (myCabinetType == TS2 && fitInCycle(p, ringNumber)))
                    {   
                    #ifdef DEBUG_NEMA
                    std::cout<<"current in cycle time="<<currentInCycleTime<<" phase: "<<aPhaseIndex<<std::endl;
                    #endif
                    found[ringNumber] = true;
                    break;
                }
            }
        }
        if (ringNumber == 0){
            if (found[ringNumber]){
                activeRing1Index = aPhaseIndex;
                activeRing1Phase = activeRing1Index + 1;
            }
        }
        else{
            if (found[ringNumber]){
                activeRing2Index = aPhaseIndex;
                activeRing2Phase = activeRing2Index + 1;
            }
        }
    }
    if (found[0] * found[1] < 1){
        // If one or the other phases weren't found, default to the coordinated phase.
        // This ensures that no barriers are crossed
        activeRing2Phase = r2coordinatePhase;
        activeRing2Index = r2coordinatePhase - 1;
        activeRing1Phase = r1coordinatePhase;
        activeRing1Index = r1coordinatePhase - 1;
    }
}

void
NEMALogic::calculateInitialPhasesTS2(){
    // Modifications where made to 170 algorithm so that it works with both.
    calculateInitialPhases170();

    // Set the phase expected duration to initialize correctly
    phaseExpectedDuration[activeRing1Phase - 1] = activeRing1Phase == r1coordinatePhase? coordModeCycleTS2(0, activeRing1Phase) : minGreen[activeRing1Phase - 1];
    phaseExpectedDuration[activeRing2Phase - 1] = activeRing2Phase == r2coordinatePhase? coordModeCycleTS2(0, activeRing2Phase) : minGreen[activeRing2Phase - 1];
}

double
NEMALogic::coordModeCycle170(NEMAPhase* phase){
    return ModeCycle(myCycleLength - (simTime - cycleRefPoint - offset) - phase->yellow - phase->red, myCycleLength);  
}

double
NEMALogic::coordModeCycleTS2(NEMAPhase* phase){
    // This puts the phase green for the rest of the cycle, plus the first bit in which it must be green
    // We don't need the yellow and red here because the force off already incorporates that.
    return ModeCycle((myCycleLength + phase->yellow) - (simTime - cycleRefPoint - offset), myCycleLength);
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

// ===========================================================================
// Phase Definitions
// =========================================================================== 

NEMAPhase::NEMAPhase(
                     int phaseName,
                     bool isBarrier,
                     bool isGreenRest,
                     bool isCoordinated,
                     bool minRecall,
                     bool maxRecall,
                     bool fixForceOff,
                     int barrierNum,
                     int ringNum,
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
    
    // Timing Parameters
    maxGreenDynamic = maxDuration;
    myStartTime = 0.;
    myExpectedDuration = minDuration;
    maxStartTime = 0.;
    myLastEnd = 0.;
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
    }

    // create the phase detector info
    myDetectorInfo = phaseDetectorInfo(latching, 
        crossPhaseTarget > 0 ? controller->getPhaseObj(crossPhaseTarget) : nullptr, 
        crossPhaseSource > 0 ? controller->getPhaseObj(crossPhaseSource) : nullptr
    );
}

void
NEMAPhase::recalculateTiming(void){
    yellow = STEPS2TIME(myCorePhase->yellow);
    red = STEPS2TIME(myCorePhase->red);
    minDuration = STEPS2TIME(myCorePhase->minDuration);
    maxDuration = STEPS2TIME(myCorePhase->maxDuration);
    vehExt = STEPS2TIME(myCorePhase->vehext);
}


std::string& 
NEMAPhase::getNEMAState() {
    std::string newState = "";
    std::string curState = myCorePhase->getState();
    if (myLightState >= LightState::Green) {
        newState = curState;

    } else if (myLightState >= LightState::Red) {
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
NEMAPhase::checkMyDetectors(NEMALogic* controller){
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
    
    // clear the last transition decision
    lastTransitionDecision = nullptr;

    // Calculate the Max Green Time & Expected Duration here:
    if (controller->coordinateMode){
        if (coordinatePhase){
            myExpectedDuration = controller->coordModeCycle(this);
        } else {
            // In case the phase is being
            if (fixForceOff){
                maxGreenDynamic = controller->ModeCycle(forceOffTime - controller->getCurrentOffsetTime(), controller->getCurrentCycleLength());
            } else {
                maxGreenDynamic = MIN2(maxDuration, controller->ModeCycle(forceOffTime - controller->getCurrentOffsetTime(), controller->getCurrentCycleLength()));
            }
            myExpectedDuration = minDuration;
        }
    } 
    else {
        myExpectedDuration = minDuration;
    }

    // Implements the maxRecall functionality
    if (maxRecall){
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
        if (nextPhases[!ringNum]->getToPhase() == otherPhase && otherPhase->readyToSwitch){
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
    
    // return trySwitch(controller, nextPhase);
}

const double
NEMAPhase::getTransitionTime(NEMALogic* controller){
    if (!transitionActive){
        return (yellow + red);
    } else {
        return MAX2(0.0, ((controller->getCurrentTime() - myLastEnd) - (yellow + red)));  
    }
}


void
NEMAPhase::update(NEMALogic* controller){
    // Continuation Logic
    double duration = controller->getCurrentTime() - myStartTime;

    // Check the vehicle extension timer
    myExpectedDuration += calcVehicleExtension(duration);
    
    // Special Logic for Green Rest, which behaves uniquely
    if (myLightState == LightState::GreenRest){
        // check all other detectors and see if anything else is active. If so, 
        // start the green rest timer countdown, which is == to the max duration of the phase
        bool vehicleActive = false;
        for (auto &p : controller->getPhaseObjs()){
            if ((p.phaseName != phaseName) 
                && (p.phaseName != controller->getActivePhase(!ringNum)->phaseName) 
                && p.myDetectorInfo.detectActive){
                greenRestTimer -= TS;
                vehicleActive = true;
                break;
            }
        }
        if (!vehicleActive){
            greenRestTimer = maxDuration;
            myStartTime = controller->getCurrentTime() - minDuration;
            myExpectedDuration = minDuration;
        }

        if (greenRestTimer < TS){
            readyToSwitch = true;
            // force the counterparty to be ready to switch too
            controller->getActivePhase(!ringNum)->readyToSwitch = true;
        }

        // Special Behavior when the Green Rest Circles all the way around in coordinated mode
        if (coordinatePhase){
            // This means that we have green rested until I should "start" again. Just call my entry function again.
            if (controller->getTimeInCycle() <= (forceOffTime - maxDuration + TS / 2)){
                enter(controller, this);
            }
        }

    }

    // Check to see if a switch is desired
    if (duration >= myExpectedDuration){
        // nextPhases = trySwitch(controller, nextPhase);
        readyToSwitch = true;
    } 
}


double
NEMAPhase::calcVehicleExtension(double duration){
    double extTime = 0.0;
    if (!coordinatePhase && myExpectedDuration < maxGreenDynamic){
        if (myDetectorInfo.detectActive){
            extTime = MIN2(vehExt, maxGreenDynamic - duration);
        }
    }

    return extTime;
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
}

std::vector<PhaseTransitionLogic*>
NEMAPhase::trySwitch(NEMALogic* controller){
    std::vector<PhaseTransitionLogic *> nextPhases;

    // Give preference to the last transition decision
    // This is the decision that was made that forced the phase from G -> Y
    if (lastTransitionDecision != nullptr){
        nextPhases.push_back(lastTransitionDecision);
    }

    for (auto &t: myTransitions){
        if (t->okay(controller)){
            nextPhases.push_back(t);
        }
    }
    return nextPhases;
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
    // Order matters here. The complexity decreases you go down
    if (fromPhase->coordinatePhase){
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
    // Simplest transition logic. Just check if a detector (or recall) is active on that phase and return it
    return toPhase->callActive();
}

bool
PhaseTransitionLogic::coordBase(NEMALogic* controller){
    // first check if the free logic is upheld
    if (freeBase(controller)){
        // Then check if the "to phase" can fit, which means that there is enough time to fit the current transition + the minimum time of the next phase 
        double transitionTime = fromPhase->getTransitionTime(controller);
        double timeTillForceOff = controller->ModeCycle(fromPhase->forceOffTime - controller->getTimeInCycle(), controller->getCurrentCycleLength());
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
            // now determine if there my prior phase can fit or not. We already know that I can fit.
            NEMAPhase* priorPhase = toPhase->getSequentialPriorPhase();
            double timeTillForceOff = controller->ModeCycle(priorPhase->forceOffTime - controller->getTimeInCycle(), controller->getCurrentCycleLength());
            double transitionTime = fromPhase->getTransitionTime(controller);
            
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
NEMALogic::getPhaseObj(int phaseNum){
    for (auto &p : myPhaseObjs){
        if (p->phaseName == phaseNum){
            return p;
        }
    }
    assert(0);
}


void
NEMALogic::getNextPhases(std::vector<float_t> &distances, std::vector<PhaseTransitionLogic*[2] > &transitions){
        std::vector<std::vector<PhaseTransitionLogic* >> potentialPhases;
        // Get an Array of Potential Transitions
        for (const auto &p : myActivePhaseObjs){
            potentialPhases.push_back(p->trySwitch(this));
        }

        // Loop through all combination of transitions, keeping only the valid ones.
        for (const auto &r1_t : potentialPhases[0]){
            for (const auto &r2_t : potentialPhases[1]){
                if (r1_t->getToPhase()->ringNum == r2_t->getToPhase()->ringNum){
                    transitions.push_back({r1_t, r2_t});
                    distances.push_back((r1_t->distance + r2_t->distance) / 2);
                } else {
                    // If the rings are different, add a choice where one of them is the default choice for whatever ring it is
                    // create two choices, one for each of the phase as they are on  different rings
                    if (r1_t->getFromPhase()->readyToSwitch){
                        // R2 default 
                        int defaultR2P = myActivePhaseObjs[1]->readyToSwitch? 
                            defaultBarrierPhases[1][r1_t->getToPhase()->ringNum]->phaseName : myActivePhaseObjs[1]->phaseName;
                        PhaseTransitionLogic* defaultR2Transition = myActivePhaseObjs[1]->getTransition(defaultR2P);
                        // only add it if it does not cross a barrier!
                        if (defaultR2Transition->getToPhase()->barrierNum == r1_t->getToPhase()->barrierNum){
                            transitions.push_back({r1_t, defaultR2Transition});
                            distances.push_back((defaultR2Transition->distance + r1_t->distance) / 2);
                        }
                    }

                    if (r2_t->getFromPhase()->readyToSwitch){
                        // R1 default 
                        int defaultR1P = myActivePhaseObjs[0]->readyToSwitch? 
                            defaultBarrierPhases[0][r2_t->getToPhase()->ringNum]->phaseName : myActivePhaseObjs[0]->phaseName;
                        PhaseTransitionLogic* defaultR1Transition = myActivePhaseObjs[0]->getTransition(defaultR1P);
                        // only add it if it does not cross a barrier!
                        if (defaultR1Transition->getToPhase()->barrierNum == r2_t->getToPhase()->barrierNum){
                            transitions.push_back({defaultR1Transition, r2_t});
                            distances.push_back((defaultR1Transition->distance + r2_t->distance) / 2);
                        }
                }
                // If the distances are <= 1, this means that this is the shortest transition possible 
                // and we should break without considering the other options.  
                if (distances.back() < 1){
                    return;
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
    
    // Check the Detectors
    for (auto &p : myPhaseObjs){
        p.checkMyDetectors(this);
    }

    // Update the timing parameters
    for (const auto& p : myActivePhaseObjs){
        p->update(this);
    }

    // Calculate the Next Phases, but only if one or the other is ready to transition
    if (myActivePhaseObjs[0]->readyToSwitch || myActivePhaseObjs[1]->readyToSwitch){
        // Then sort the list 
        std::vector<float_t> distances;
        std::vector<PhaseTransitionLogic*[2] > transitions;
        getNextPhases(distances, transitions);
        // Sort the next phases and select the closest.
        if (transitions.size() > 0){
            std::sort(transitions.begin(), transitions.end(), [&](int i, int j) { return distances[i - 1] < distances[j - 1]; });
        }

        // Try the exit logic. This doesn't necessarily mean that the phase will exit, 
        // as it could go into green rest or green transfer, but this is considered an "exit"
        for (const auto& p : myActivePhaseObjs){
            if (p->readyToSwitch){
                p->exit(this, nextPhases);
            }
        }

        // This is the only time when something might have happened, so we update the phase strings here
        myPhase.setState(composeLightString());
        myPhase.setName(toString(myActivePhaseObjs[0]->phaseName) + "+" + toString(myActivePhaseObjs[1]->phaseName));
        myStep = 1 - myStep;
    }

    // Basic Assertion to ensure that the Barrier is not crossed
    assert(myActivePhaseObjs[0]->barrierNum == myActivePhaseObjs[1]->barrierNum);
    return TS;
}
