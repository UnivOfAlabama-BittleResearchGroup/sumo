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
/// @file    NEMAController.h
/// @author  Tianxin Li
/// @author  Qichao Wang
/// @date    August 2020
///
// An actuated NEMA-phase-compliant traffic light logic
/****************************************************************************/
#pragma once
#include <config.h>

#include <utility>
#include <vector>
#include <bitset>
#include <map>
#include <microsim/MSEventControl.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include "MSSimpleTrafficLightLogic.h"
#include "microsim/output/MSE2Collector.h"
#include "MSPhaseDefinition.h"
#include <set>


// ===========================================================================
// class declarations
// ===========================================================================
class NLDetectorBuilder;
class MSE2Collector;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NEMALogic
 * @brief A NEMA (adaptive) traffic light logic based on E2Detector
 */
class NEMALogic : public MSSimpleTrafficLightLogic {
public:

    /// @brief Definition of a map from lanes to corresponding area detectors
    typedef std::map<MSLane*, MSE2Collector*> LaneDetectorMap;

    /// @brief Definition of a map from detectors to corresponding lanes
    typedef std::map<MSE2Collector*, MSLane*, ComparatorIdLess> DetectorLaneMap;

    /** @brief Constructor
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] programID This tls' sub-id (program id)
     * @param[in] phases Definitions of the phases
     * @param[in] step The initial phase index
     * @param[in] delay The time to wait before the first switch
     * @param[in] parameter The parameter to use for tls set-up
     */
    NEMALogic(MSTLLogicControl& tlcontrol,
              const std::string& id, const std::string& programID,
              const SUMOTime offset,
              const MSSimpleTrafficLightLogic::Phases& phases,
              int step, SUMOTime delay,
              const std::map<std::string, std::string>& parameter,
              const std::string& basePath);


    /** @brief Initialises the tls with information about incoming lanes
     * @param[in] nb The detector builder
     * @exception ProcessError If something fails on initialisation
     */
    void init(NLDetectorBuilder& nb) override;

    /// @brief Destructor
    ~NEMALogic();


    SUMOTime trySwitch() override;

    /// @name Dynamic Information Retrieval
    /// @{

    /** @brief Returns the definition of the current phase
     * @return The current phase
     */
    const MSPhaseDefinition& getCurrentPhaseDef() const override;
    /// @}

    /// @brief called when switching programs
    void activateProgram() override;
    void deactivateProgram() override;

    bool showDetectors() const {
        return myShowDetectors;
    }

    void setShowDetectors(bool show);

    std::map<int, std::vector<MSE2Collector*>> getPhase2DetectorMap() {
        std::map<int, std::vector<MSE2Collector*>> temp;
        for (auto const& detectInfo : phase2DetectorMap) {
            temp[detectInfo.first] = detectInfo.second.detectors;
        }
        return temp;
    }

    /// @brief retrieve all detectors used by this program
    std::map<std::string, double> getDetectorStates() const override;

    // control logic
    std::string NEMA_control();

    std::string combineStates(std::string state1, std::string state2);

    int nextPhase(std::vector<int> ring, int phaseNum, int& distance,  bool sameAllowed, int ringNum);

    std::tuple<int, int> getNextPhases(int currentR1Index, int currentR2Index, int& r1Distance, int& r2Distance, bool toUpdateR1, bool toUpdateR2, bool stayOk = false);

    double ModeCycle(double a, double b);

    std::string transitionState(std::string curState, int RYG);

    std::set<std::string> getLaneIDsFromNEMAState(std::string state);

    void setNewMaxGreens(std::vector<double> newMaxGreens);
    void setNewSplits(std::vector<double> newSplits);
    void setNewCycleLength(double newCycleLength);
    void setNewOffset(double newOffset);

    // not using for now, but could be helpful for cycle change controller
    double getCurrentCycleLength() {
        return myCycleLength;
    }

    void setCycleLength(double newCycleLength) {
        myCycleLength = newCycleLength;
    }

    bool isGreenPhase(std::string state) {
        for (char ch : state) {
            if (ch == 'g' || ch == 'G') {
                return true;
            }
        }
        return false;
    }

    /// @brief try to set the given parameter. Parameters prefixed with 'NEMA.' control functionality
    void setParameter(const std::string& key, const std::string& value) override;

    /// @brief try to get the value of the given parameter. Parameters prefixed with 'NEMA.' control functionality
    const std::string getParameter(const std::string& key, const std::string defaultValue = "") const override;

protected:
    /// @brief Initializes timing parameters and calculate initial phases
    void constructTimingAndPhaseDefs();

    // create a small datatype for mapping detector to phase index
    // This is the one copied from MSActuatedTrafficLightLogic
    // not used in our controller, but it is here for meeting the SUMO default traffic logic light check
    // this one and related could be removed with extra efforts
    struct DetectorInfo {
        DetectorInfo(MSE2Collector* _det, int numPhases):
            det(_det),
            servedPhase(numPhases, false)
        {}
        MSE2Collector* det;
        SUMOTime lastGreenTime = 0;
        std::vector<bool> servedPhase;
    };

    typedef std::vector<std::vector<DetectorInfo*>> detectorMap;

    /// @brief private convience method to get the sim time
    inline SUMOTime getCurrentSimTime() const {return MSNet::getInstance()->getCurrentTimeStep();}
};

enum class LightState {
	Red,
	Yellow,
	Green,
	GreenXfer,
    GreenRest
};


struct phaseDetectorInfo {
    phaseDetectorInfo():
        detectors(),
        cpdTarget(),
        cpdSource(),
        detectActive(),
        latching()
    {}
    phaseDetectorInfo(int _cross_phase_source, bool _latching):
        cpdSource(_cross_phase_source),
        latching(_latching)
    {}
    std::vector<MSE2Collector*> detectors = {nullptr};
    int cpdTarget = 0;
    int cpdSource = 0;
    bool detectActive = false;
    bool latching = false;
};


class Phase {
    public:
        // #TODO: Update Documentation
        /** @brief Constructor
         * @param[in] phaseName Name of the phase as an integer. Special as it will be used in calculations 
         * @param[in] programID This tls' sub-id (program id)
         * @param[in] phases Definitions of the phases
         * @param[in] step The initial phase index
         * @param[in] delay The time to wait before the first switch
         * @param[in] parameter The parameter to use for tls set-up
         */
        Phase(const int phaseName, 
              const MSPhaseDefinition& phaseDef,
              const bool isBarrier,
              const bool isGreenRest,
              const bool isCoordinated,
              const int barrierNum
              );

        /// @brief Destructor
        ~Phase();
        
        /// @brief return reference to instance
        static Phase* getInstance();

        // return the current state
        inline LightState getCurrentState() const { return myLightState; }

        // Build a Map of Valid Transitions
        void init(NEMALogic* controller);

        // Try to switch the phase. Should only be called on active phases. 
        // Should return a vector of the potential next phases
        std::vector<Phase*> trySwitch(NEMALogic* controller);
        
        // tick the phase. This is called on all phases at every step. 
        void tick(NEMALogic* controller);
        
        // construct timing stuff
        void enter(NEMALogic* controller);
        void exit(NEMALogic* controller);

        // Need-to-know Phase Settings
        bool isAtBarrier;
        bool isGreenRest;
        int barrierNum;
        bool coordinatePhase; 
        
    private:
        // Save my Pointer
        Phase* myInstance = nullptr;

        // Phase Knowledge Space
        LightState myLightState;
        NEMALogic* myController;
        phaseDetectorInfo myDetectorInfo;

        // Timing Parameters
        // -----------------
        // Dynamic max green. Typically MSPhaseDefinition max 
        double maxGreenDynamic;

        // Coordination Parameters
        double maxStartTime;
        double forceOffTime;

        // Potential Transition Map
        const std::vector<PhaseTransitionLogic> myTransitions;

        // Read Detectors
        void checkMyDetectors();
};


class PhaseTransitionLogic {
    public:
        PhaseTransitionLogic(
            const Phase* fromPhase,
            const Phase* toPhase
        );
        
        // Check to see if transition is okay
        int isTransitionOk();

        const int distance;

        /// @brief deconstructor
        ~PhaseTransitionLogic();

    private:
        Phase* fromPhase;
        Phase* toPhase;
        
        // function that determines if the transition is okay and returns the distance if so.
        // distance to a phase itself is always 4
        // If it can't transition, the distance is -1;
        bool transitionOk;
};



// class NEMASupervisor {
//     public:
//         /// @brief Constructor
//         NEMASupervisor(

//         );

//         /// @brief Destructor
//         ~NEMASupervisor();
        
//         /// @brief tick the controller, aka move it forward by one simulation step
//         void tick();

//     private:
//         // #TODO: is there ever a "Dual Ring" with != 2 rings? 
//         Phase* activePhases[2] = {nullptr, nullptr}; 
//         std::vector<std::vector<Phase>> myRings;
        
//         // Measure distance between the diagonal. 
//         // [1] 2  | [3]  4
//         // 5  [6] |  7  [8]
//         // 1, 6 -> 3, 8 has the same travelled top to bottom ring, but 3 is closer to the diagonal 
//         // [1] 2  | [3]  4
//         //    \------x---x
//         // 5  [6] |  7  [8] 
//         int measureDistance(Phase* phases[2]);

// }

