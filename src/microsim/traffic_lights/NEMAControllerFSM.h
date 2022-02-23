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
class NEMAPhase;

// ===========================================================================
// Enumeration
// ===========================================================================
enum class LightState {
    Red,
    Yellow,
    Green,
    GreenXfer,
    GreenRest
};

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

    /// @brief retrieve all detectors used by this program
    std::map<std::string, double> getDetectorStates() const override;

    // control logic
    std::string NEMA_control();

    std::string combineStates(std::string state1, std::string state2);

    int nextPhase(std::vector<int> ring, int phaseNum, int& distance,  bool sameAllowed, int ringNum);

    void getNextPhases(std::vector<float_t> &distances, std::vector<PhaseTransitionLogic*[2] > &transitions);

    double ModeCycle(double a, double b);

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

    /// @brief Wrapper Function to Simplify Accessing Time
    inline SUMOTime getCurrentTime(void) const {return simTime; };
    
    /// @brief Wrapper Function to Simplify Accessing Offset Cycle Time
    inline SUMOTime getCurrentOffsetTime(void) const {return simTime + myOffset; };


    // General Force Offs Function
    const double coordModeCycle(NEMAPhase* phase)  {
        switch (myCabinetType){
            case Type170:
                return coordModeCycle170(phase);
            case TS2:
                return coordModeCycleTS2(phase);
            default:
                // Default to Type 170
                return coordModeCycle170(phase);
        }
    };

    /// Coordinated Mode
    bool coordinateMode;

    /// @brief set the active phase
    inline void setActivePhase(NEMAPhase* phase){ myActivePhaseObjs[phase->ringNum] = phase; };
    
    /// @brief get the active phases
    inline NEMAPhase* getActivePhase(int ringNum){ return myActivePhaseObjs[ringNum]; };

    /// @brief return all phases for a given ring
    std::vector<NEMAPhase*> getPhasesByRing(int ringNum); 

    /// @brief return all phases for a given ring
    NEMAPhase* getPhaseObj(int phaseNum); 

    /// @brief return all Phase objects 
    inline std::vector<NEMAPhase*> getPhaseObjs(void) {return myPhaseObjs;}; 
protected:
    /// offset
    double offset;
    double myNextOffset;
    
    /// file paths
    std::string outputStateFilePath;
    std::ofstream outputStateFile;

    /// @brief called at every trySwitch to update the traffic lights
    void update(void);

    /// variable to save time
    double simTime = 0;
    inline void setCurrentTime(void) {simTime = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep()); }

    /// @brief variable to store the active phases
    NEMAPhase* myActivePhaseObjs[2] = {nullptr, nullptr};
    // This is where the phases ultim   ately live
    std::vector<NEMAPhase* > myPhaseObjs;
    // Store the default phases for each of the barrier. This is what the controller will transition to if
    // call on just 8, in 2, 6 -> [4, 8] not [3, 8]
    // These are the dual entry phases
    NEMAPhase* defaultBarrierPhases[2][2]; 

    /// @brief Measure Distance Between Two Point on the same ring
    int measureRingDistance(int p1, int p2, int ringNum);

    /// @brief Initializes timing parameters and calculate initial phases
    void constructTimingAndPhaseDefs(std::string &barriers, std::string &coordinates, std::string &ring1, std::string &ring2);

    /// @brief Construct the Light String Every Time that there is a change
    std::string composeLightString();

    /// @brief Helper function to construct the phaseDetector objects
    void createDetectorObjects();

    /// @brief helper function for finding if vector contains a phase
    const bool vectorContainsPhase(std::vector<int> v, int phaseNum);

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

    /// @brief return whether there is a major link from the given lane in the given phase
    bool hasMajor(const std::string& state, const LaneVector& lanes) const;
    /// @}
    std::vector<int> readParaFromString(std::string s);

    //convert laneIDs string to string vector
    std::vector<std::string> string2vector(std::string s);

    //decide whether the detector is for left turn lane
    //if it is, use the detector length for left turn lane
    bool isLeftTurnLane(const MSLane* const lane) const;

    //convert "1" to int 1
    int string2int(std::string s);

    // protected:
    detectorMap myDetectorForPhase;

    std::vector<DetectorInfo> myDetectorInfoVector;

    /// @brief A map from lanes to detectors
    LaneDetectorMap myLaneDetectorMap;

    /// @brief A map from lanes names to phases
    std::map<std::string, int> myLanePhaseMap;

    /// @brief A map from detectors to lanes
    DetectorLaneMap myDetectorLaneMap;

    // detector length
    double myDetectorLength;

    // detector length for left turn lane
    double myDetectorLengthLeftTurnLane;

    // total cycle length
    double myCycleLength;

    // total cycle length in the next cycle
    double myNextCycleLength;

    /// Whether the detectors shall be shown in the GUI
    bool myShowDetectors;

    /// The output file for generated detectors
    std::string myFile;

    /// The frequency for aggregating detector output
    SUMOTime myFreq;

    /// Whether detector output separates by vType
    std::string myVehicleTypes;

    /*
    {
        {3,4,1,2},
        {7,8,5,6}
    }
    */
    std::vector<std::vector<int>> rings;

    /*
    {
        {1 : phaseDetectorInfo{
                detectors: {det1, det2, ...},
                crossPhaseDetector: 6
            },
        },
        {2 : ...
    }
    */
    // std::map<int, phaseDetectorInfo> phase2DetectorMap;
    std::map<int, std::vector<std::string>> phase2ControllerLanesMap;

    bool fixForceOff;
    double cycleRefPoint;// missing update
    bool whetherOutputState;
    bool ignoreErrors;
    
    // Cabinet Type
    // #TODO write a parser to convert parameter to type 
    enum cabinetType {
        Type170,
        TS2
    };

    // Store the cabinet type
    cabinetType myCabinetType;

    cabinetType parseCabinetType(std::string inputType){
        std::string cleanString;
        for (const char& c : inputType){
            if (isalpha(c) || isdigit(c)){
                cleanString += (char)::tolower(c);
            }
        }
        if (cleanString == "type170"){
            return Type170;
        } else if (cleanString == "ts2"){
            return TS2;
        } else {
            throw InvalidArgument("Please set cabinetType for NEMA tlLogic to either Type170 or TS2");
        }
    };

    /// @brief virtual phase that holds the current state
    MSPhaseDefinition myPhase;

    // Green Transfer Option
    bool greenTransfer;

    // handle error
    void error_handle_not_set(std::string param_variable, std::string param_name);
    void validate_timing();

    /// @brief implement any changes that may have come via traci
    void implementTraciChanges(void);

    // read 1 detector state
    bool readDetector(int phase);

    // TS2 Specific Timing
    void calculateForceOffsTS2();
    // Type170 Specific Timing
    void calculateForceOffs170();
    // General Force Offs Function
    void calculateForceOffs(){
        switch (myCabinetType){
            case Type170:
                return calculateForceOffs170();
            case TS2:
                return calculateForceOffsTS2();
            default:
                return calculateForceOffs170();
        }
    }


    // TS2 Specific Initial Phases
    void calculateInitialPhasesTS2();
    // Type170 Specific Initial Phases
    void calculateInitialPhases170();
    // General Force Offs Function
    void calculateInitialPhases(){
        switch (myCabinetType){
            case Type170:
                return calculateInitialPhases170();
            case TS2:
                return calculateInitialPhasesTS2();
            default:
                // Default to Type170
                return calculateInitialPhases170();
        }
    }

    // TS2 Specific Coordinated Mode Cycle
    double coordModeCycleTS2(NEMAPhase* phase);
    // Type170 Specific Coordinated Mode Cycle
    double coordModeCycle170(NEMAPhase* phase);

    // TS2 Specific fit in cycle algorithm
    bool fitInCycleTS2(int phase,  int ringNum);
    // Type170 fitInCycle algorithm
    // bool fitInCycle170(int _phase, int _ringNum){
    //     return true;
    // }
    // 
    double fitInCycle(int phase, int ringNum){
        switch (myCabinetType){
            case Type170:
                return true;
            case TS2:
                return fitInCycleTS2(phase, ringNum);
            default:
                // Default to Type 170
                return true;
        }
    }
};


// I wanted it to inherit the phase but this complicated things
class NEMAPhase {
    public:
        // 
        typedef struct phaseDetectorInfo {
            phaseDetectorInfo():
                detectors(),
                cpdTarget(),
                cpdSource(),
                detectActive(),
                latching()
            {}
            phaseDetectorInfo(bool latching, NEMAPhase* cpdTarget, NEMAPhase* cpdSource):
                latching(latching),
                cpdSource(cpdSource),
                cpdTarget(cpdTarget)
            {}
            std::vector<MSE2Collector*> detectors = {nullptr};
            NEMAPhase* cpdTarget = nullptr;
            NEMAPhase* cpdSource = nullptr;
            bool detectActive = false;
            bool latching = false;
        };

        // #TODO: Update Documentation
        /** @brief Constructor
         * @param[in] phaseName Name of the phase as an integer. Special as it will be used in calculations 
         * @param[in] programID This tls' sub-id (program id)
         * @param[in] phases Definitions of the phases
         * @param[in] step The initial phase index
         * @param[in] delay The time to wait before the first switch
         * @param[in] parameter The parameter to use for tls set-up
         */
        NEMAPhase(int phaseName,
                  bool isBarrier,
                  bool isGreenRest,
                  bool isCoordinated,
                  bool minRecall,
                  bool maxRecall,
                  bool fixForceOff,
                  int barrierNum,
                  int ringNum,
                  MSPhaseDefinition* phase);

        /// @brief Destructor
        ~NEMAPhase();

        /// @brief to make the deconstructor polymorphic
        virtual ~NEMAPhase(){};
        
        /// @brief return reference to instance
        NEMAPhase* getInstance();

        // return the current state
        inline LightState getCurrentState() const { return myLightState; }
        inline std::vector<MSE2Collector*> getDetectors() const { return myDetectorInfo.detectors; } 

        
        /// @brief set the detector vector
        inline void setDetectors(std::vector<MSE2Collector*> detectors) {myDetectorInfo.detectors = detectors;};

        // Build a Map of Valid Transitions and store the detector-based information
        void init(NEMALogic* controller,  int crossPhaseTarget, int crossPhaseSource, bool latching);
        
        // update the phase. This checks detectors etc. 
        void update(NEMALogic* controller);

        /// @brief phase exit logic
        void exit(NEMALogic* controller, PhaseTransitionLogic* nextPhases[2]);
        
        /// @brief simple method to check if there is a recall on the phase.
        inline const bool hasRecall(void) { return minRecall || maxRecall; }; 
        
        /// @brief simple method to check if there is either a recall or an active detector
        inline const bool callActive(void) { return minRecall || maxRecall || myDetectorInfo.detectActive; };

        /// @brief simple method to check if a detector is active
        inline const bool detectActive(void) { return myDetectorInfo.detectActive; };


        // Check Detectors. Called on all phases at every step
        void checkMyDetectors(NEMALogic* controller);

        // Need-to-know Phase Settings
        int phaseName;
        bool isAtBarrier;
        bool isGreenRest;
        int barrierNum;
        int ringNum;
        bool coordinatePhase; 
        bool fixForceOff;
        bool minRecall;
        bool maxRecall;

        /// Need to Know Phase Settings
        double greenRestTimer;
        double greatestStartTime;

        /// @brief flag to for the supervisory controller to denote whether phase is ready to switch or not.
        bool readyToSwitch;

        /// @brief stores the force off time in coordinated mode
        double forceOffTime;

        /// @brief accessory method to get the transition time
        const double getTransitionTime(NEMALogic* controller);

        /// @brief get the prior phase
        inline NEMAPhase* getSequentialPriorPhase() { return sequentialPriorPhase; };
        
        /// @brief set the prior phase
        inline NEMAPhase* setSequentialPriorPhase(NEMAPhase* priorPhase) { sequentialPriorPhase = priorPhase; };
        
        /// @brief try to calculate the next phases        
        std::vector<PhaseTransitionLogic*> trySwitch(NEMALogic* controller);
        
        /// @brief find a transition given the to phase
        PhaseTransitionLogic* getTransition(int toPhase);

        /// Return the ryg light string for the phase
        std::string& getNEMAState(void);

        /// @brief accessory function to recalculate timing
        void recalculateTiming(void);

        /// @brief Force Enter. This Should only be called at initialization time
        inline void forceEnter(NEMALogic *controller) { enter(controller, sequentialPriorPhase); };
        
        /// core timing.
        double yellow;
        double red;
        double minDuration;
        double maxDuration;
        double nextMaxDuration; 
        double vehExt;

    private:
        // save the core phase
        MSPhaseDefinition* myCorePhase = nullptr;

        // Save my Pointer
        NEMAPhase* myInstance = nullptr;
        NEMAPhase* myLastPhaseInstance = nullptr;
        NEMAPhase* sequentialPriorPhase = nullptr;

        // Phase Knowledge Space
        LightState myLightState;
        phaseDetectorInfo myDetectorInfo;

        // Timing Parameters
        // -----------------
        // Dynamic max green. Typically MSPhaseDefinition max 
        double maxGreenDynamic;
        double myStartTime;
        double myExpectedDuration;
        double myLastEnd;

        // Coordination Parameters
        double maxStartTime;

        // Calculate the vehicle extension
        double calcVehicleExtension(double currentTime);

        // Potential Transition Map
        std::vector<PhaseTransitionLogic*> myTransitions;

        // entry parameters for the state
        void enter(NEMALogic* controller, NEMAPhase* lastPhase);        
        // exit state for the phase
        std::vector<NEMAPhase *> exit(NEMALogic* controller, NEMAPhase* nextPhase);

        // variable to store the 
        bool transitionActive;

        /// @brief pointer to save the last transition
        PhaseTransitionLogic* lastTransitionDecision;
        
};


class PhaseTransitionLogic {
    public:
        PhaseTransitionLogic(
            NEMAPhase* fromPhase,
            NEMAPhase* toPhase
        );
        
        // Check to see if transition is okay
        bool okay(NEMALogic* controller);

        /// @brief build the transition logic based on the from and to phase
        std::function<bool(NEMALogic*)> buildLogic(NEMALogic* controller);

        const int distance;

        /// @brief deconstructor
        ~PhaseTransitionLogic();

        /// @brief to make the deconstructor polymorphic
        virtual ~PhaseTransitionLogic(){};

        inline NEMAPhase* getToPhase(void) const { return toPhase; };
        inline NEMAPhase* getFromPhase(void) const { return fromPhase; };

    private:
        NEMAPhase* fromPhase;
        NEMAPhase* toPhase;
        
        /// @brief build the transition logic based on the from and to phase
        void buildLogic(void);

        bool fromBarrier(NEMALogic* controller);

        bool fromCoord(NEMALogic* controller);

        bool freeBase(NEMALogic* controller);

        bool coordBase(NEMALogic* controller);


};

