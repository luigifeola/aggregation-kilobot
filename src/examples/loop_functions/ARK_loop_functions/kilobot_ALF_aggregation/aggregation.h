/**
 * @file <aggregation.h>
 *
 * @author Luigi Feola <feola@diag.uniroma1.it>
 *
 * @brief This is the header file of the aggregation ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system.
 *
 */

#ifndef AGGREGATION_H
#define AGGREGATION_H

namespace argos
{
    class CSpace;
    class CFloorEntity;
    class CSimulator;
}

#include <math.h>
#include <iostream>
#include <time.h>
#include <string.h>
#include <string>
#include <cstring>
#include <cmath>
#include <numeric>
#include <array>
#include <random>
#include <algorithm>
#include <vector>
#include <bitset>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>

using namespace argos;

class aggregationCALF : public CALF
{

public:
    aggregationCALF();

    virtual ~aggregationCALF() {}

    virtual void Init(TConfigurationNode &t_tree);

    virtual void Reset();

    virtual void Destroy();

    virtual void PostStep();

    virtual void PostExperiment();

    /** Log Kilobot pose and state */
    void KiloLOG();

    /** Get a Vector of all the Kilobots in the space */
    void
    GetKilobotsEntities();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode &t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode &t_tree);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2 &vec_position_on_plane);

    /** 2D vector rotation */
    CVector2 VectorRotation2D(Real angle, CVector2 vec);

    /** Simulate proximity sensor*/
    std::vector<int> Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors);

private:
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/

    struct SVirtualWalls
    {
        CVector2 Center;
        Real Radius;       // radius for the circular arena
        Real Wall_width;   // wall width
        Real Wall_height;  // wall height
        Real Wall_numbers; // number of walls
    };

    SVirtualWalls m_ArenaStructure;


    typedef enum //kilobot walk
    {
        BROWNIAN = 0,
        PERSISTENT = 1,
    } SRobotWalk;

    struct FloorColorData //contains components of area color
    {
        UInt8 R;
        UInt8 G;
        UInt8 B;
    };

    UInt32 random_seed;
    
    /************************************/
    /*       Experiment variables       */
    /************************************/

    /* random number generator */
    CRandom::CRNG *c_rng;

    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /*Kilobots properties*/
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<argos::CColor> m_vecKilobotsColours;
    std::vector<CRadians> m_vecKilobotsOrientations;
    std::vector<SRobotWalk> m_vecKilobotWalks_ALF; //when adaptive, switch among persistent and brownian walk

    /* output LOG files */
    std::ofstream m_kiloOutput;

    /* output file name*/
    std::string m_strKiloOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;
};

#endif