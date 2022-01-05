#include "aggregation.h"

namespace
{

    const double kEpsilon = 0.0001;

    // environment setup
    // const double kScaling = 1.0 / 4.0;      //for no scaling set kScaling=0.5
    const double kKiloDiameter = 0.033;
    double vArena_size = 0.5;
    double vDistance_threshold = vArena_size / 2.0 - 0.04;

    // wall avoidance stuff
    const CVector2 left_direction(1.0, 0.0);
    const int kProximity_bits = 8;

    int internal_counter = 0;
}

aggregationCALF::aggregationCALF() : m_unDataAcquisitionFrequency(10)
{
    c_rng = CRandom::CreateRNG("argos");
}

void aggregationCALF::Reset()
{
    m_kiloOutput.close();
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

void aggregationCALF::Destroy()
{
    m_kiloOutput.close();
}

void aggregationCALF::Init(TConfigurationNode &t_node)
{
    /* Initialize ALF*/
    CALF::Init(t_node);

    /* Other initializations: Varibales, Log file opening... */
    /*********** LOG FILES *********/
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);

    /** 
     * 
     * WARNING:  set and read the right parameters you need from .argos 
     * 
     * */

    /* Read parameters from .argos*/
    // TConfigurationNode &tModeNode = GetNode(t_node, "extra_parameters");
    // GetNodeAttributeOrDefault(tModeNode, "adaptive", adaptive_walk, false);
    // GetNodeAttributeOrDefault(tModeNode, "adaptive_timeut", adaptive_timeout, false);

    random_seed = GetSimulator().GetRandomSeed();
}

void aggregationCALF::SetupInitialKilobotStates()
{
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsColours.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());

    m_vecKilobotWalks_ALF.resize(m_tKilobotEntities.size());

    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);


    /* Compute the number of kilobots on the field*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

}

void aggregationCALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotWalks_ALF[unKilobotID] = PERSISTENT;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_x, rand_y;

    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /* Get a random position and orientation for the kilobot initialized into a square but positioned in the circular arena */
    CQuaternion random_rotation;
    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
    Real radius = m_ArenaStructure.Radius - m_ArenaStructure.Wall_width / 2 - kKiloDiameter / 2 - kEpsilon;
    do
    {
        rand_x = c_rng->Uniform(CRange<Real>(-radius, radius));
        rand_y = c_rng->Uniform(CRange<Real>(-radius, radius));
        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), CVector3(rand_x, rand_y, 0), random_rotation, false);

        if (tries == maxTries - 1)
        {
            std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while (!distant_enough || (rand_x * rand_x) + (rand_y * rand_y) > radius * radius);

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = argos::CColor::BLACK;
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

void aggregationCALF::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    /** 
     * 
     * WARNING:  too many constant parameters 
     * 
     * */
    vDistance_threshold = vArena_size / 2.0 - 0.04;
    std::cout << "Arena size: " << vArena_size << "\n";

    TConfigurationNode &tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
    /* Get the node defining the walls parametres*/
    TConfigurationNode &t_VirtualWallsNode = GetNode(tVirtualEnvironmentsNode, "CircularWall");
    GetNodeAttribute(t_VirtualWallsNode, "radius", m_ArenaStructure.Radius);
    GetNodeAttribute(t_VirtualWallsNode, "width", m_ArenaStructure.Wall_width);
    GetNodeAttribute(t_VirtualWallsNode, "height", m_ArenaStructure.Wall_height);
    GetNodeAttribute(t_VirtualWallsNode, "walls", m_ArenaStructure.Wall_numbers);

    m_ArenaStructure.Radius = vArena_size / 2.0;

    std::ostringstream entity_id;
    CRadians wall_angle = CRadians::TWO_PI / m_ArenaStructure.Wall_numbers;
    CVector3 wall_size(m_ArenaStructure.Wall_width, 2.0 * m_ArenaStructure.Radius * Tan(CRadians::PI / m_ArenaStructure.Wall_numbers), m_ArenaStructure.Wall_height);

    /* Wall positioning */
    for (UInt32 i = 0; i < m_ArenaStructure.Wall_numbers; i++)
    {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle * i;
        // CVector3 wall_position((m_ArenaStructure.Radius) * Cos(wall_rotation), (m_ArenaStructure.Radius) * Sin(wall_rotation), 0);
        CVector3 wall_position(m_ArenaStructure.Radius * Cos(wall_rotation), m_ArenaStructure.Radius * Sin(wall_rotation), 0);
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO, CRadians::ZERO);

        CBoxEntity *wall = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size);
        AddEntity(*wall);
    }

}

void aggregationCALF::GetExperimentVariables(TConfigurationNode &t_tree)
{
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");
    GetNodeAttribute(tExperimentVariablesNode, "kilo_filename", m_strKiloOutputFileName);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

CVector2 aggregationCALF::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}

std::vector<int> aggregationCALF::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_start = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_end = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_start) >= 0.0 || obstacle_direction.DotProduct(sector_dir_end) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
}

void aggregationCALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = GetKilobotLedColor(c_kilobot_entity);

    /** WARNING: here you could update walk state based on LED kilo colour */
    /** Print kilo state*/
    // for (int kID; kID < m_vecKilobotWalks_ALF.size(); kID++)
    // {
    //     std::cout << "kID:" << kID << " state: ";
    //     switch (m_vecKilobotWalks_ALF[kID])
    //     {
    //
    //     default:
    //         std::cout << "Error no state";
    //         break;
    //     }
    // }
}

void aggregationCALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    bool bMessageToSend = false;

    /** 
     * 
     * WARNING:  2 times the same if —> try to merge
     * 
     * */
    /********************************************/
    /********* WALL AVOIDANCE STUFF *************/
    /********************************************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], m_ArenaStructure.Center);
    if (fDistance > vDistance_threshold)
    {
        std::vector<int> proximity_vec;
        CRadians collision_angle = ATan2(m_vecKilobotsPositions[unKilobotID].GetY(), m_vecKilobotsPositions[unKilobotID].GetX());
        CVector2 collision_direction = CVector2(vDistance_threshold * Cos(collision_angle + CRadians(M_PI)), vDistance_threshold * Sin(collision_angle + CRadians(M_PI))).Normalize();

        // std::cout << "collision angle: " << collision_angle << std::endl;
        // std::cout << "collision direction: " << collision_direction << std::endl;
        proximity_vec = Proximity_sensor(collision_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);

        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y)
                                               { return (x << 1) + y; });
        /* To turn off the wall avoidance decomment the following line */
        //proximity_sensor_dec = 0;

        /** Print proximity values */
        // std::cerr << "kID:" << unKilobotID << " sensor ";
        // for (int item : proximity_vec)
        // {
        //     std::cerr << item << '\t';
        // }
        // std::cerr << std::endl;

        // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;
    }
    else{
        proximity_sensor_dec = 0;
    }

    // /********************************************/
    // /********* SENDING MESSAGE ******************/
    // /********************************************/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        return;
    }
    else
    {
        /* Compose the message for a kilobot */
        tKilobotMessage.m_sID = unKilobotID;                                //ID of the receiver
        tKilobotMessage.m_sType = 0;                                        // ARK does not need to send anything
        tKilobotMessage.m_sData = proximity_sensor_dec;

        bMessageToSend = (proximity_sensor_dec == 0) ? false : true;
    }

    if (bMessageToSend)
    {

        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

        for (int i = 0; i < 9; ++i)
        {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (i == 0)
            {
                tMessage = tKilobotMessage;
            }
            else
            {
                tMessage = tEmptyMessage;
            }
            /* Packing the message */
            m_tMessages[unKilobotID].data[i * 3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2 + i * 3] = tMessage.m_sData;
            //std::cout<<" robot "<<tMessage.m_sID<<" "<<tMessage.m_sType<<std::endl;
        }
        //std::cout<<"payload: "<<tKilobotMessage.m_sData<<std::endl;
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, &m_tMessages[unKilobotID]);
    }
    else
    {
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, NULL);
    }
}
void aggregationCALF::PostExperiment()
{
    std::cout << "END\n";
}
void aggregationCALF::PostStep()
{
    // std::cout << "Time: " << m_fTimeInSeconds << std::endl;
    internal_counter += 1;

    /**
     *
     * WARNING:  decomment to LOG kiloState
     *
     * */
    // if (internal_counter % m_unDataAcquisitionFrequency == 0 || internal_counter <= 1)
    // {
    //     KiloLOG();
    // }
}


void aggregationCALF::KiloLOG()
{
    // std::cerr << "Logging kiloPosition\n";

    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds;
    for (size_t kID = 0; kID < m_vecKilobotsPositions.size(); kID++)
    {
        m_kiloOutput
            << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << m_vecKilobotWalks_ALF[kID];
    }
    m_kiloOutput << std::endl;
}

CColor aggregationCALF::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;

    /** Draw the threshold for wall avoidance */
    if (SquareDistance(vec_position_on_plane, CVector2(0.0, 0.0)) < pow(vDistance_threshold + 0.005, 2) &&
        SquareDistance(vec_position_on_plane, CVector2(0.0, 0.0)) > pow(vDistance_threshold - 0.005, 2))
    {
        cColor = CColor::ORANGE;
    }

    Real fKiloVision = Distance(vec_position_on_plane, m_vecKilobotsPositions[0]);
    if (fKiloVision < 0.03 /*&& fKiloVision > 0.03 - 0.02*/)
    {
        cColor = CColor(0, 0, 125, 0);
    }

    return cColor;
}
REGISTER_LOOP_FUNCTIONS(aggregationCALF, "ALF_aggregation_loop_function")
