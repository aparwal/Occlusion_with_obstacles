/* Include the controller definition */
#include "footbot_chain_los.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>


/****************************************/
/****************************************/

CFootBotChainLOS::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotChainLOS::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotChainLOS::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotChainLOS::SStateData::Reset() {
   State = STATE_INIT;
   GoalVisibility = false;
   ObjectVisibility = false;
   LandmarkNum = 0;
   ChainNum = 0;
   timeinchain = 0;
}

void CFootBotChainLOS::SStateData::Init(TConfigurationNode& t_node) {
   // try {
   //    GetNodeAttribute(t_node, "approach_distance", ApproachDistance);
   //    GetNodeAttribute(t_node, "minimum_move_around_time", MinimumMoveAroundTime);
   // }
   // catch(CARGoSException& ex) {
   //    THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   // }
}  


CFootBotChainLOS::SStateData::SStateData(){
   Reset();
}

/****************************************/
/****************************************/

CFootBotChainLOS::CFootBotChainLOS() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcLEDs(NULL),
   m_pcLight(NULL),
   // m_pcRNG(NULL),
   m_pcCamera(NULL) {}

/****************************************/
/****************************************/
void CFootBotChainLOS::Init(TConfigurationNode& t_node) {
	try{
		/*
		* NOTE: ARGoS creates and initializes actuators and sensors
		* internally, on the basis of the lists provided the configuration
		* file at the <controllers><footbot_diffusion><actuators> and
		* <controllers><footbot_diffusion><sensors> sections. If you forgot to
		* list a device in the XML and then you request it here, an error
		* occurs.
		*/
		m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
		m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
	  	m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
	  	m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
	  	m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
	  	m_pcCamera 	  = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      // m_pcLEDs->SetAllColors(CColor::BLACK);
      

      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the footbot_occlusion controller for robot \"" << GetId() << "\"", ex);
   }
   /* Enable camera filtering */
   m_pcCamera->Enable();
}

/****************************************/
/****************************************/

void CFootBotChainLOS::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Set LED color */
   // m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

void CFootBotChainLOS::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

CVector2 CFootBotChainLOS::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}


void CFootBotChainLOS::ControlStep() {
   
   switch(m_sStateData.State) {
      case SStateData::STATE_INIT:{
         // m_pcLEDs->SetSingleColor(12, CColor::BLUE);
         Explore();
         break;
      }
      case SStateData::STATE_EXPLORE:{
         m_pcLEDs->SetSingleColor(12, CColor::BLUE);
         Explore();
         break;
      }
      case SStateData::STATE_REVERSE:{
         m_pcLEDs->SetSingleColor(12, CColor::WHITE);
         Reverse();
         break;
      }
      case SStateData::STATE_LANDMARK:{
         m_pcLEDs->SetSingleColor(12, CColor::RED);
         Landmark();
         break;
      }
      case SStateData::STATE_CHAIN:{
         m_pcLEDs->SetSingleColor(12, CColor::YELLOW);
         Chain();
         break;
      }
      case SStateData::STATE_PUSH:{
         m_pcLEDs->SetSingleColor(12, CColor::BLACK);
         Explore();
         break;
      }

      default: {
         LOGERR << "Unkown State: "<<m_sStateData.State << std::endl;
      }
   }
   UpdateState();
}


/****************************************/
/****************************************/


void CFootBotChainLOS::UpdateState() {

   /* Update visibility variables*/
   CheckForGoalAndObject();
   
   switch(m_sStateData.State) {
      case SStateData::STATE_INIT:{
         if (m_sStateData.ChainNum)
            m_sStateData.State = SStateData::STATE_PUSH;
         else if(m_sStateData.LandmarkNum || m_sStateData.ObjectVisibility)
            m_sStateData.State=SStateData::STATE_EXPLORE;
         break;
      }
      case SStateData::STATE_EXPLORE:{
         if(m_sStateData.ChainNum)
            m_sStateData.State = SStateData::STATE_PUSH;

         else if (!(m_sStateData.LandmarkNum ||m_sStateData.ObjectVisibility))
            m_sStateData.State = SStateData::STATE_REVERSE;

         else if ((m_sStateData.GoalVisibility) && (m_sStateData.LandmarkNum))
            m_sStateData.SStateData::STATE_LANDMARK;
         break;
      }

      case SStateData::STATE_REVERSE:{
         if(m_sStateData.LandmarkNum || m_sStateData.ObjectVisibility)
            m_sStateData.State = SStateData::STATE_LANDMARK;
         // else if(m_sStateData.ObjectVisibility & m_sStateData.GoalVisibility)
         //    m_sStateData.State=SStateData::STATE_CHAIN;
         break;
      }

      case SStateData::STATE_LANDMARK:{
         // if(m_sStateData.ObjectVisibility && m_sStateData.LandmarkNum>1) 
         //    m_sStateData.State=SStateData::STATE_EXPLORE;
         // else 
         if (m_sStateData.GoalVisibility || m_sStateData.ChainNum)
            m_sStateData.State = SStateData::STATE_CHAIN;
         break;
      }

      case SStateData::STATE_CHAIN:{
         
         if ((m_sStateData.ChainNum == 1) & (!m_sStateData.ObjectVisibility) & (!m_sStateData.GoalVisibility)){
            ++m_sStateData.timeinchain;
            if (m_sStateData.timeinchain > 40)
               m_sStateData.State = SStateData::STATE_PUSH;
         }
         else 
            m_sStateData.timeinchain = 0;

         // if(m_sStateData.ObjectVisibility & !m_sStateData.GoalVisibility)
         //    m_sStateData.State=SStateData::STATE_LANDMARK;
         break;
      }
      case SStateData::STATE_PUSH:{
         // if (m_sStateData.ObjectVisibility & !m_sStateData.LandmarkNum)
         

         // if(m_sStateData.ObjectVisibility & !m_sStateData.GoalVisibility)
         //    m_sStateData.State=SStateData::STATE_LANDMARK;
         break;
      }            

               
      default: {
         LOGERR << "Unkown State: "<<m_sStateData.State << std::endl;
      }
   }
}

/****************************************/
/****************************************/

void CFootBotChainLOS::Explore(){

   /* Get the diffusion vector to perform obstacle avoidance */
   bool bCollision;
   CVector2 cDiffusion = DiffusionVector(bCollision);
   //    /* Random walk*/
   SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion );

}

/****************************************/
/****************************************/

void CFootBotChainLOS::Reverse(){
   m_pcWheels->SetLinearVelocity(-m_sWheelTurningParams.MaxSpeed, -m_sWheelTurningParams.MaxSpeed);
}


/****************************************/
/****************************************/

void CFootBotChainLOS::Landmark(){
   m_pcWheels->SetLinearVelocity(0,0);
}


/****************************************/
/****************************************/

void CFootBotChainLOS::Chain(){
   // m_pcWheels->SetLinearVelocity(0,0);
}

/****************************************/
/****************************************/

void CFootBotChainLOS::Push(){
   // m_pcWheels->SetLinearVelocity(0,0);
}

/****************************************/
/****************************************/
void CFootBotChainLOS::CheckForGoalAndObject(){
   m_sStateData.ObjectVisibility = false;
   m_sStateData.GoalVisibility = false;
   m_sStateData.LandmarkNum = 0;
   m_sStateData.ChainNum = 0;
   
   CVector2 cAccum;
   size_t LandmarkSeen = 0;   

   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   
   /* Go through the camera readings to locate oject */
   if(! sReadings.BlobList.empty()) {

      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
         /*Look for green*/
         if(sReadings.BlobList[i]->Color == CColor::GREEN){
            /*Change state variable*/            
            m_sStateData.ObjectVisibility = true;
         }
         if(sReadings.BlobList[i]->Color == CColor::YELLOW){
            /*Change state variable*/
            ++m_sStateData.ChainNum;
         }
         if(sReadings.BlobList[i]->Color == CColor::RED){
            /*Change state variable*/            
            ++m_sStateData.LandmarkNum;
         }
      }
   }
   // if (m_sStateData.ObjectVisibility)
   //    ++m_sStateData.LandmarkNum;
   /* Get light readings */
   const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
   for(size_t i = 0; i < tReadings.size(); ++i) {
      if (tReadings[i].Value)
         m_sStateData.GoalVisibility = true;
   }  
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 */
REGISTER_CONTROLLER(CFootBotChainLOS, "footbot_chain_los_controller")