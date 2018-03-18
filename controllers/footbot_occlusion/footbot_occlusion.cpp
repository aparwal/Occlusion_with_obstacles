/* Include the controller definition */
#include "footbot_occlusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>


/****************************************/
/****************************************/

CFootBotOcclusion::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotOcclusion::SDiffusionParams::Init(TConfigurationNode& t_node) {
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

void CFootBotOcclusion::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CFootBotOcclusion::SStateData::Reset() {
   State = STATE_SEARCH_OBJECT;
   GoalVisibility = false;
   ObjectVisibility = false;
   ObjectReached = false;
}

// void CFootBotOcclusion::SStateData::Init(TConfigurationNode& t_node) {
//    GoalVisibility = true;
// }
CFootBotOcclusion::SStateData::SStateData(){
   State = STATE_SEARCH_OBJECT;
   GoalVisibility = false;
   ObjectVisibility = false;
   ObjectReached = false;
}

/****************************************/
/****************************************/

CFootBotOcclusion::CFootBotOcclusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcLEDs(NULL),
   m_pcLight(NULL),
   // m_pcRNG(NULL),
   m_pcCamera(NULL) {}

/****************************************/
/****************************************/
void CFootBotOcclusion::Init(TConfigurationNode& t_node) {
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
      m_pcLEDs->SetAllColors(CColor::BLACK);
      

      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the footbot_occlusion controller for robot \"" << GetId() << "\"", ex);
   }
   /* Enable camera filtering */
   m_pcCamera->Enable();
}


/****************************************/
/****************************************/

void CFootBotOcclusion::ControlStep() {
   // LOG << m_sStateData.State << std::endl;
   switch(m_sStateData.State) {
      case SStateData::STATE_SEARCH_OBJECT:{
         m_pcLEDs->SetAllColors(CColor::BLUE);
         SearchObject();
         break;
      }
      case SStateData::STATE_APPROACH_OBJECT:{
         m_pcLEDs->SetAllColors(CColor::YELLOW);
         SearchObject();
         break;
      }
      case SStateData::STATE_CHECK_FOR_GOAL:{
         GoalOccluded();
         break;
      }

      case SStateData::STATE_GOAL_NOT_OCCLUDED:{
         
      	GoalNotOccluded();
         break;
      }
      case SStateData::STATE_GOAL_OCCLUDED:{
         
         GoalOccluded();
         break;
      }         
      default: {
         LOGERR << "Unkown State: "<<m_sStateData.State << std::endl;
      }
   }
}

/****************************************/
/****************************************/

void CFootBotOcclusion::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Set LED color */
   m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

void CFootBotOcclusion::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

void CFootBotOcclusion::Diffuse(){
   /* Doing basic diffusion here */
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAccumulator.Angle()) &&
      cAccumulator.Length() < m_sDiffusionParams.Delta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_sWheelTurningParams.MaxSpeed,m_sWheelTurningParams.MaxSpeed);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAccumulator.Angle().GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_sWheelTurningParams.MaxSpeed, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_sWheelTurningParams.MaxSpeed);
      }
   }
}


/****************************************/
/****************************************/

CVector2 CFootBotOcclusion::DiffusionVector(bool& b_collision) {
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

/****************************************/
/****************************************/

void CFootBotOcclusion::UpdateState() {
   /* Reset state flags */
   m_sStateData.GoalVisibility = false;

   if(m_sStateData.ObjectVisibility)
      m_sStateData.State=SStateData::STATE_APPROACH_OBJECT;

   else if(m_sStateData.ObjectVisibility && m_sStateData.ObjectReached){
      /*Look for light*/
      const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
      /* Calculate a normalized vector that points to the closest light */
      CVector2 cAccum;
      for(size_t i = 0; i < tReadings.size(); ++i) {
         cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
      }
      // LOG<< cAccum.Length() <<std::endl;
      if(cAccum.Length() > 0.0f) {
           m_sStateData.GoalVisibility = true;
           m_sStateData.State = SStateData::STATE_GOAL_NOT_OCCLUDED;
      }

      else m_sStateData.State = SStateData::STATE_GOAL_OCCLUDED;
   }

}

/****************************************/
/****************************************/

void CFootBotOcclusion::SearchObject(){

   m_sStateData.ObjectVisibility = false;
   CVector2 cAccum;
   size_t unBlobsSeen = 0;   

   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   
   /* Go through the camera readings to locate oject */
   if(! sReadings.BlobList.empty()) {


      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
         /*Look for green*/
         if(sReadings.BlobList[i]->Color == CColor::GREEN) {
         
            cAccum += CVector2(m_sWheelTurningParams.MaxSpeed,
                               sReadings.BlobList[i]->Angle);
            /*Change state variable*/            
            m_sStateData.ObjectVisibility = true;
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
         }

      }
   }

   /* Get the diffusion vector to perform obstacle avoidance */
   bool bCollision;
   CVector2 cDiffusion = DiffusionVector(bCollision);

   if (m_sStateData.ObjectVisibility){
      cAccum /= unBlobsSeen;
      /*Move towards object*/
      SetWheelSpeedsFromVector(cAccum+m_sWheelTurningParams.MaxSpeed * cDiffusion );
   }
   else{
      /* Random walk*/
       SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion );
   }

   UpdateState();
}

// TODO: Plain diffision for now, needs to be chaged later

// void CFootBotOcclusion::ApproachObject(){
//     Get the diffusion vector to perform obstacle avoidance 
//       bool bCollision;
//       CVector2 cDiffusion = DiffusionVector(bCollision);
// }
// void CFootBotOcclusion::CheckForGoal(){}

void CFootBotOcclusion::GoalNotOccluded(){
   m_pcLEDs->SetAllColors(CColor::RED);
	
   /*Do a check for light*/
   // UpdateState();
}

void CFootBotOcclusion::GoalOccluded(){
	m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
	/* Set LED color */
    m_pcLEDs->SetAllColors(CColor::RED);

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
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotOcclusion, "footbot_occlusion_controller")