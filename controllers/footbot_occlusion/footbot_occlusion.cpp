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

void CFootBotOcclusion::SStateData::Reset() {
   State = STATE_GOAL_NOT_OCCLUDED;
   GoalVisibility = true;
}

// void CFootBotOcclusion::SStateData::Init(TConfigurationNode& t_node) {
//    GoalVisibility = true;
// }
CFootBotOcclusion::SStateData::SStateData(){
   State = STATE_GOAL_NOT_OCCLUDED;
   GoalVisibility = true;
}



/****************************************/
/****************************************/

CFootBotOcclusion::CFootBotOcclusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
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
      m_pcLEDs->SetAllColors(CColor::RED);
      

	   /*
	    * Parse the configuration file
	    *
	    * Variables here are read from the config file so we don't
	    * have to recompile if we want to try other settings.
	    */
	   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
	   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
	   /* Controller state */
	   // m_sStateData.Init(GetNode(t_node, "state"));
      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the footbot_occlusion controller for robot \"" << GetId() << "\"", ex);
   }
}


/****************************************/
/****************************************/

void CFootBotOcclusion::ControlStep() {
   // LOG << m_sStateData.State << std::endl;
   switch(m_sStateData.State) {
      case SStateData::STATE_SEARCH_OBJECT:{
         GoalNotOccluded();
         break;
      }
      case SStateData::STATE_APPROACH_OBJECT:{
         GoalNotOccluded();
         break;
      }
      case SStateData::STATE_CHECK_FOR_GOAL:{
         GoalNotOccluded();
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
   m_pcLEDs->SetAllColors(CColor::RED);
}


/****************************************/
/****************************************/

void CFootBotOcclusion::UpdateState() {
   /* Reset state flags */
   m_sStateData.GoalVisibility = false;


   /*Look for light*/
   const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
   /* Calculate a normalized vector that points to the closest light */
   CVector2 cAccum;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }
   LOG<< cAccum.Length() <<std::endl;
   if(cAccum.Length() > 0.0f) {
   	  m_sStateData.GoalVisibility = true;
        m_sStateData.State = SStateData::STATE_GOAL_NOT_OCCLUDED;
      /* Make the vector long as 1/4 of the max speed */
      // cAccum.Normalize();
      // cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   }
   else m_sStateData.State = SStateData::STATE_GOAL_OCCLUDED;



}

/****************************************/
/****************************************/

// TODO: Plain diffision for now, needs to be chaged later

// void CFootBotOcclusion::SearchObject(){}
// void CFootBotOcclusion::ApproachObject(){}
// void CFootBotOcclusion::CheckForGoal(){}
void CFootBotOcclusion::GoalNotOccluded(){
   m_pcLEDs->SetAllColors(CColor::GREEN);
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
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
   /*Do a check for light*/
   UpdateState();
}

void CFootBotOcclusion::GoalOccluded(){
	m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
   UpdateState();
	/* Set LED color */
    m_pcLEDs->SetAllColors(CColor::BLUE);
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