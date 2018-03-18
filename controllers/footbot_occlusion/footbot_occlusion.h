/*
 * AUTHOR: Anand Parwal <aparwal@wpi.edu>
 *
 * Occlusion controller implemented for swarm intelligence project
 *
 * This controller follows the paper:
 * Chen, Jianing, et al. "Occlusion-based cooperative transport with 
 * a swarm of miniature mobile robots." 
 * IEEE Transactions on Robotics 31.2 (2015): 307-321.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/occlusion.argos
 */

#ifndef FOOTBOT_OCCLUSION_H
#define FOOTBOT_OCCLUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class in argos.
 */
class CFootBotOcclusion : public CCI_Controller {

public:

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_occlusion_controller><parameters><wheel_turning>
    * section.
    */
    
    //blalbalba

	/*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The two possible states in which the controller can be */
      enum EState {
         STATE_SEARCH_OBJECT = 0,
         STATE_APPROACH_OBJECT,
      	 STATE_CHECK_FOR_GOAL,
      	 STATE_GOAL_NOT_OCCLUDED, 	//(Move around object)
         STATE_GOAL_OCCLUDED, 		//(Push object)
      } State;

      /* True when the goal is not occluded */
      bool GoalVisibility;

      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
   };



   /* Class constructor. */
   CFootBotOcclusion();

   /* Class destructor. */
   virtual ~CFootBotOcclusion() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_occlusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * Returns true if the robot is currently searching for object.
    */
   inline bool IsSearching() const {
      return m_sStateData.State == SStateData::STATE_SEARCH_OBJECT;
   }

   /*
    * Returns true if the robot is currently approaching the object.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_APPROACH_OBJECT;
   }

   /*
    * Returns true if the robot is currently at the object and cheking for goal.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_CHECK_FOR_GOAL;
   }

   /*
    * Returns true if the robot is currently moving around the object.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_GOAL_NOT_OCCLUDED;
   }

   /*
    * Returns true if the robot is currently pushing the object.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_GOAL_OCCLUDED;
   }
            // STATE_SEARCH_OBJECT = 0,
         // STATE_APPROACH_OBJECT,
      	 // STATE_CHECK_FOR_GOAL,
      	 STATE_GOAL_NOT_OCCLUDED = 0, 	//(Move around object)
         STATE_GOAL_OCCLUDED,