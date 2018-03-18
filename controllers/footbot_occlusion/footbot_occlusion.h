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
/* Definitions for random number generation */
// #include <argos3/core/utility/math/rng.h>


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
    * <controllers><footbot_foraging_controller><parameters><wheel_turning>
    * section.
    */
   // struct SWheelTurningParams {
   //    /*
   //     * The turning mechanism.
   //     * The robot can be in three different turning states.
   //     */
   //    enum ETurningMechanism
   //    {
   //       NO_TURN = 0, // go straight
   //       SOFT_TURN,   // both wheels are turning forwards, but at different speeds
   //       HARD_TURN    // wheels are turning with opposite speeds
   //    } TurningMechanism;
   //    /*
   //     * Angular thresholds to change turning state.
   //     */
   //    CRadians HardTurnOnAngleThreshold;
   //    CRadians SoftTurnOnAngleThreshold;
   //    CRadians NoTurnAngleThreshold;
   //    /* Maximum wheel speed */
   //    Real MaxSpeed;

   //    void Init(TConfigurationNode& t_tree);
   // };

    
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
      // void Init(TConfigurationNode& t_node);
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
   inline bool IsApproaching() const {
      return m_sStateData.State == SStateData::STATE_APPROACH_OBJECT;
   }

   /*
    * Returns true if the robot is currently at the object and cheking for goal.
    */
   inline bool IsChecking() const {
      return m_sStateData.State == SStateData::STATE_CHECK_FOR_GOAL;
   }

   /*
    * Returns true if the robot is currently moving around the object.
    */
   inline bool IsNotOccluded() const {
      return m_sStateData.State == SStateData::STATE_GOAL_NOT_OCCLUDED;
   }

   /*
    * Returns true if the robot is currently pushing the object.
    */
   inline bool IsOccluded() const {
      return m_sStateData.State == SStateData::STATE_GOAL_OCCLUDED;
   }



private:

   /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
 */   /* Pointer to the omnidirectional camera sensor */
   void UpdateState();



   /*
    * Executes the different states.
    */
   
   // TODO

   // void SearchObject();
   // void ApproachObject();
   // void CheckForGoal();
   void GoalNotOccluded();
   void GoalOccluded();


   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* The random number generator */
   // CRandom::CRNG* m_pcRNG;
   /* The controller state information */
   SStateData m_sStateData;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_occlusion_controller><diffusion> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

};

#endif