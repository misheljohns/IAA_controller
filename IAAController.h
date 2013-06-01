#ifndef OPENSIM_IAAController_H_
#define OPENSIM_IAAController_H_

// Authors:  Mishel Johns, Chris Ploch

//==============================================================================
//==============================================================================

#include "Controller.h"


namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * IAAController is a concrete controller that excites muscles in response
 * to muscle lengthening to simulate a simple stretch reflex. This controller is 
 * meant to serve as an example how to implement a controller in
 * OpenSim. It is intended for demonstrative purposes only. 
 *
 * @author  Ajay Seth
 */
class OSIMSIMULATION_API IAAController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(IAAController, Controller);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a IAAController.*/
    /**@{**/  	
	OpenSim_DECLARE_PROPERTY(gain, double, 
		"Factor by which the stretch reflex is scaled." );

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
	/** Default constructor. */
	IAAController();

	// Uses default (compiler-generated) destructor, copy constructor and copy 
    // assignment operator.

	/** Convenience constructor 
	* @param gain		gain on the stretch response
	*/
	IAAController(double gain);

	/** Compute the controls for actuators (muscles)
	 *  This method defines the behavior of the IAAController 
	 *
	 * @param s			system state 
	 * @param controls	writable model controls
	 */
	void computeControls(const SimTK::State& s, SimTK::Vector &controls) const OVERRIDE_11;


private:
	// Connect properties to local pointers.  */
	void constructProperties();
	// ModelComponent interface to connect this component to its model
	void connectToModel(Model& aModel);

	//=============================================================================
};	// END of class IAAController

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_IAAController_H_