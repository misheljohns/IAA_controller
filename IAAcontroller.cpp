/* -------------------------------------------------------------------------- *
 *                      OpenSim:  Controller.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Chand T. John, Ajay Seth                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// Author:  Mishel Johns, Chris Ploch

//==============================================================================
//==============================================================================

// Include OpenSim and functions
#include <OpenSim/OpenSim.h>
//#include "InducedAccelerationsSolver.cpp"

// This allows us to use OpenSim functions, classes, etc., without having to
// prefix the names of those things with "OpenSim::".
using namespace OpenSim;

// This allows us to use SimTK functions, classes, etc., without having to
// prefix the names of those things with "SimTK::".
using namespace SimTK;

/**
 * This controller will try to use IAA on the model to bring it back to its starting position of its center of mass
 */
class IAAController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(IAAController, Controller);

// This section contains methods that can be called in this controller class.
public:
	/**
	 * Constructor
	 *
	 * @param aModel Model to be controlled
	 * @param aKp Position gain by which the position error will be multiplied
	 * @param aKv velocity gain by which velocity error will be multiplied
	 */
	/////////////////////////////////////////////////////////////
	// 2) Add a parameter aKv for velocity gain to the         //
	//    argument list for this function.  Also add this      //
	//    parameter to the initializer list below so that a    //
	//    new member variable kv is initialized to the value   //
	//    of aKv.  Remember to add a line describing aKv in    //
	//    the comment above (below the line describing aKp).   //
	/////////////////////////////////////////////////////////////
	IAAController(double aKp,double aKv) : Controller(), kp( aKp ), kv( aKv) 
	{
	}

	/**
	 * This function is called at every time step for every actuator.
	 *
	 * @param s Current state of the system
	 * @param controls Controls being calculated
	 */
	void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
	{
		// Get the current time in the simulation.
		double t = s.getTime();

		//Model model1 = getModel();
		//model1.getActuators
		
		
		int num = getActuatorSet().getSize();
		Muscle** listMusc = new Muscle*[num];
		double* listacts = new double[num];
		double* listmaxfrcs = new double[num];
		for(int i = 0; i < num; i++)
		{
			//get muscles
			listMusc[i] = dynamic_cast<Muscle*>	( &getActuatorSet().get(i) );
			listacts[i] = listMusc[i]->getActivation(s);
			listmaxfrcs[i] = listMusc[i]->getMaxIsometricForce();
			//listMusc[i]->getPennationAngle ??
			//listMusc[i]->getActiveForceLengthMultiplier
			//listMusc[i]->getForceVelocityMultiplier
			//std::cout << "Muscle "<< i <<" activation = " << listMusc[i]->getActivation(s)<< std::endl;
			//std::cout << "Muscle "<< i <<" max isometric force = " << listMusc[i]->getMaxIsometricForce()<< std::endl;			
		}
		
		//_model->
		//const Coordinate& Coords = _model->getCoordinateSet().get( "blockToGround_zTranslation" );
		//double z  = Coords.getValue(s);
		//double zv  = Coords.getSpeedValue(s);


		/* Desired acceleration B */
		// B = kp*(Desired position - Actual position) + kv*(- Actual velocity)



		//dummy for now
		//Vector* indacc = new Vector[num];
		//indacc[0] = Vector(

		double** indacc = new double*[num];
		for(int i = 0; i < num; ++i)
		{
			indacc[i] = new double[3];
		}

		//randomly giving values for now
		indacc[0][0] = 12.000;
		indacc[0][1] = 2.000;
		indacc[0][2] = 5.000;
		indacc[1][0] = 12.000;
		indacc[1][1] = 2.000;
		indacc[1][2] = 2.000;


		/*
		A - matrix of induced accelerations of the CoM when force is increased by 1N
		B - desired acceleration
		of the form Ax = B
		x - increase in force required for each muscle

		Underconstrained problem;
		We need to fix this while minimizing |x|, perhaps weighting it by 1/((1-listacts[i])*listmaxfrcs[i]) <so muscles with a lot more potential to increase the force will be activated while almost saturated ones have minimized forces.
		But we should be encouraging heavily loaded muscles to reduce their loads if doing that gives us accelerations in the desired direction.
		We need to check for activations going over 1, and fixing it. Also, it might sometimes not be possible to get a linear combination of accelerations in the direction we want, in which case we need to try for an approx solution.
		*/



		for(int i = 0; i < num; i++)
		{
			// Thelen muscle has only one control
			Vector muscleControl(1, 0.8);
			// Add in the controls computed for this muscle to the set of all model controls
			listMusc[i]->addInControls(muscleControl, controls);
		}

	//delete[] ListMusc;
	}

// This section contains the member variables of this controller class.
private:

	/** Gains on position and velocity error */
	double kp;
	double kv;

};


//______________________________________________________________________________
/**
 * Run a forward dynamics simulation with a controller attached to a model.
 * The model consists of a block attached by two muscles to two walls.  The
 * block can make contact with the ground.
 */
int main()
{
	bool useVisualizer = true;

	try {
		// Create an OpenSim model from the model file provided.
		//Model osimModel( "tugOfWar_model_ThelenOnly.osim" );
		Model osimModel( "C:\\OpenSim 3.0\\Models\\Gait2392_Simbody\\gait2392_simbody.osim" );
		osimModel.setUseVisualizer(useVisualizer);
		
		// Define the initial and final simulation times.
		double initialTime = 0.0;
		double finalTime = 0.2;

		// Create the controller. Pass Kp, Kv values.
		IAAController *controller = new IAAController(0.1,0.1);

		// Give the controller the Model's actuators so it knows
		// to control those actuators.
		controller->setActuators( osimModel.updActuators() );

		// Add the controller to the Model.
		osimModel.addController( controller );

		// Initialize the system and get the state representing the
		// system.
		SimTK::State& si = osimModel.initSystem();

		// Define non-zero (defaults are 0) states for the free joint.
		CoordinateSet& modelCoordinateSet = osimModel.updCoordinateSet();

		// Setup visualizer (if required).
        if (useVisualizer) {
            Visualizer& viz = osimModel.updVisualizer().updSimbodyVisualizer();
            viz.setWindowTitle("Testing controller");
		    viz.setBackgroundType(viz.GroundAndSky);
		    viz.setGroundHeight(0.0);
		    viz.setShowShadows(true);
		    //viz.setShowFrameRate(false);
		    //viz.setShowSimTime(true);
		    //viz.setShowFrameNumber(false);
        }

        // Compute initial conditions for muscles.
		//osimModel.computeEquilibriumForAuxiliaryStates(si); // this function seems to not exist anymore
		osimModel.equilibrateMuscles(si);

		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator integrator( osimModel.getMultibodySystem() );
		integrator.setAccuracy( 1.0e-4 );

		Manager manager( osimModel, integrator );

		// Examine the model.
		osimModel.printDetailedInfo( si, std::cout );

		// Print out the initial position and velocity states.
		for( int i = 0; i < modelCoordinateSet.getSize(); i++ ) {
			std::cout << "Initial " << modelCoordinateSet[i].getName()
				<< " = " << modelCoordinateSet[i].getValue( si )
				<< ", and speed = "
				<< modelCoordinateSet[i].getSpeedValue( si ) << std::endl;
		}

		// Integrate from initial time to final time.
		manager.setInitialTime( initialTime );
		manager.setFinalTime( finalTime );
		std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
		manager.integrate( si );

		// Save the simulation results.
		osimModel.printControlStorage( "tugOfWar_controls.sto" );
		manager.getStateStorage().print( "tugOfWar_states.sto" );

		std::cout << "\nResults saved. " << std::endl;
	}
    catch (const std::exception &ex) {
		
		// In case of an exception, print it out to the screen.
        std::cout << ex.what() << std::endl;

		// Return 1 instead of 0 to indicate that something
		// undesirable happened.
        return 1;
    }

	// If this program executed up to this line, return 0 to
	// indicate that the intended lines of code were executed.
	return 0;
}
