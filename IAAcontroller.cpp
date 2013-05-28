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
// AJ's InducedAccelerationSolver
#include <OpenSim/Analyses/InducedAccelerationsSolver.h>

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

	IAAController(double aKp,double aKv) : Controller(), kp( aKp ), kv( aKv) 
	{
		//modelCopy = _model->clone();
		
		//modelCopy->setAllControllersEnabled(false);
		//modelCopy->getControllerSet().get(0).setDisabled(true);
		// getControllers
		// getMycontroller.setEnabled(False)
		//
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
		//Model modelCopy( "C:\\OpenSim 3.0\\Models\\Gait2392_Simbody\\gait2392_simbody.osim" );
		Model* modelCopy = getModel().clone();
		
		// Initialize the system and get the state representing the
		// system.
		SimTK::State& si = modelCopy->initSystem();

		// Define non-zero (defaults are 0) states for the free joint.
		CoordinateSet& modelCoordinateSet = modelCopy->updCoordinateSet();

        // Compute initial conditions for muscles.
		//osimModel.computeEquilibriumForAuxiliaryStates(si); // this function seems to not exist anymore
		modelCopy->equilibrateMuscles(si);
		modelCopy->setAllControllersEnabled(false);
		//Model model1 = getModel();
		//_model->getSimbodyEngine().
		//model1.getActuators		
		
		int numMuscs = getActuatorSet().getSize();
		Muscle** listMusc = new Muscle*[numMuscs];
		double* listacts = new double[numMuscs];
		double* listmaxfrcs = new double[numMuscs];
		for(int i = 0; i < numMuscs; i++)
		{
			//get muscles
			listMusc[i] = dynamic_cast<Muscle*>	( &getActuatorSet().get(i) );
			listacts[i] = listMusc[i]->getActivation(s);
			listmaxfrcs[i] = listMusc[i]->getMaxIsometricForce();
			//listMusc[i]->getPennationAngle
			//listMusc[i]->getActiveForceLengthMultiplier
			//listMusc[i]->getForceVelocityMultiplier
			//std::cout << "Muscle "<< i <<" activation = " << listMusc[i]->getActivation(s)<< std::endl;
			//std::cout << "Muscle "<< i <<" max isometric force = " << listMusc[i]->getMaxIsometricForce()<< std::endl;			
		}
		//std::cout << numMuscs << std::endl;
		
		//_model->
		//CoordinateSet _coordSet = _model->getCoordinateSet();
		//const Coordinate& Coords = _model->getCoordinateSet().get( "blockToGround_zTranslation" );
		//double z  = Coords.getValue(s);
		//double zv  = Coords.getSpeedValue(s);

		int numdims = 3;

		/* Desired acceleration B */
		Matrix b(numdims,1);
		
		//using position of pelvis for now
		b(0,0) = -1*_model->getCoordinateSet().get( "pelvis_tx" ).getValue(s);
		b(1,0) = -1*_model->getCoordinateSet().get( "pelvis_ty" ).getValue(s);
		b(2,0) = -1*_model->getCoordinateSet().get( "pelvis_tz" ).getValue(s);

		//failed  attempt to get CoM accelerations
		
		Matrix des_com_pos(3,1);
		des_com_pos(0,0) = 0;
		des_com_pos(1,0) = 0;
		des_com_pos(2,0) = 0;

		
		//Model* model1 = getModel().clone();		
		//model1->initStateWithoutRecreatingSystem();
		//model1->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
		//model1->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
		std::cout << "before realize" << std::endl;
		modelCopy->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
		//_model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
		//_model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);//runtime error
		//std::cout << "ding" << std::endl;
		/*
		MultibodySystem &sys = _model->updMultibodySystem();
		//Vector &mobilityForces = sys.updMobilityForces(s, Stage::Dynamics);
		sys.realize(s, Stage::Dynamics);
		//mobilityForces[0] = 0;
		//mobilityForces[1] = 0;
		//sys.realize(s, Stage::Acceleration); //-crashes it every time; so giving  up on COM accleration for now
		Matrix com_pos, com_vel;
		Vec3 com_pos_vec = sys.getMatterSubsystem().calcSystemMassCenterLocationInGround(s);
		Vec3 com_vel_vec = sys.getMatterSubsystem().calcSystemMassCenterVelocityInGround(s);
		for(int i = 0; i < 3; i ++)
		{
			com_pos(i,0) = com_pos_vec(i);
			com_vel(i,0) = com_vel_vec(i);
		}
		std::cout<<"COM_position : "<<com_pos<<std::endl;
		std::cout<<"COM_velocity : "<<com_vel<<std::endl;
		b = kp*(des_com_pos - com_pos) + kv*(0 - com_vel);
		std::cout<<"COM_desired_acc : "<<b<<std::endl;
		*/

		std::cout << "dong" << std::endl;
		//get Induced Accelerations
		Matrix A(numdims,numMuscs);
		A = 1;
		
		InducedAccelerationsSolver iaaSolver(*_model);
		std::cout << "after iaa solver" << std::endl;
		// Compute velocity contribution		
		Vector udot_vel = iaaSolver.solve(s, "velocity"); 
		std::cout<<"acc_vel : "<<udot_vel<<std::endl;
		
		// Compute gravity contribution
		Vector udot_grav = iaaSolver.solve(s, "gravity");
		std::cout<<"acc_grav : "<<udot_grav<<std::endl;

		//int i = 1;
		//Vector udot_musc1 = iaaSolver.solve(s,listMusc[i]->getName(),true); //this uses the controller in the model, so this goes in an infinite loop!!!
		//std::cout<<"acc_muscl1 : "<<udot_musc1<<std::endl;
		//Vec3 udot_com_musc1 = iaaSolver.getInducedMassCenterAcceleration(s);
		//std::cout<<"acc_com_muscl1 : "<<udot_com_musc1<<std::endl;



		std::cout<<"A : "<<A;
		std::cout<<"b : "<<b;
		Matrix W(numMuscs,numMuscs);
		W = 1;
		Matrix WTW(numMuscs,numMuscs);
		WTW = (~W)*W;
		//std::cout<<WTW;
		Matrix WTWinv(numMuscs,numMuscs);
		FactorLU WTWLU(WTW);
		WTWLU.inverse(WTWinv);
		//std::cout<<WTWinv;
		Matrix AWTWAT(numdims,numdims);
		AWTWAT = A*WTW*(~A);
		Matrix AWTWATinv(numdims,numdims);
		FactorLU AWTWATLU(AWTWAT);
		AWTWATLU.inverse(AWTWATinv);
		Matrix x(numMuscs,1);
		x = WTWinv*(~A)*AWTWATinv*b;
		std::cout<<"x :"<<x<<"Working, muhahahaha"<<std::endl;
	
	
	//x = inv(W'*W)*A'*inv(A*inv(W'*W)*A')*b

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



		//soln for A'*inv(A*A') for 2x1 matrix
		/*double* x = new double[num];
		x[0] = indacc[0]*B[0]/(indacc[0]*indacc[0] + indacc[1]*indacc[1]);
		x[1] = indacc[1]*B[0]/(indacc[0]*indacc[0] + indacc[1]*indacc[1]);*/

		//appplying the weights
		//x = Wy, where W - matrix of weights, diagonal elements = 1/((1-listacts[i])*listmaxfrcs[i])
		//y = inv(w)*x

		//flawed method, look at new eqn in README
		//double** W = new double*[num];
		//for(int i = 0; i < num; ++i)
		//{
		//	W[i] = new double[numd];
		//}
		////giving values
		//W[0][0] = 1/((1-listacts[0])*listmaxfrcs[0]);
		//W[0][1] = 0.000;
		//W[1][0] = 0.000;
		//W[1][1] = 1/((1-listacts[1])*listmaxfrcs[1]);
		////we need to normalize W, not doing it now..

		//double* y = new double[num];
		//y[0] = (1/W[0][0]*W[1][1])*W[1][1]*x[0];
		//y[1] = (1/W[0][0]*W[1][1])*W[0][0]*x[1];

		for(int i = 0; i < numMuscs; i++)
		{
			// Thelen muscle has only one control
			Vector muscleControl(1,0.5);// x[i]
			// Add in the controls computed for this muscle to the set of all model controls
			listMusc[i]->addInControls(muscleControl, controls);
			//Vector muscleControl2(1,0.1);
			//listMusc[0]->addInControls(muscleControl2, controls);
		}
		
	//delete[] ListMusc;
	}

// This section contains the member variables of this controller class.
private:

	/** Gains on position and velocity error */
	double kp;
	double kv;
	//Model* modelCopy;

};


//______________________________________________________________________________
/**
 * Run a forward dynamics simulation with a controller attached to a model.
 * The model consists of a block attached by two muscles to two walls.  The
 * block can make contact with the ground.
 */
int main()
{
	
	bool useVisualizer = false;

	try {
		// Create an OpenSim model from the model file provided.
		//Model osimModel( "tugOfWar_model_ThelenOnly.osim" );
		Model osimModel( "C:\\OpenSim 3.0\\Models\\Gait2392_Simbody\\gait2392_simbody.osim" );
		osimModel.setUseVisualizer(useVisualizer);
		
		// Define the initial and final simulation times.
		double initialTime = 0.0;
		double finalTime = 1.0;

		// Create the controller. Pass Kp, Kv values.
		IAAController *controller = new IAAController(100,0);

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
		integrator.setAccuracy( 1.0e-8 );

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
		osimModel.printControlStorage( "output_controls.sto" );
		manager.getStateStorage().print( "output_states.sto" );

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
