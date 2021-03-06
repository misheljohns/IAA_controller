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
//#include <OpenSim/Analyses/InducedAccelerationsSolver.h>
#include "InducedAccelerationsSolver.h"

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
	//std::cout<<"time:    XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX   "<<t<<std::endl;
		//Model modelCopy( "C:\\OpenSim 3.0\\Models\\Gait2392_Simbody\\gait2392_simbody.osim" );
		Model* modelCopy = getModel().clone();
		modelCopy->setUseVisualizer(false);
		modelCopy->updControllerSet().clearAndDestroy();
		
		
		// Initialize the system and get the state representing the
		// system.
		modelCopy->initSystem();
		//si = s;

		// Define non-zero (defaults are 0) states for the free joint.
		//CoordinateSet& modelCoordinateSet = modelCopy->updCoordinateSet();

        // Compute initial conditions for muscles.
		//osimModel.computeEquilibriumForAuxiliaryStates(si); // this function seems to not exist anymore
		//modelCopy->equilibrateMuscles(si);
		
		
		//std::cout<<"orig model visual:"<<_model->getUseVisualizer()<<std::endl;
		//std::cout<<"orig model controllers  enabled:"<<_model->getAllControllersEnabled()<<std::endl;
		//std::cout<<"copy model visual:"<<modelCopy->getUseVisualizer()<<std::endl;
		//std::cout<<"copy model controllers  enabled:"<<modelCopy->getAllControllersEnabled()<<std::endl;

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
	//std::cout <<"Number of muscles:"<< numMuscs << std::endl;
		//std::cout <<"muscle 0 excitation:"<< listMusc[0]->getExcitation(s) << std::endl;
		//std::cout <<"muscle 0 control:"<< listMusc[0]->getControl(s) << std::endl;
		//std::cout <<"Number of muscles:"<< numMuscs << std::endl;
		
		
		//_model->
		//CoordinateSet _coordSet = _model->getCoordinateSet();
		//const Coordinate& Coords = _model->getCoordinateSet().get( "blockToGround_zTranslation" );
		//double z  = Coords.getValue(s);
		//double zv  = Coords.getSpeedValue(s);

		int numdims = 3;

		/* Desired acceleration B */
		Matrix b(numdims,1);

		//
		////using position of pelvis for now
		//b(0,0) = -1*_model->getCoordinateSet().get( "pelvis_tx" ).getValue(s);
		//b(1,0) = -1*_model->getCoordinateSet().get( "pelvis_ty" ).getValue(s);
		//b(2,0) = -1*_model->getCoordinateSet().get( "pelvis_tz" ).getValue(s);

		//failed  attempt to get CoM accelerations
		
		Matrix des_com_pos(3,1);
		des_com_pos(0,0) = 0;
		des_com_pos(1,0) = 0.05;
		des_com_pos(2,0) = 0;
	//std::cout<<"Desired CoM position : "<<des_com_pos<<std::endl;
		
		//Model* model1 = getModel().clone();		
		//model1->initStateWithoutRecreatingSystem();
		//model1->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
		//model1->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
		//std::cout << "before realize" << std::endl;
		//modelCopy->getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
		//_model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
		//_model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);//runtime error
		//std::cout << "ding" << std::endl;
		
		MultibodySystem &sys = _model->updMultibodySystem();
		//Vector &mobilityForces = sys.updMobilityForces(s, Stage::Dynamics);
		//sys.realize(s, Stage::Dynamics);
		//mobilityForces[0] = 0;
		//mobilityForces[1] = 0;
		//sys.realize(s, Stage::Acceleration); //-crashes it every time; so giving  up on COM accleration for now
		
		Matrix com_pos(3,1);
		Matrix com_vel(3,1);
		Vec3 com_pos_vec = sys.getMatterSubsystem().calcSystemMassCenterLocationInGround(s);
		Vec3 com_vel_vec = sys.getMatterSubsystem().calcSystemMassCenterVelocityInGround(s);
		for(int i = 0; i < 3; i ++)
		{
			com_pos(i,0) = com_pos_vec(i);
			com_vel(i,0) = com_vel_vec(i);
		}
		//std::cout<<"COM_position vector : "<<com_pos_vec<<std::endl;
		//std::cout<<"COM_velocity vector : "<<com_vel_vec<<std::endl;
	//std::cout<<"COM_position matrix : "<<com_pos<<std::endl;
		//std::cout<<"COM_velocity matrix : "<<com_vel<<std::endl;
		b = kp*(des_com_pos - com_pos) + kv*(0 - com_vel);
		b(1,0) = b(1,0) + 9.8;
	//std::cout<<"COM_desired_acc (b): "<<b<<std::endl;
		

		//std::cout << "After b calc" << std::endl;
		
		
		//get Induced Accelerations
		Matrix A(numdims,numMuscs);
		//A = 1;
		
		InducedAccelerationsSolver iaaSolver(*modelCopy);
		//std::cout << "after iaa solver init" << std::endl;
		
		/*
		// Compute velocity contribution		
		Vector udot_vel = iaaSolver.solve(s, "velocity"); 
		std::cout<<"acc_vel : "<<udot_vel<<std::endl;
		
		// Compute gravity contribution
		Vector udot_grav = iaaSolver.solve(s, "gravity");
		std::cout<<"acc_grav : "<<udot_grav<<std::endl;
		*/
		
		//std::cout<<"Nummuscs"<<numMuscs<<std::endl;

		for(int i = 0; i < numMuscs; i++)
		{
			//std::cout<<i;
			//std::cout<<"In loop for IAA"<<std::endl;
			Vector udot_musc = iaaSolver.solve(s,listMusc[i]->getName(),true);//computepotentials for now
			//std::cout << "after iaa solving" << std::endl;
			//std::cout<<"acc_muscl : "<<udot_musc<<std::endl;
			//std::cout<<"IAA model controllers"<<iaaSolver._modelCopy.getAllControllersEnabled()<<std::endl;

			//Vec3 udot_com_musc = iaaSolver.getInducedMassCenterAcceleration(s);
			//iaaSolver.getInducedBodyAcceleration(s,"pelvis");
			//const SimTK::State& s_solver = iaaSolver.getSolvedState(s);
			//Vec3 udot_com_musc = modelCopy->getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_solver);
			Vec3 udot_com_musc = sys.getMatterSubsystem().calcSystemMassCenterAccelerationInGround(iaaSolver._modelCopy.getWorkingState());
			

			//std::cout << "after iaa com solving" << std::endl;
			//std::cout<<"acc_com_muscl"<<i<<" : "<<udot_com_musc<<std::endl;
			//A.row(i) = (Matrix) udot_com_musc;
			for(int j = 0; j < 3; j ++)
			{
				//std::cout << "inside 2nd loop" << std::endl;
				A(j,i) = udot_com_musc(j);
				//A(j,i) = 1;
			}
		}

		//std::cout << "after iaa com solving" << std::endl;
		

	//std::cout<<"A : "<<A;
		//std::cout<<"size of A : "<<A.nrow()<<" x "<<A.ncol()<<std::endl;
		//std::cout<<"b : "<<b;
		Matrix W(numMuscs,numMuscs);
		W = 0;
		for(int i = 0; i < numMuscs; i++)
		{
			W(i,i) = 10/listmaxfrcs[i]+0.1/(1-listacts[i])-0.1/listacts[i]; //including the activations this way is not a very good soln, but it's a hack to make the optimization keep the activations below 1
		}
		//W = 1;
	//std::cout<<"W :"<<W<<std::endl;
		Matrix WTW(numMuscs,numMuscs);
		WTW = (~W)*W;
		//std::cout<<WTW;
		Matrix WTWinv(numMuscs,numMuscs);
		FactorLU WTWLU(WTW);
		WTWLU.inverse(WTWinv);
		//std::cout<<WTWinv;
		Matrix AWTWinvAT(numdims,numdims);
		AWTWinvAT = A*WTWinv*(~A);
		//std::cout<"AWTWAT"<<AWTWAT;
		Matrix AWTWinvATinv(numdims,numdims);
		FactorLU AWTWinvATLU(AWTWinvAT);
		AWTWinvATLU.inverse(AWTWinvATinv);
		//std::cout<<"AWTWATinv"<<AWTWATinv;
		Matrix x(numMuscs,1);
		x = WTWinv*(~A)*AWTWinvATinv*b;
	//std::cout<<"x :"<<x<<"Working, muhahahaha"<<std::endl;
	//std::cout<<"Ax :"<<A*x<<"Working, muhahahaha"<<std::endl;
	
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
		//std::cout<<"x :"<<x<<std::endl;
		for(int i = 0; i < numMuscs; i++)
		{
			//double deltaAct = x(i,0)/listmaxfrcs[i];
			double act = x(i,0)/listmaxfrcs[i];
		//std::cout<<"present act :"<<listacts[i]<<std::endl;
			if(act > 0.99)
				act = 0.999;
			else if(act < 0.001)
				act = 0.001;
		//std::cout<<"new act :"<<act<<std::endl;	
			// Thelen muscle has only one control
			Vector muscleControl(1,act);// x[i]
			// Add in the controls computed for this muscle to the set of all model controls
			listMusc[i]->addInControls(muscleControl, controls);
			//Vector muscleControl2(1,0.1);
			//listMusc[0]->addInControls(muscleControl2, controls);
		}

		free(modelCopy);
		//delete &iaaSolver;
	//delete[] ListMusc;
		_model->updVisualizer().getSimbodyVisualizer().report(s);
	}
	

// This section contains the member variables of this controller class.
private:

	/** Gains on position and velocity error */
	double kp;
	double kv;
	//Model* _modelCopy;

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
		//Model osimModel( "C:\\OpenSim 3.0\\Models\\Gait2392_Simbody\\gait2392_simbody.osim" );
		//Model osimModel( "gait2392_simbody.osim" );
		//Model osimModel( "gait2392_simbody_weld.osim" );
		//Model osimModel( "gait10dof18musc_weld_fast.osim" );
		Model osimModel( "6dof_fast.osim" );

		//Vec3 grav;
		//grav(0) = 0.0;
		//grav(1) = 0.0;
		//grav(2) = 0.0;
		//osimModel.setGravity(grav);

		//Model osimModel( "gait10dof18musc.osim" );
		osimModel.setUseVisualizer(useVisualizer);
		
		// Define the initial and final simulation times.
		double initialTime = 0.0;
		double finalTime = 1.0;

		// Create the controller. Pass Kp, Kv values.
		IAAController *controller = new IAAController(500,10);

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

		// Compute initial conditions for muscles.
		//osimModel.computeEquilibriumForAuxiliaryStates(si); // this function seems to not exist anymore
		osimModel.equilibrateMuscles(si);

		// Setup visualizer (if required).
        if (useVisualizer) {
            Visualizer& viz = osimModel.updVisualizer().updSimbodyVisualizer();
            viz.setWindowTitle("Testing controller");
		    //viz.setBackgroundType(viz.GroundAndSky);
		    viz.setGroundHeight(0.0);
		    //viz.setShowShadows(true);
			//viz.setMode(SimTK::Visualizer::Mode::PassThrough);
			//viz.setDesiredFrameRate(600);
			
		    viz.setShowFrameRate(false);
		    viz.setShowSimTime(true);
			//viz.set
		    //viz.setShowFrameNumber(false);
        }

		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator integrator( osimModel.getMultibodySystem() );
		integrator.setAccuracy( 1.0e-3 );
		//SimTK::CPodesIntegrator
		//SimTK::VerletIntegrator

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
		////manager.setUseConstantDT(true);
		////manager.setDT()
		manager.setUseSpecifiedDT(true);
		double aDT[1000];
		double step = (finalTime - initialTime)/1000;
		for (int i = 0; i < 1000; i++)
		{
			aDT[i] = step;
		}
		////manager.doIntegration
		////std::cout<<manager.getFixedStepSize(1);
		manager.setDTArray(1000,aDT);
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

