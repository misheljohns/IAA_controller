// Author:  Mishel Johns, Chris Ploch

//==============================================================================
//==============================================================================

// Include OpenSim and functions
#include <OpenSim/OpenSim.h>

// AJ's InducedAccelerationSolver
#include "InducedAccelerationsSolver.h"

// This allows us to use OpenSim functions, classes, etc., without having to
// prefix the names of those things with "OpenSim::".
using namespace OpenSim;

// This allows us to use SimTK functions, classes, etc., without having to
// prefix the names of those things with "SimTK::".
using namespace SimTK;

/**
 * This controller will try to use IAA on the model to bring it back to the specified position of its center of mass
 */
class IAAController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(IAAController, Controller);

// This section contains methods that can be called in this controller class.
public:
	/**
	 * Constructor
	 *
	 * @param aKp Position gain by which the position error will be multiplied
	 * @param aKv velocity gain by which velocity error will be multiplied
	 * @param modelCopy Model to be controlled
	 */

	IAAController(double aKp,double aKv, Model* modelCopy) : Controller(), kp( aKp ), kv( aKv)
	{

		_modelCopy = modelCopy->clone();
		_modelCopy->setUseVisualizer(false);
		_modelCopy->updControllerSet().clearAndDestroy();
		_modelCopy->initSystem();

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
				
		int numMuscs = getActuatorSet().getSize();
		int numdims = 3;

		Muscle** listMusc = new Muscle*[numMuscs];
		double* listacts = new double[numMuscs];
		double* listmaxfrcs = new double[numMuscs];
		for(int i = 0; i < numMuscs; i++)
		{
			//get muscles
			listMusc[i] = dynamic_cast<Muscle*>	( &getActuatorSet().get(i) );
			listacts[i] = listMusc[i]->getActivation(s);
			listmaxfrcs[i] = listMusc[i]->getMaxIsometricForce()*listMusc[i]->getActiveForceLengthMultiplier(s)*listMusc[i]->getForceVelocityMultiplier(s);
			//std::cout<<"length multiplier"<<listMusc[i]->getActiveForceLengthMultiplier(s)<<std::endl;
			//std::cout<<"velocity multiplier"<<listMusc[i]->getForceVelocityMultiplier(s)<<std::endl;
		}
		//std::cout <<"Number of muscles:"<< numMuscs << std::endl;
						
		Matrix des_com_pos(3,1);
		//des_com_pos(0,0) = 0.0;
		//des_com_pos(1,0) = -0.01;
		//des_com_pos(2,0) = 0.0;

		des_com_pos(0,0) = 0.1*sin(0.5*t);
		des_com_pos(1,0) = 0.1*cos(0.5*t);
		des_com_pos(2,0) = 0.001*t;
		//std::cout<<"Desired CoM position : "<<des_com_pos<<std::endl;
		




		MultibodySystem &sys = _model->updMultibodySystem();
		
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

		/* Desired acceleration B */
		Matrix b(numdims,1);
		b = kp*(des_com_pos - com_pos) + kv*(0 - com_vel);
		b(1,0) = b(1,0) + 9.8;
		//std::cout<<"COM_desired_acc (b): "<<b<<std::endl;
		
		//get Induced Accelerations
		Matrix A(numdims,numMuscs);
		InducedAccelerationsSolver iaaSolver(*_modelCopy);
		//std::cout << "after iaa solver init" << std::endl;
		
		/*
		// Compute velocity contribution		
		Vector udot_vel = iaaSolver.solve(s, "velocity"); 
		std::cout<<"acc_vel : "<<udot_vel<<std::endl;
		
		// Compute gravity contribution
		Vector udot_grav = iaaSolver.solve(s, "gravity");
		std::cout<<"acc_grav : "<<udot_grav<<std::endl;
		*/
		
		for(int i = 0; i < numMuscs; i++)
		{
			Vector udot_musc = iaaSolver.solve(s,listMusc[i]->getName(),true);//computepotentials for now
			//std::cout<<"acc_muscl : "<<udot_musc<<std::endl;
			Vec3 udot_com_musc = sys.getMatterSubsystem().calcSystemMassCenterAccelerationInGround(iaaSolver._modelCopy.getWorkingState());
			//std::cout<<"acc_com_muscl"<<i<<" : "<<udot_com_musc<<std::endl;

			for(int j = 0; j < 3; j ++)
			{
				A(j,i) = udot_com_musc(j);
			}
		}
		//std::cout<<"A : "<<A;
		//std::cout<<"size of A : "<<A.nrow()<<" x "<<A.ncol()<<std::endl;

		/* Trying to find optimum controls from IAA */
		Matrix W(numMuscs,numMuscs);
		W = 0;
		for(int i = 0; i < numMuscs; i++)
		{
			if(!(listmaxfrcs[i] <= 0)&&!(listmaxfrcs[i] >= 0))
			{
				std::cout<<"max force of muscle "<<i<<" is undefined."<<std::endl;
				listmaxfrcs[i] = 1;
			}
			W(i,i) = 1 + 1000/(0.1+abs(listmaxfrcs[i]))+0.1/(1.001-listacts[i])-0.1/(0.001+listacts[i]); //including the activations this way is not a very good soln, but it's a hack to make the optimization keep the activations below 1
			//W(i,i) = 1 + 1000/(listmaxfrcs[i])+0.1/(1-listacts[i])-0.1/(0.00+listacts[i]); 
			if((listacts[i]  < 0.001) || (listacts[i] > 0.999))
				std::cout<<"activation "<<i<<" crossed limit : "<<listacts[i]<<std::endl;
		}
		//W = 1;
		//std::cout<<"W :"<<W<<std::endl;
		
		Matrix WTW(numMuscs,numMuscs);
		Matrix WTWinv(numMuscs,numMuscs);
		Matrix AWTWinvAT(numdims,numdims);
		Matrix AWTWinvATinv(numdims,numdims);
		Matrix x(numMuscs,1);
		double act;
		bool ok_to_proceed;

		for(int itercount = 1;itercount < 10;itercount++)
		{
			ok_to_proceed = true;
			WTW = (~W)*W;
			//std::cout<<WTW;			
			FactorLU WTWLU(WTW);
			WTWLU.inverse(WTWinv);
			//std::cout<<WTWinv;			
			AWTWinvAT = A*WTWinv*(~A);
			//std::cout<"AWTWAT"<<AWTWAT;
			FactorLU AWTWinvATLU(AWTWinvAT);
			AWTWinvATLU.inverse(AWTWinvATinv);
			//std::cout<<"AWTWATinv"<<AWTWATinv;
			x = WTWinv*(~A)*AWTWinvATinv*b;
			//std::cout<<"x :"<<x<<std::endl;
			
			for(int i = 0; i < numMuscs; i++)
			{
				if(listmaxfrcs[i] < 0.1)
					listmaxfrcs[i] = abs(listmaxfrcs[i]) + 0.1;
				act = x(i,0)/listmaxfrcs[i];
				if(act < -0.01)
				{
					ok_to_proceed = false;
					W(i,i) = 10000;
				}
				else if(act > 1)
				{
					ok_to_proceed = false;
					W(i,i) = 1.2*W(i,i)*act;
				}
			}

			if(ok_to_proceed)
			{
				//std::cout<<"Optimization done, iterations: "<<itercount<<std::endl;
				break;
			}
			
		}
		if(!ok_to_proceed)
		{
			//std::cout<<"Optimization not perfect, approximating solution"<<std::endl;
		}
		//std::cout<<"x :"<<x<<std::endl;
		//std::cout<<"Ax :"<<A*x<<std::endl;
	
		//debug code
		if(!(x(0,0) <= 0)&&!(x(0,0) >= 0))
		{
			std::cout<<"Problem - infinite x :"<<std::endl;
			std::cout<<"x :"<<x<<std::endl;
			std::cout<<"A :"<<A<<std::endl;
			std::cout<<"W :"<<W<<std::endl;
			std::cout<<"b :"<<b<<std::endl;
			for(int i = 0; i < numMuscs; i++)
			{
				std::cout<<"activation "<<i<<" : "<<listacts[i]<<std::endl;
				std::cout<<"maxfrc "<<i<<" : "<<listmaxfrcs[i]<<std::endl;
				std::cout<<"length multiplier"<<listMusc[i]->getActiveForceLengthMultiplier(s)<<std::endl;
				std::cout<<"velocity multiplier"<<listMusc[i]->getForceVelocityMultiplier(s)<<std::endl;
				std::cout<<"Wcalc "<<1 + 1000/(0.1+abs(listmaxfrcs[i]))+0.1/(1.001-listacts[i])-0.1/(0.001+listacts[i])<<std::endl;
			}
		}

		for(int i = 0; i < numMuscs; i++)
		{
			act = x(i,0)/listmaxfrcs[i];
		//std::cout<<"present act :"<<listacts[i]<<std::endl;
			if(act > 0.999)
				act = 0.999;
			else if(act < 0.001)
				act = 0.001;
		//std::cout<<"new act :"<<act<<std::endl;	

			//Apply controls
			// Thelen muscle has only one control
			Vector muscleControl(1,act);// x[i]
			// Add in the controls computed for this muscle to the set of all model controls
			listMusc[i]->addInControls(muscleControl, controls);
		}
		//_model->updVisualizer().getSimbodyVisualizer().report(s);
	}

// This section contains the member variables of this controller class.
private:

	/** Gains on position and velocity error */
	double kp;
	double kv;
	Model* _modelCopy;

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
		double finalTime = 10.0;

		// Create the controller. Pass Kp, Kv values, model
		IAAController *controller = new IAAController(500,10,&osimModel);

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
		    viz.setBackgroundType(viz.GroundAndSky);
		    viz.setGroundHeight(0.0);
		    viz.setShowShadows(true);
		    viz.setShowFrameRate(false);
		    viz.setShowSimTime(true);
        }

		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator integrator( osimModel.getMultibodySystem() );
		integrator.setAccuracy( 1.0e-3 );

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

		//manager.setUseSpecifiedDT(true);
		//double aDT[1000];
		//double step = (finalTime - initialTime)/1000;
		//for (int i = 0; i < 1000; i++)
		//{
		//	aDT[i] = step;
		//}
		//manager.setDTArray(1000,aDT);

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
