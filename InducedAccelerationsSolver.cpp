/* -------------------------------------------------------------------------- *
 *               OpenSim:  InducedAccelerationsSolver.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/RollingOnSurfaceConstraint.h>
#include "InducedAccelerationsSolver.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define CENTER_OF_MASS_NAME string("center_of_mass")

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Destructor.
 */
InducedAccelerationsSolver::~InducedAccelerationsSolver()
{
	delete &_coordSet;
	delete &_bodySet;
	delete _storeConstraintReactions;
}
//_____________________________________________________________________________
/*
 * Construct an InducedAccelerationsSolver instance.
 *
 * @param aModel Model for which the analysis is to be run.
 */
InducedAccelerationsSolver::InducedAccelerationsSolver(const Model& model) : Solver(model)
{
	setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void InducedAccelerationsSolver::setNull()
{
	setAuthors("Ajay Seth");
	setupProperties();

	_forceThreshold = 6.00;
	_coordNames.setSize(0);
	_bodyNames.setSize(1);
	_bodyNames[0] = CENTER_OF_MASS_NAME;
	_computePotentialsOnly = false;
	_reportConstraintReactions = false;
	// Analysis does not own contents of these sets
	_coordSet.setMemoryOwner(false);
	_bodySet.setMemoryOwner(false);

	_storeConstraintReactions = NULL;
}

//_____________________________________________________________________________
/**
 * Assemble the list of contributors for induced acceleration analysis
 */
void InducedAccelerationsSolver:: assembleContributors()
{
	Array<string> contribs;
	if (!_computePotentialsOnly)
		contribs.append("total");

	const Set<Actuator> &actuatorSet = _model->getActuators();

	//Do the analysis on the bodies that are in the indices list
	for(int i=0; i< actuatorSet.getSize(); i++) {
		contribs.append(actuatorSet.get(i).getName()) ;
	}
 
	contribs.append("gravity");
	contribs.append("velocity");

	_contributors = contribs;
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 * This method, for the purpose of example, records the position and
 * orientation of each body in the model.  You will need to customize it
 * to perform your analysis.
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states: includes generalized coords and speeds
 */
int InducedAccelerationsSolver::record(const SimTK::State& s)
{
	int nu = _model->getNumSpeeds();
	double aT = s.getTime();
	cout << "time = " << aT << endl;

	SimTK::Vector Q = s.getQ();

	// Reset Accelerations for coordinates at this time step
	for(int i=0;i<_coordSet.getSize();i++) {
		_coordIndAccs[i]->setSize(0);
	}

	// Reset Accelerations for bodies at this time step
	for(int i=0;i<_bodySet.getSize();i++) {
		_bodyIndAccs[i]->setSize(0);
	}

	// Reset Accelerations for system center of mass at this time step
	_comIndAccs.setSize(0);
	_constraintReactions.setSize(0);

    SimTK::State s_analysis = _model->getWorkingState();

	_model->initStateWithoutRecreatingSystem(s_analysis);
	// Just need to set current time and position to determine state of constraints
	s_analysis.setTime(aT);
	s_analysis.setQ(Q);

	// Check the external forces and determine if contact constraints should be applied at this time
	// and turn constraint on if it should be.
	Array<bool> constraintOn = applyContactConstraintAccordingToExternalForces(s_analysis);

	// Hang on to a state that has the right flags for contact constraints turned on/off
	_model->setPropertiesFromState(s_analysis);
	// Use this state for the remainder of this step (record)
	s_analysis = _model->getMultibodySystem().realizeTopology();
	// DO NOT recreate the system, will lose location of constraint
	_model->initStateWithoutRecreatingSystem(s_analysis);

	// Cycle through the force contributors to the system acceleration
	for(int c=0; c< _contributors.getSize(); c++){			
		//cout << "Solving for contributor: " << _contributors[c] << endl;
		// Need to be at the dynamics stage to disable a force
		_model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Dynamics);
		
		if(_contributors[c] == "total"){
			// Set gravity ON
			_model->getGravityForce().enable(s_analysis);

			//Use same conditions on constraints
			s_analysis.setTime(aT);
			// Set the configuration (gen. coords and speeds) of the model.
			s_analysis.setQ(Q);
			s_analysis.setU(s.getU());
			s_analysis.setZ(s.getZ());

			//Make sure all the actuators are on!
			for(int f=0; f<_model->getActuators().getSize(); f++){
				_model->updActuators().get(f).setDisabled(s_analysis, false);
			}

			// Get to  the point where we can evaluate unilateral constraint conditions
			 _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Acceleration);

		    /* *********************************** ERROR CHECKING *******************************
			SimTK::Vec3 pcom =_model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterLocationInGround(s_analysis);
			SimTK::Vec3 vcom =_model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterVelocityInGround(s_analysis);
			SimTK::Vec3 acom =_model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_analysis);

			SimTK::Matrix M;
			_model->getMultibodySystem().getMatterSubsystem().calcM(s_analysis, M);
			cout << "mass matrix: " << M << endl;

			SimTK::Inertia sysInertia = _model->getMultibodySystem().getMatterSubsystem().calcSystemCentralInertiaInGround(s_analysis);
			cout << "system inertia: " << sysInertia << endl;

			SimTK::SpatialVec sysMomentum =_model->getMultibodySystem().getMatterSubsystem().calcSystemMomentumAboutGroundOrigin(s_analysis);
			cout << "system momentum: " << sysMomentum << endl;

			const SimTK::Vector &appliedMobilityForces = _model->getMultibodySystem().getMobilityForces(s_analysis, SimTK::Stage::Dynamics);
			appliedMobilityForces.dump("All Applied Mobility Forces");
		
			// Get all applied body forces like those from conact
			const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces = _model->getMultibodySystem().getRigidBodyForces(s_analysis, SimTK::Stage::Dynamics);
			appliedBodyForces.dump("All Applied Body Forces");

			SimTK::Vector ucUdot;
			SimTK::Vector_<SimTK::SpatialVec> ucA_GB;
			_model->getMultibodySystem().getMatterSubsystem().calcAccelerationIgnoringConstraints(s_analysis, appliedMobilityForces, appliedBodyForces, ucUdot, ucA_GB) ;
			ucUdot.dump("Udots Ignoring Constraints");
			ucA_GB.dump("Body Accelerations");

			SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(_constraintSet.getSize(), SimTK::SpatialVec(SimTK::Vec3(0)));
			SimTK::Vector constraintMobilityForces(0);

			int nc = _model->getMultibodySystem().getMatterSubsystem().getNumConstraints();
			for (SimTK::ConstraintIndex cx(0); cx < nc; ++cx) {
				if (!_model->getMultibodySystem().getMatterSubsystem().isConstraintDisabled(s_analysis, cx)){
					cout << "Constraint " << cx << " enabled!" << endl;
				}
			}
			//int nMults = _model->getMultibodySystem().getMatterSubsystem().getTotalMultAlloc();

			for(int i=0; i<constraintOn.getSize(); i++) {
				if(constraintOn[i])
					_constraintSet[i].calcConstraintForces(s_analysis, constraintBodyForces, constraintMobilityForces);
			}
			constraintBodyForces.dump("Constraint Body Forces");
			constraintMobilityForces.dump("Constraint Mobility Forces");
			// ******************************* end ERROR CHECKING *******************************/
	
			for(int i=0; i<constraintOn.getSize(); i++) {
				_constraintSet.get(i).setDisabled(s_analysis, !constraintOn[i]);
				// Make sure we stay at Dynamics so each constraint can evaluate its conditions
				_model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Acceleration);
			}

			// This should also push changes to defaults for unilateral conditions
			_model->setPropertiesFromState(s_analysis);

		}
		else if(_contributors[c] == "gravity"){
			// Set gravity ON
			_model->updForceSubsystem().setForceIsDisabled(s_analysis, _model->getGravityForce().getForceIndex(), false);

			//s_analysis = _model->initSystem();
			s_analysis.setTime(aT);
			s_analysis.setQ(Q);

			// zero velocity
			s_analysis.setU(SimTK::Vector(nu,0.0));
			s_analysis.setZ(s.getZ());

			// disable actuator forces
			for(int f=0; f<_model->getActuators().getSize(); f++){
				_model->updActuators().get(f).setDisabled(s_analysis, true);
			}
		}
		else if(_contributors[c] == "velocity"){		
			// Set gravity off
			_model->updForceSubsystem().setForceIsDisabled(s_analysis, _model->getGravityForce().getForceIndex(), true);

			s_analysis.setTime(aT);
			s_analysis.setQ(Q);

			// non-zero velocity
			s_analysis.setU(s.getU());
			s_analysis.setZ(s.getZ());
			
			// zero actuator forces
			for(int f=0; f<_model->getActuators().getSize(); f++){
				_model->updActuators().get(f).setDisabled(s_analysis, true);
			}
			// Set the configuration (gen. coords and speeds) of the model.
			_model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Velocity);
		}
		else{ //The rest are actuators		
			// Set gravity ON
			_model->updForceSubsystem().setForceIsDisabled(s_analysis, _model->getGravityForce().getForceIndex(), true);

			// zero actuator forces
			for(int f=0; f<_model->getActuators().getSize(); f++){
				_model->updActuators().get(f).setDisabled(s_analysis, true);
			}

			//s_analysis = _model->initSystem();
			s_analysis.setTime(aT);
			s_analysis.setQ(Q);

			// zero velocity
			SimTK::Vector U(nu,0.0);
			s_analysis.setU(U);
			s_analysis.setZ(s.getZ());
			// light up the one actuator who's contribution we are looking for
			int ai = _model->getActuators().getIndex(_contributors[c]);
			if(ai<0)
				throw Exception("InducedAcceleration: ERR- Could not find actuator '"+_contributors[c],__FILE__,__LINE__);
			
			Actuator &actuator = _model->getActuators().get(ai);
			actuator.setDisabled(s_analysis, false);
			actuator.overrideForce(s_analysis, false);
			Muscle *muscle = dynamic_cast<Muscle *>(&actuator);
			if(muscle){
				if(_computePotentialsOnly){
					muscle->overrideForce(s_analysis, true);
					muscle->setOverrideForce(s_analysis, 1.0);
				}
			}

			// Set the configuration (gen. coords and speeds) of the model.
			_model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Model);
			_model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Velocity);

		}// End of if to select contributor 

		// cout << "Constraint 0 is of "<< _constraintSet[0].getConcreteClassName() << " and should be " << constraintOn[0] << " and is actually " <<  (_constraintSet[0].isDisabled(s_analysis) ? "off" : "on") << endl;
		// cout << "Constraint 1 is of "<< _constraintSet[1].getConcreteClassName() << " and should be " << constraintOn[1] << " and is actually " <<  (_constraintSet[1].isDisabled(s_analysis) ? "off" : "on") << endl;

		// After setting the state of the model and applying forces
		// Compute the derivative of the multibody system (speeds and accelerations)
		_model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Acceleration);

		// Sanity check that constraints hasn't totally changed the configuration of the model
		double error = (Q-s_analysis.getQ()).norm();

		// Report reaction forces for debugging
		/*
		SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(_constraintSet.getSize());
		SimTK::Vector mobilityForces(0);

		for(int i=0; i<constraintOn.getSize(); i++) {
			if(constraintOn[i])
				_constraintSet.get(i).calcConstraintForces(s_analysis, constraintBodyForces, mobilityForces);
		}*/

		// VARIABLES
		SimTK::Vec3 vec,angVec;

		// Get Accelerations for kinematics of bodies
		for(int i=0;i<_coordSet.getSize();i++) {
			double acc = _coordSet.get(i).getAccelerationValue(s_analysis);

			if(getInDegrees()) 
				acc *= SimTK_RADIAN_TO_DEGREE;	
			_coordIndAccs[i]->append(1, &acc);
		}

		// cout << "Input Body Names: "<< _bodyNames << endl;

		// Get Accelerations for kinematics of bodies
		for(int i=0;i<_bodySet.getSize();i++) {
			Body &body = _bodySet.get(i);
			// cout << "Body Name: "<< body->getName() << endl;
			SimTK::Vec3 com(0);
			// Get the center of mass location for this body
			body.getMassCenter(com);
			
			// Get the body acceleration
			_model->getSimbodyEngine().getAcceleration(s_analysis, body, com, vec);
			_model->getSimbodyEngine().getAngularAcceleration(s_analysis, body, angVec);	

			// CONVERT TO DEGREES?
			if(getInDegrees()) 
				angVec *= SimTK_RADIAN_TO_DEGREE;	

			// FILL KINEMATICS ARRAY
			_bodyIndAccs[i]->append(3, &vec[0]);
			_bodyIndAccs[i]->append(3, &angVec[0]);
		}

		// Get Accelerations for kinematics of COM
		if(_includeCOM){
			// Get the body acceleration in ground
			vec = _model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_analysis);

			// FILL KINEMATICS ARRAY
			_comIndAccs.append(3, &vec[0]);
		}

		// Get induced constraint reactions for contributor
		if(_reportConstraintReactions){
			for(int j=0; j<_constraintSet.getSize(); j++){
				_constraintReactions.append(_constraintSet[j].getRecordValues(s_analysis));
			}
		}

	} // End cycling through contributors at this time step

	// Set the accelerations of coordinates into their storages
	int nc = _coordSet.getSize();
	for(int i=0; i<nc; i++) {
		_storeInducedAccelerations[i]->append(aT, _coordIndAccs[i]->getSize(),&(_coordIndAccs[i]->get(0)));
	}

	// Set the accelerations of bodies into their storages
	int nb = _bodySet.getSize();
	for(int i=0; i<nb; i++) {
		_storeInducedAccelerations[nc+i]->append(aT, _bodyIndAccs[i]->getSize(),&(_bodyIndAccs[i]->get(0)));
	}

	// Set the accelerations of system center of mass into a storage
	if(_includeCOM){
		_storeInducedAccelerations[nc+nb]->append(aT, _comIndAccs.getSize(), &_comIndAccs[0]);
	}
	if(_reportConstraintReactions){
		_storeConstraintReactions->append(aT, _constraintReactions.getSize(), &_constraintReactions[0]);
	}

	return(0);
}

/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * @param s SimTK:State
 */
void InducedAccelerationsSolver::initialize(const SimTK::State& s)
{	
	// Go forward with a copy of the model so Analysis can add to model if necessary
	_model = _model->clone();

	SimTK::State s_copy = s;
	double time = s_copy.getTime();

	_externalForces.setSize(0);

	//Only add constraint set if constraint type for the analysis is Roll
	for(int i=0; i<_constraintSet.getSize(); i++){
		Constraint* contactConstraint = &_constraintSet.get(i);
		if(contactConstraint)
			_model->updConstraintSet().adoptAndAppend(contactConstraint);
	}

	// Create a set of constraints used to model contact with the ground
	// based on external forces (ExternalForces) applied to the model
	for(int i=0; i < _model->getForceSet().getSize(); i++){
		ExternalForce *exf = dynamic_cast<ExternalForce *>(&_model->getForceSet().get(i));
		if(exf){
			addContactConstraintFromExternalForce(exf);
			exf->setDisabled(s_copy, true);
		}
	}

	// Get value for gravity
	_gravity = _model->getGravity();

	SimTK::State &s_analysis =_model->initSystem();

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	setupStorage();
}

//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param s SimTK:State
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerationsSolver::begin(SimTK::State &s)
{
	if(!proceed()) return(0);

	initialize(s);

	// RESET STORAGES
	for(int i = 0; i<_storeInducedAccelerations.getSize(); i++){
		_storeInducedAccelerations[i]->reset(s.getTime());
	}

	cout << "Performing Induced Accelerations Analysis" << endl;

	// RECORD
	int status = 0;
	status = record(s);

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param s, state
 * @param stepNumber
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerationsSolver::step(const SimTK::State &s, int stepNumber)
{
	if(proceed(stepNumber) && getOn())
		record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param State
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerationsSolver::end(SimTK::State &s)
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}




//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int InducedAccelerationsSolver::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// Write out induced accelerations for all kinematic variables
	for(int i = 0; i < _storeInducedAccelerations.getSize(); i++){
		Storage::printResult(_storeInducedAccelerations[i],aBaseName+"_"
			                   +getName()+"_"+_storeInducedAccelerations[i]->getName(),aDir,aDT,aExtension);
	}

	if(_reportConstraintReactions){
		Storage::printResult(_storeConstraintReactions, aBaseName+"_"
			                   +getName()+"_"+_storeConstraintReactions->getName(),aDir,aDT,aExtension);
	}
	return(0);
}


void InducedAccelerationsSolver::addContactConstraintFromExternalForce(ExternalForce *externalForce)
{
	_externalForces.append(externalForce);
}

Array<bool> InducedAccelerationsSolver::applyContactConstraintAccordingToExternalForces(SimTK::State &s)
{
	Array<bool> constraintOn(false, _constraintSet.getSize());
	double t = s.getTime();

	for(int i=0; i<_externalForces.getSize(); i++){
		ExternalForce *exf = _externalForces[i];
		SimTK::Vec3 point, force, gpoint;

		force = exf->getForceAtTime(t);
		
		// If the applied force is "significant" replace it with a constraint
		if (force.norm() > _forceThreshold){
			// get the point of contact from applied external force
			point = exf->getPointAtTime(t);
			// point should be expressed in the "applied to" body for consistency across all constraints
			if(exf->getPointExpressedInBodyName() != exf->getAppliedToBodyName()){
				int appliedToBodyIndex = _model->getBodySet().getIndex(exf->getAppliedToBodyName());
				if(appliedToBodyIndex < 0){
					cout << "External force appliedToBody " <<  exf->getAppliedToBodyName() << " not found." << endl;
				}

				int expressedInBodyIndex = _model->getBodySet().getIndex(exf->getPointExpressedInBodyName());
				if(expressedInBodyIndex < 0){
					cout << "External force expressedInBody " <<  exf->getPointExpressedInBodyName() << " not found." << endl;
				}

				const Body &appliedToBody = _model->getBodySet().get(appliedToBodyIndex);
				const Body &expressedInBody = _model->getBodySet().get(expressedInBodyIndex);

				_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
				_model->getSimbodyEngine().transformPosition(s, expressedInBody, point, appliedToBody, point);
			}

			_constraintSet.get(i).setContactPointForInducedAccelerations(s, point);

			// turn on the constraint
			_constraintSet.get(i).setDisabled(s, false);
			// return the state of the constraint
			constraintOn[i] = true;

		}
		else{
			// turn off the constraint
			_constraintSet.get(i).setDisabled(s, true);
			// return the state of the constraint
			constraintOn[i] = false;
		}
	}

	return constraintOn;
}
