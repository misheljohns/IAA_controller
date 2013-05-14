#ifndef OPENSIM_INDUCED_ACCELERATIONS_SOLVER_H_
#define OPENSIM_INDUCED_ACCELERATIONS_SOLVER_H_
/* -------------------------------------------------------------------------- *
 *                OpenSim:  InducedAccelerationsSolver.h                      *
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
#include <OpenSim/Simulation/Solver.h>
#include "OpenSim/Analyses/osimAnalysesDLL.h"

namespace OpenSim { 

class Model;
class ExternalForce;

//=============================================================================
//=============================================================================
/**
 A Solver to compute the induced accelerations of an applied "force" acting
 on the model. The solver can applyg constraints to replace external (contact)
 forces in order to decompose contributions of external reaction forces and to
 subsequentlt determine the induced (generalized) accelerations (udot).
 The solver can compute the induced acceleration and constraint reaction force
 contributions for any model Force. 
 
 The solver converts the unconstrained system:

(1)  [M]*udot = G(q) + V(q,u) + S(t, q,u) + F(t, q, u, z, x)
		where G = gravitational, V = velocity dependent, S = external
		or contact forces w.r.t. to ground and F are all other applied
		model forces like motors and muscles.
		q generalized coordinates, u mobilities (gen speeds, qdot = [N]*u)
		z other state variables (e.g. muscle fiber lengths), x controls
		t time, q, u, and z constitute the State of the model

 to a constrained system:
(2a) [M]*udot = G(q) + V(q,u) -[~C]*lambda + F(t, q, u, z, x)
(2b) [C]*udot = b(t,q,u) 
		by replacing the external forces or passive contact force S with
		rigid kinematic constraints imposed on the system acceleration
		([C] and b obtained by differentiating position and velocity
		 constraints) and enforced in the system dynamics by Lagrange
		 multipliers, lambda. 
	NOTE: The validity of the constraints can be tested by the accuracy 
		  by which [~C]*lambda can reconstitute S, when all forces are 
		  applied.

	Eqn 2 is linear in both udot (accelerations) and external (or contact)
	 forces and the contribution of applied forces to the model's
	 accelerations (udot_i) and external forces (reaction_i) can be directly
	 evaluated. The contribution of a force (like gravity or a muscle) to the
	 system acceleration is called its "induced acceleration" and the 
	 contribution to the reaction (or external) force is its "induced reaction".
	 
	 NOTE, the sum of induced accelerations and reactions MUST must total to
	 the complete system acceleration or external force when all model forces
	 are applied.  
	 
 The solver can apply any OpenSim::Constraint that implements
 setContactPointForInducedAccelerations() to replace external and/or contact 
 forces that are applied during a forward dynamics simulation.
 
  @author Ajay Seth
 */
class OSIMANALYSES_API InducedAccelerationsSolver : public Solver {
OpenSim_DECLARE_CONCRETE_OBJECT(InducedAccelerationsSolver, Solver);

//=============================================================================
// METHODS
//=============================================================================
public:
//----------------------------------------------------------------------------
// CONSTRUCTION
//----------------------------------------------------------------------------
/** Construct an InducedAccelerations solver applied to the given model */
	InducedAccelerationsSolver(const Model &model);

/** Convenience constructor.
	InducedAccelerationsSolver applied to model with constraints to
	be applied to replace external forces. 
	@param[in]	replacementConstraints	Set of constraints to replace
				external forces (contacts) in the model
	@see setReplacementConstraints() for details*/
	InducedAccelerationsSolver(const Model &model, 
		const ConstraintSet& replacementConstraints);

//----------------------------------------------------------------------------
// CONFIGURE SOLVER
//----------------------------------------------------------------------------
/** Add a constraint that will replace an external or contact force 
	in the model (identified by name). Replacing a force that is not an
	ExternalForce, ElasticFoundationForce or HuntCrossleyForce (in contact
	with ground) will cause an Exception.

	Any OpenSim::Constraint that implements the method
	 setContactPointForInducedAccelerations(SimTK::State state, Vec3 point)
	 (@see OpenSim::Constraint) can be applied as a replacement.

	A threshold is used to determine when the constraint should be engaged.
	If the external force magnitude exceeds the threshold, it is replaced 
	by the constraint to solve for induced accelerations.
	*/
	void replaceForceWithConstraint(const string& forceToReplace
		const Constraint& replacementConstraint,
		double threshold); 

//----------------------------------------------------------------------------
// SOLVE 
//----------------------------------------------------------------------------
	/** Solve for the induced (generalized) accelerations (udot) resulting 
	    from an applied force. An applied force is expressed as any 
		combination of mobility (generalized) forces and/or body forces.
		
		@param[in]	state					current State of the model
		@param[in]	appliedMobilityForces   Vector of applied mobility forces
		@param[in]	appliedBodyForces		Vector of spatial forces applied 
											to the model (1 per body) 
		@param[out]	constraintReactions     (optional) Vector of induced
											reaction forces
	    @return     A const reference to the Vector of resulting model 
					induced generalized accelerations (udot).
	*/
	const SimTK::Vector& solve(const SimTK::State& state,
		const SimTK::Vector& appliedMobilityForces, 
		const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces,
		SimTK::Vector_<SimTK::SpatialVec>& constraintReactions=0);

	/** Solve for the induced (generalized) accelerations (udot) resulting
		from any model force identified by name
		@param[in]	state		current State of the model
		@param[in]	forceName   name of model Force contributor
								"gravity" and "velocity" are special names
								for gravitational force and velocity
								dependent forces (Coriolis and gyroscopic).
								Otherwise use Force component name.

		@param[out]	constraintReactions     (optional) Vector of induced
											reaction forces
		@return     A const reference to the Vector of the induced 
					generalized accelerations (udot) from the specified force.
	*/
	const SimTK::Vector& solve(const SimTK::State& s,
				const string& forceName,
				SimTK::Vector_<SimTK::SpatialVec>& constraintReactions=0);

private:
	ConstraintSet _replacementConstraints; 


//=============================================================================
}; // END of class InducedAccelerationsSolver
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef OPENSIM_INDUCED_ACCELERATIONS_SOLVER_H_