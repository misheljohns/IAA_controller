Steps:

1. main():
	loads model, adds controller, adds visualizer, integrates (forward dynamics), saves results

2. class IAAController:
	gets list of actuators
	gets present activation, max force, so current force
	runs IAA on the model, get acceleration/force for each muscle <_computePotentialsOnlyProp>, get IAA matrix [A]
	gets present CoM position, velocity, acceleration, uses a control law to get desired acc as input for the next step [B]
		B = kp*(Xdes - Xmeas) + kv(0 - Vmeas)
	solves Ax = B to get x		[x] - vector of <additional> muscle forces to be applied; weighted by 1/((1-listacts[i])*listmaxfrcs[i]) <so muscles with a lot more potential to increase the force will be activated while almost saturated ones have minimized forces.
		General norm minimization with equality constraints - Min ||Wx||; subject to Ax = B
		[x = inv(W'*W)*A'*inv(A*inv(W'*W)*A')*b]

Doubts:
	- Thelen muscle quation with PE, SE and active force terms... how do we take care of the PE and SE terms in the controller... and do we really need to look at the length and velocity multipliers?
	- It seems IAA can give induced accelerations of any coordinate, not just the center of mass. Since the coordinate position is easier to calculate and access in each timestep, would it make more sense to work with coordinates of the pevis or the thorax rather than the CoM position?
	
Further ideas:
	- perhaps we can pass the initial state of the system (si) to the controller before adding it, and also pass to it what coordinates we want to bring back to the initial states after perturbation.
	- if the weighting system does not work to stop activations > 1, we can try iterations: from x, get a - muscle activations required to get the calculated force; if a > 1, set a = 1, redo minimization with other muscles only
	- Is there a matrix library in SimTK, instead of using armadillo?
	- adding contact points to 2392/fixing the feet to the ground
	- How do we render videos? - not possible
	
	
- call SimTK::Vector& solve(const SimTK::State& s,
				const string& forceName,
				SimTK::Vector_<SimTK::SpatialVec>& constraintReactions=0); this vector is a set of Udots in generalized coordinates	
- for now, in indaccsolver, use realizeaccelerations to return a dummy vector of the right size to play with (realizeaccelerations does not return anything, instead weshouldget the accelerations from the model after running this.. we can get a vector of zeros by calling _model->getCoordinateSet().get(i).getAccelerationValue(s))
- use this vector to updUdot; &updUdot = <vector>
- get com acceleration with vec = _model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_analysis); (not working without realize up to acceleration stage 2, but we can't seem to get there even with the realizeaccleration stage command)
- stack each vector together to get the matrix we need
	
	
	
	
- Array to Matrix
- Matrix mult, inverse, transpose
- calc com velocity, position
- how to fix the feet - rolling

aj trunk list



TODO:
- try pelvis..?, com seems to work
- equilibriate problems with some muscles...
- use simple forward intergrator?

todo:
- get activations, max force and calc weight
- calc new activations <iterate?>




