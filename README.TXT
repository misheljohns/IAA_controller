Steps:

1. main():
	loads model, adds controller, adds visualizer, integrates (forward dynamics), saves results

2. class IAAController:
	gets list of actuators
	gets present activation, max force, so current force
	runs IAA on the model, get acceleration/force for each muscle <_computePotentialsOnlyProp>, get IAA matrix [A]
	gets present CoM position, velocity, acceleration, uses a control law to get desired acc as input for the next step [B]
	solves Ax = B to get x		[x] - vector of <additional> muscle forces to be applied; weighted by 1/((1-listacts[i])*listmaxfrcs[i]) <so muscles with a lot more potential to increase the force will be activated while almost saturated ones have minimized forces.
	from x, get a - muscle activations required to get the calculated force
	if a > 1, set a = 1, redo from 11
	
Doubts:
	- Thelen muscle quation with PE, SE and active force terms... how do we take care of the PE and SE terms in the controller... and do we really need to look at the length and velocity multipliers?
	- 