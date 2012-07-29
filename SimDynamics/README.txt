SimDynamics provides an interface to physics engine support with an experimental implementation for Bullet.
Several issues have to be concerned for the current version:
* Simox/VirtualRobot allows to build robot structures that are not optimal when using them with Bullet. E.g. 
    - Virtual joints without 3d models that make sense in the simox framework to handle coordinate systems are not directly supported in Bullet. 
      We tweaked the generation so that btEmptyCollisionShapes are used for such cases but there may be situations where thsi does not work perfectly
    - Multiple revolute (hinge) joints following each other without any 3d models or translations can cause invalid worldTransform computations. (ToDo Allow Generic6Dof joints in Simox, then there we can model them directly in bullet)
    - Center of mass (com) transformations: Bullet computes com poses for all involved models. 
      Currently the VirtualRobot's collision models are used to create bullet objects and here the com transformations are considered. 
      But com transformations for VisualiztaionModels are ignored right now. 
      If they are different to the collision models or if there is no collision model but a visu model, the visualization poses in simox will be invalid.
    - Bullet computes the joint value / angle by determining the rotation of the worldposes of the connected btRigidBodies. There seems to be a bug when the rotation is at +/- PI. Avoid transformations between RobotNodes that include a rotation around 180 degrees.
* Bullet models joints with btHingeConstarints. We found out that these constraints are not hard ones, so there can be situations where the constraints are violated which means that the robot structure looks somehow broken. Usually the contraints solvers of bullet try to re-adjust all limbs according to their constraints but if that is not possible or if the impacting forces/torques are too high the result is not as expected.
    - This effect is part of the solver algorithm, so it cannot be totaly elimentated. But we noticed that multiple succeeding joints without displacement/3d models makes this artefact worse.
    
The best results can be achieved with modeling the robot the following way:
* Always use Visualiztaion and Collision Models (COM issue)
* Model your robot like <RobotNode with jointAxis and 3d model> without fixed joints (e.g. for rotations of the coordinate frame etc)
* Try to model rotated 3d models within you 3d model definition (.iv file) instead of rotate the models with postJointTransformation or DH parameters. This might affect the rotation of your coordinate systems but avoids an error in bullet when the rotation flips from/to +/-PI.
