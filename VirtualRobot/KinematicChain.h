#ifndef _VirtualRobot_KinematicChain_h_
#define _VirtualRobot_KinematicChain_h_

#include "VirtualRobotImportExport.h"

#include "Nodes/RobotNode.h"
#include "RobotNodeSet.h"

#include <string>
#include <vector>


namespace VirtualRobot
{
	class Robot;

	class VIRTUAL_ROBOT_IMPORT_EXPORT KinematicChain : public RobotNodeSet
	{
		public:
			friend class RobotFactory;

			/*!
			  Initialize this set with a vector of RobotNodes. 
			  The nodes have to be ordered: node[i] must be parent or grandparent or.. of node[i+1]
			  \param name	The name of this kinematic chain.
			  \param robotNodes A set of robot nodes.
			  \param tcp	This specifies the TCP node of the robot's kinematic tree. 
							tcp does not have to be a node of this set. If not given, the last entry of robotNodes is used.
			  \param kinematicRoot	This specifies the first node of the robot's kinematic tree to be used for updating all members of this set. 
									kinematicRoot does not have to be a node of this set. If not given, the first entry of robotNodes is used.
			  */
			KinematicChain(const std::string& name, const std::vector< RobotNodePtr >& robotNodes, RobotNodePtr tcp = RobotNodePtr(), RobotNodePtr kinematicRoot = RobotNodePtr() );

			/*!
			*/
			virtual ~KinematicChain();
			
			/** @brief Replaces the standard Inverse Kinematics solver.
			 * This way, a specialized IK solver implementing the IKSolver interface
			 * can be associated to a subchain of the robot model. Standard is 
			 * the normal differential IK solver. 
			 * @param ik shared pointer to the solver functor.
			 * /
			void registerIKsolver(boost::shared_ptr<IKsolver> ik);

			/** @brief Replaces the forward calculation algorithm.
			 * @details Sometimes it is necessary to replace a kinematic sub chain
			 * stored in the config file. Examples could be the application of a 
			 * statistically learned model (with implicit knowledge) or the connection 
			 * to a tracking system.
			 * @param fk shared pointer to a kinematics functor. 
			 * /
			void registerKinematicsFunctor(boost::shared_ptr<KinematicsFunctor> fk);
			

			/** @brief Replaces the standard calculus of the Jacobian Matrixm.
			 * @param j shared pointer to a functor. 
			 * /
			void registerJacobianFunctor(boost::share_ptr<JacobianFunctor> j);

			/** @brief Computes the IK for a sub chain of the robot.
			 * If registered, an alternative algorithm is used \sa registerIKsolver.
			 * @param target Beeing an intelligent coordinate the target may be defined in any frame.
			 * @param precision Distance between target and found solution (should be available at the solver).
			 * @throw MissingCoordinatesException Is thrown, for instance, if the IK-algorithm requires a homogeneous matrix and only a position is given.
			 * /
			Matrix solveIK(Coord *target, double &precision) const throw MissingCoordinatesException;

			/** @brief Computes the Jacobian for a subchain.
			 * @return Better Matrix or enhance Coord to understand Jacobians (even for 4x4 information).
			 * /
			Matrix getJacobian(Matrix) const;

			/** @brief Computes the FK for a sub chain of the robot.
			 * If registered, an alternative algorithm is used \sa registerIKsolver.
			 * @param m Vector with the joint angles.
			 * @throw MissingCoordinatesException Is thrown, if not enough angles are specified. 
			 * @return Coord of the endframe. The robot is not altered!
			 * /
			Coord computeFK(Matrix m) const throw MissingCoordinatesException;

			/** @brief Computes the FK for a sub chain of the robot.
			 * If registered, an alternative algorithm is used \sa registerIKsolver.
			 * @param coordinate A Coord instance. Should be set to the requested parameter space.
			 * @throw MissingCoordinatesException Is thrown, if any or the wrong angles are specified. 
			 * @return Coord of the endframe. The robot is not altered!
			 * /
			Coord computeFK(Coord coordinate);
			
			*/

		private:
			/*
			static shared_ptr<IKsolver> _IKsolver;
			static shared_ptr<KinematicsFunctor> _kinematicsFunctor;
			static shared_ptr<JacobianFunctor> _jacobianFunctor;
			*/

		protected:
			RobotNodePtr kinematicRoot;
	};

} // namespace VirtualRobot

#endif // _VirtualRobot_KinematicChain_h_
