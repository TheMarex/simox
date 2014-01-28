/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_JacobiProvider_h_
#define _VirtualRobot_JacobiProvider_h_

#include "../VirtualRobotImportExport.h"

#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace VirtualRobot
{


class VIRTUAL_ROBOT_IMPORT_EXPORT JacobiProvider : public boost::enable_shared_from_this<JacobiProvider>
{
public:
	
    /*!
      @brief Several methods are offered for inverting the Jacobi (i.e. building the Pseudoinverse)
    */
    enum InverseJacobiMethod
    {
		eSVD,       //<! PseudoInverse Jacobian. Performing SVD and setting very small eigen values to zero results in a quite stable inverting of the Jacobi. (default)
		eSVDDamped, //<! Using the damped PseudoInverse algorithm
		eTranspose  //<! The Jacobi Transpose method is faster than SVD and works well for redundant kinematic chains.
    };

	/*!

	*/
	JacobiProvider(RobotNodeSetPtr rns, InverseJacobiMethod invJacMethod = eSVD);

	virtual ~JacobiProvider();

	virtual Eigen::MatrixXf getJacobianMatrix() = 0;
	virtual Eigen::MatrixXf getJacobianMatrix(RobotNodePtr tcp) = 0;
	virtual Eigen::MatrixXf computePseudoInverseJacobianMatrix(const Eigen::MatrixXf &m) const;
	virtual Eigen::MatrixXf getPseudoInverseJacobianMatrix();
	virtual Eigen::MatrixXf getPseudoInverseJacobianMatrix(RobotNodePtr tcp);

	VirtualRobot::RobotNodeSetPtr getRobotNodeSet();

	/*
		If set, a weigthed inverse Jacobian is computed. The weighting is only applied in eTranspose mode!
		jointScaling.rows() must be nDoF
		Large entries result in small joint deltas.
	*/
	void setJointWeights(const Eigen::VectorXf &jointWeights);
protected:
    
	RobotNodeSetPtr rns; 
	InverseJacobiMethod inverseMethod;

	Eigen::VectorXf jointWeights; // only used in eTranspose mode
	
};

typedef boost::shared_ptr<JacobiProvider> JacobiProviderPtr;
} // namespace VirtualRobot

#endif // _VirtualRobot_JacobiProvider_h_
