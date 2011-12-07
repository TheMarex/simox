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
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_RrtWorkspaceVisualization_h_
#define _Saba_RrtWorkspaceVisualization_h_

#include "../Saba.h"
#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/Robot.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Saba
{

/*!
 *
 * A visualization of an RRT search tree.
 * @see CoinRrtWorkspaceVisualization
 *
 */
class SABA_IMPORT_EXPORT RrtWorkspaceVisualization
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Constructor
		Robot must have a node with name TCPName. 
		The visualizations are build by determining the TCP's position in workspace according to the configurations of a path or tree .
	*/
	RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, CSpacePtr cspace, const std::string &TCPName);
	RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr robotNodeSet, const std::string &TCPName);

	/*!
	*/
	virtual ~RrtWorkspaceVisualization();

	/*!
		Add visualization of a path in cspace.
	*/
	//virtual bool addCSpacePath(CSpacePathPtr path);

	/*!
		Add visualization of a path in cspace.
	*/
	//virtual bool addTree(CSpaceTreePtr tree);

	/*!
		Add visualization of a path in cspace.
	*/
	//virtual bool addConfig(const Eigen::VectorXf &c);

	/*!
		Clears all visualizations.
	*/
	virtual void reset();

	/*!
		Set name of TCP joint. Does not affect already added paths or trees.
	*/
	void setTCPName(const std::string TCPName);

protected:
	VirtualRobot::RobotPtr robot;
	CSpacePtr cspace;
	VirtualRobot::RobotNodeSetPtr robotNodeSet;
	VirtualRobot::RobotNodePtr TCPNode;

	std::string TCPName;

};

} // namespace Saba

#endif // _Saba_RrtWorkspaceVisualization_h_
