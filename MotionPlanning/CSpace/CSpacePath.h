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
#ifndef _saba_CSpacePath_h
#define _saba_CSpacePath_h

#include "../Saba.h"
#include <vector>
#include <string>

#include<Eigen/StdVector>
 
#include "CSpace.h"

namespace Saba {

/*!
 *
 * \brief A path in c-space.
 *
 */
class SABA_IMPORT_EXPORT CSpacePath
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Constructor. Dimension of this path is retrieved from cspace.
	  \param cspace Is used to retrieve the dimension of the corresponding c-space and the corresponding joints
	*/
	CSpacePath(CSpacePtr cspace);

	//! Destructor
	virtual ~CSpacePath();

	/*!
	  Insert a configuration as one point to the solution path.
	  \param c configuration vector
	*/
	void addPathPoint(const Eigen::VectorXf &c);

	/*!
	  Get a point in solution path with a index.
	  \param nr index to point in path
	*/
	Eigen::VectorXf getPathEntry(unsigned int nr) const;

	//! return total number of path points
	unsigned int getNrOfPathPoints() const;

	//! to retrieve entries of path
	bool getPathEntries(unsigned int start, unsigned int end , std::vector<Eigen::VectorXf > &storePosList) const;

	/*!
	  Creates a copy of the path instance.
	  \return pointer to new instance (copy)
	*/
	CSpacePathPtr clone() const;

	CSpacePathPtr createSubPath(unsigned int startIndex, unsigned int endIndex) const;

	//! reset all data
	virtual void reset();

	//! reverse position order: end becomes start
	virtual void reverse();

	/*!
		Return euclidean c-space length of complete path
		\param forceDisablingMetricWeights When disabled, the metricWeights that might have been specified in the corresponding c-space
		are not used for computing the path length.
	*/
	float getPathLength(bool forceDisablingMetricWeights = false) const;

	/*!
		Return length of part of the path (in CSpace!)
		\param startIndex The start point
		\param endIndex The end point
		\param useMetricWeights When set, the metricWeights that have been specified in the corresponding c-space are used for computing the path length.
	*/
	float getPathLength(unsigned int startIndex, unsigned int endIndex, bool useMetricWeights = false) const;

	/*!
	  Erase a point in path.
	  \param pos position of point in path array
	*/
	virtual void erasePosition(unsigned int pos);

	/*!
	   Erases all points from start to end in path (including start&end).
	  \param startPos start position of point in path array
	  \param endPos end position of point in path array
	*/
	virtual unsigned int removePositions(unsigned int startPos, unsigned int endPos);

	/*!
	  Insert a point into path.
	  \param pos position of point being inserted
	  \param c configuration / valid joint values to insert as a point in solution path
	*/
	virtual void insertPosition(unsigned int pos, const Eigen::VectorXf &c);
	virtual void insertPosition(unsigned int pos, std::vector<Eigen::VectorXf > &newConfigurations);
	virtual void insertPath(unsigned int pos, CSpacePathPtr pathToInsert);


	/*!
	 return position on path for time t (0<=t<=1)
	 If storeIndex!=NULL the index of the last path point is stored
	 */
	virtual void interpolatePath(float t, Eigen::VectorXf &storePos, int *storeIndex = NULL) const;

	//! return time t (0<=t<=1) for path entry with number nr
	virtual float getTime(unsigned int nr);

	unsigned int getDimension() const {return dimension;}


	//! prints path contents to console
	virtual void print() const;

	//! For quick access to data.
	const std::vector <Eigen::VectorXf >& getPathData() const;

	/*!
		Creates the corresponding path in workspace.
		\param r The RobotNode that should be considered (e.g. the TCP of the RobotNodeSet)
		\return For each path point of this c-space path, the pose of r in workspace is computed and added to the resulting vector. 
				(The result is a std::vector of Eigen::Matrix4f, but since Eigen alignes memory allocation in a special way, there must be 
				aligned_allocator passed to the std::vector template.)
	*/
	std::vector<Eigen::Matrix4f > createWorkspacePath(VirtualRobot::RobotNodePtr r);


	CSpacePtr getCSpace();
protected:

	std::vector<Eigen::VectorXf > path;		//!< vector with configurations which represent the path
	unsigned int dimension;		//!< dimension of rrt space
	CSpacePtr cspace;
};

} // namespace Saba


#endif // _saba_CSpacePath_h
