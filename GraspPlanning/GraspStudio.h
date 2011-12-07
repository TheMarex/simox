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
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _GraspStudio_h_
#define _GraspStudio_h_


/*! \defgroup GraspStudio The Grasp Planning Library
GraspStudio offers algorithms for grasp scoring and planning covering 3D force space calculations and a full
implementation of the 6d wrench space algorithm for grasp quality measurement.
*/

#ifdef WIN32

// needed to have M_PI etc defined
#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

// eigen wants this on windows
#if !defined(NOMINMAX)
#define NOMINMAX
#endif

#endif

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/VirtualRobotException.h"
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef WIN32
#  include <winsock2.h>
#  include <windows.h>
#  pragma warning ( disable : 4251 )
#  if defined(GraspStudio_EXPORTS)
#    define GRASPSTUDIO_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define GRASPSTUDIO_IMPORT_EXPORT __declspec(dllimport)
#  endif
#else
#  define GRASPSTUDIO_IMPORT_EXPORT
#endif


namespace GraspStudio
{
	// only valid within the GraspStudio namespace
	using std::cout;
	using std::endl;

	class GraspQualityMeasure;
	class GraspQualityMeasureWrenchSpace;
	class ContactConeGenerator;

	class GraspQualityMeasure;
	class ApproachMovementGenerator;
	class ApproachMovementSurfaceNormal;
	class GraspPlanner;
	class GenericGraspPlanner;

	typedef boost::shared_ptr<GraspQualityMeasure> GraspQualityMeasurePtr;
	typedef boost::shared_ptr<GraspQualityMeasureWrenchSpace> GraspQualityMeasureWrenchSpacePtr;
	typedef boost::shared_ptr<ContactConeGenerator> ContactConeGeneratorPtr;
	typedef boost::shared_ptr<GraspQualityMeasure> GraspQualityMeasurePtr;
	typedef boost::shared_ptr<ApproachMovementGenerator> ApproachMovementGeneratorPtr;
	typedef boost::shared_ptr<ApproachMovementSurfaceNormal> ApproachMovementSurfaceNormalPtr;
	typedef boost::shared_ptr<GraspPlanner> GraspPlannerPtr;
	typedef boost::shared_ptr<GenericGraspPlanner> GenericGraspPlannerPtr;

#define GRASPSTUDIO_INFO VR_INFO
#define GRASPSTUDIO_WARNING VR_WARNING
#define GRASPSTUDIO_ERROR VR_ERROR

#define THROW_GRASPSTUDIO_EXCEPTION(a) THROW_VR_EXCEPTION(a)

#ifdef _DEBUG
#define GRASPSTUDIO_ASSERT(a) if (!(a)) {cout << "ASSERT failed (" << #a <<")"<<endl; THROW_GRASPSTUDIO_EXCEPTION( "ASSERT failed (" << #a << ")" )};
#define GRASPSTUDIO_ASSERT_MESSAGE(a,b) if (!(a)) {cout << "ASSERT failed (" << #a <<"): "<<b<<endl; THROW_GRASPSTUDIO_EXCEPTION( "ASSERT failed (" << #a << "): " << b )};

#else
#define GRASPSTUDIO_ASSERT(a)
#define GRASPSTUDIO_ASSERT_MESSAGE(a,b)
#endif

}

#endif // _GraspStudio_h_
