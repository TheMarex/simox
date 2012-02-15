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
#ifndef __Saba_ShortcutProcessor_h__
#define __Saba_ShortcutProcessor_h__

#include "../Saba.h"
#include "PathProcessor.h"

namespace Saba
{
/*!
 *
 * \brief The ShortcutProcessor searches shortcuts in C-Space to produce smooth trajectories.
 *
 */
class SABA_IMPORT_EXPORT ShortcutProcessor : public PathProcessor
{
public:

	ShortcutProcessor(CSpacePathPtr path, CSpaceSampledPtr cspace, bool verbose = false);
	virtual ~ShortcutProcessor();

	//! A wrapper to the standard interface. Calls shortenSolutionRandom().
	virtual CSpacePathPtr optimize(int optimizeSteps);

	/*!
		Creates a shortened CSpace path.
		\param shortenLoops Calls tryRandomShort() shortenLoops times
		\param maxSolutionPathDist The max solution path dist.
		\return The local instance of the optimized solution. 
	*/
	CSpacePathPtr shortenSolutionRandom(int shortenLoops = 300, int maxSolutionPathDist = 30);


	/*!
		Goes through path and checks if direct shortcut between node before to node behind current 
		node is collision free.
		Has to be called multiple times in order to get a good result.
		Slow method, shortenSolutionRandom() gets similar results but is much faster.
	*/
	void doPathPruning();

protected:

	// returns number of kicked nodes
	int tryRandomShortcut(int maxSolutionPathDist);
	CSpaceSampledPtr cspace;
};

}// namespace

#endif // __Saba_ShortcutProcessor_h__
