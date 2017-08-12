/** \file
	\brief Path simplify
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#pragma once
#include <common/types.h>
#include <map/traval_map.h>
#include <pathplan/abstract_path_optimize.h>
namespace NJRobot
{

class PathSimplify:public AbstractPathOptimize
{
public:
	PathSimplify(void);
	~PathSimplify(void);

	void optimize(RobotPath& path);

private:

};


}
