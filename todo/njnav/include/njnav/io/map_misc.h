/** \file
	\brief Some defines for handle the map
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NJ_MAP_MISC_H
#define NJ_MAP_MISC_H


#include <string>
#include <common/types.h>

namespace NJRobot
{

	/** \brief Markers in map*/
	enum MarkerType {	
		Marker_None = 0,				// nothing
		Marker_Select,					// select
		Marker_Eraser,					// eraser
		Marker_Point,					// point
		Marker_Line,					// line
		Marker_Goal,					// goal
		Marker_Dock,					// dock
		Marker_HomePoint,				// home point
		Marker_HomeArea,				// home area
		Marker_ForbiddenLine,			// forbidden line
		Marker_ForbiddenArea,			// forbidden area
		Marker_SimBoxObstacle,			// simulation obstacle
		Marker_PathPoint,                // path point
		Marker_SpecalArea,
		Marker_LandMark,
		Marker_Route                      //Route
	};

	/** \brief Editor in map */
	enum EditorMode {
		Editor_None = 0,				// nothing
		Editor_Erase,					// erase
		Editor_Edit,					// edit
		Editor_Addition,				// addition
	};

	/** \brief Cairns in map */
	enum CairnType {
		None = 0,
		ForbiddenArea,
		ForbiddenLine,
		RobotHome,
		Dock,
		Goal,
		GoalWithHeading,
		SimBoxObstacle,
		PathPoint,
		SpecalArea,
		LandMark,
		Route
	};

	/** \brief Objects in map */
	struct MapObject{
		MapObject():id(0),dir(0),function(0),isRight(0),isDown(0),taskMode(0),toPointDist(0),toPointAngle(0),isAvoidance(0),SecurityDist(0),MaxSpeed(0){}
		int id;
		MarkerType mark;
		CairnType type;
		Point center;
		double dir;
		std::string tagname;
		Line line;
		std::string description;
		int function;
		bool isRight;
		bool isDown;
		//Route part
		std::string startName,endName;
		int taskMode;
		double toPointDist;
		double toPointAngle;
		bool isAvoidance;
		double SecurityDist;
		double MaxSpeed;
	};
}

#endif //NJ_MAP_MISC_H