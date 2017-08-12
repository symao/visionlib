/************************************************************************/
/* Institute of Cyber-Systems and Control, Nlict-Robot-Fundation		*/
/* HomePage:	http://www.nlict.zju.edu.cn/robot/						*/
/************************************************************************/
/* File:		NRF_MapReader.cpp										*/
/* Purpose: 	Parsing the *.map maps									*/
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// include files
#include <io/map_reader_mapper.h>
#include <common/utils.h>
#include <common/std_out.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cstring>

namespace NJRobot
{

	MapReaderMapper::MapReaderMapper()
	{
		Reset();
	}

	MapReaderMapper::~MapReaderMapper()
	{

	}

	void MapReaderMapper::Initialize()
	{
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	void MapReaderMapper::Reset()
	{
		m_isLoaded = false;
		m_curMapName = "";
	}

	void MapReaderMapper::ReadIn(const std::string & mapName)
	{
		Reset();

		m_curMapName = mapName;
		Parse(mapName);

		return ;
	}

	bool MapReaderMapper::IsMapLoaded(const std::string& mapName)
	{
		if(m_existMap.count(mapName)==0) return false;
		else return m_existMap[mapName];
	}

	bool MapReaderMapper::IsMapLoaded()
	{
		return m_existMap[m_curMapName];
	}

	void MapReaderMapper::Parse(const std::string& mapName)
	{
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		if (m_existMap.count(mapName) > 0) {
			COUT_WARN("MapReaderMapper","Cannot find map file " << mapName);
			return ;
		}
		m_curMapName = mapName;

		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		// open file
		std::fstream infile;
		std::locale::global(std::locale(""));
		infile.open(mapName.c_str(),std::ios::in);
		std::locale::global(std::locale("C"));

		if (! infile.is_open()) {
			COUT_WARN("MapReaderMapper","Failed open map file: " << mapName);
			return ;
		} else {
			//STD_OUT_DEBUG("MapReaderMapper","Succeed in opening file : " << mapName);
		}

		m_elementNum[mapName] = 0;

		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		// parse file
		std::vector <std::string> MapInfolist;
		std::vector <Point> Pointlist;
		std::vector <Point> Pointlist_m;
		std::vector <Line>  Linelist;
		std::vector <MapObject> Objectlist;
		std::vector <std::string> Stringlist;

		int resolution = 20;
		int x_min_point = 0, y_min_point = 0;
		int x_max_point = 0, y_max_point = 0;
		int num_point = 0;
		int x_min_line = 0, y_min_line = 0;
		int x_max_line = 0, y_max_line = 0;
		int num_line = 0;

		//ui
		int							UINumLines = 0;
		int							UINumCurves = 0;
		int							UINumRects = 0;
		int							UINumPolys = 0;
		int							UINumCircles = 0;
		int							UINumEclipse = 0;
		std::vector <std::string>	UiInfolist;

		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		bool hasReachLines = false;
		bool hasReachData = false;
		std::string str = "";
		while (getline(infile,str) && str.length() != 0) 
		{
			std::istringstream istrstr(str.c_str());
			std::istringstream istrstr2(str.c_str());

			if (!hasReachLines  && !hasReachData)
			{
				std::string strtmp;
				istrstr >> strtmp;
				
				if("2D-Map" == strtmp){
					while (strtmp != "")
					{
						strtmp = "";
						istrstr >> strtmp;
						if (strtmp != "")
						{
							Stringlist.push_back(strtmp);
						}else
						{
							break;
						}
					}
				}else if ("MinPos:" == strtmp) {
					istrstr >> x_min_point >> y_min_point;
				} else if ("MaxPos:" == strtmp) {
					istrstr >> x_max_point >> y_max_point;
				} else if ("NumPoints:" == strtmp) {
					istrstr >> num_point;
				} else if ("Resolution:" == strtmp) {
					istrstr >> resolution;
				} else if ("LineMinPos:" == strtmp) {
					istrstr >> x_min_line >> y_min_line;
				} else if ("LineMaxPos:" == strtmp) {
					istrstr >> x_max_line >> y_max_line;
				} else if ("NumLines:" == strtmp) {
					istrstr >> num_line;
				} else if ("UINumLines:" == strtmp) {
					istrstr >> UINumLines;
				} else if ("UINumCurves:" == strtmp) {
					istrstr >> UINumCurves;
				} else if ("UINumRects:" == strtmp) {
					istrstr >> UINumRects;
				} else if ("UINumPolys:" == strtmp) {
					istrstr >> UINumPolys;
				} else if ("UINumCircles:" == strtmp) {
					istrstr >> UINumCircles;
				} else if ("UINumEclipse:" == strtmp) {
					istrstr >> UINumEclipse;
				} else if ("UI:" == strtmp) {
					UiInfolist.push_back(str);
				} else if ("MapInfo:" == strtmp) {
					StoreMapInfo(mapName, MapInfolist, str);
				} else if ("Cairn:" == strtmp) {
					ObjectParse(mapName, Objectlist, str);
				} else if("LINES" == strtmp) {
					hasReachLines = true;
				} else if ("DATA" == strtmp) {
					hasReachData = true;
				} else
					;
			} else {
				if (hasReachLines) {
					std::string strtmp = "";
					istrstr2 >> strtmp;
					if ("DATA" == strtmp) {
						hasReachLines = false;
						hasReachData = true;
					} else {
						int x1Tmp, y1Tmp;
						int x2Tmp, y2Tmp;
						istrstr >> x1Tmp >> y1Tmp >> x2Tmp >> y2Tmp;
						Linelist.push_back(Line(Point(x1Tmp,y1Tmp),Point(x2Tmp,y2Tmp)));
					}
				} else if (hasReachData) {
					int xTmp, yTmp;
					istrstr >> xTmp >> yTmp;
					Pointlist.push_back(Point(xTmp,yTmp));
					Pointlist_m.push_back(Point(xTmp/1000,yTmp/1000));
				}			
			}
		}

		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		Range2D point_range;
		point_range.x_min = x_min_point;
		point_range.x_max = x_max_point;
		point_range.y_min = y_min_point;
		point_range.y_max = y_max_point;
		Range2D line_range;
		line_range.x_min = x_min_line;
		line_range.x_max = x_max_line;
		line_range.y_min = y_min_line;
		line_range.y_max = y_max_line;

		m_StringList[mapName] = Stringlist;
		m_existMap[mapName] = true;
		m_resolution[mapName] = resolution;
		m_pointRange[mapName] = point_range;
		m_numPoints[mapName] = num_point;
		m_Pointlist[mapName] = Pointlist;
		m_Pointlist_m[mapName] = Pointlist_m;
		m_lineRange[mapName] = line_range;
		m_numLines[mapName] = num_line;
		m_Linelist[mapName] = Linelist;
		m_MapInfolist[mapName] = MapInfolist;
		m_Objectlist[mapName] = Objectlist;
		m_UINumLines[mapName] = UINumLines;
		m_UINumCurves[mapName] = UINumCurves;
		m_UINumRects[mapName] = UINumRects;
		m_UINumPolys[mapName] = UINumPolys;
		m_UINumCircles[mapName] = UINumCircles;
		m_UINumEclipse[mapName] = UINumEclipse;
		m_UiInfolist[mapName] = UiInfolist;

		// close file
		infile.close();

		m_isLoaded = true;

		return ;
	}

	void MapReaderMapper::StoreMapInfo(const std::string& mapName, std::vector <std::string>& mapInfo, const std::string& str)
	{
		mapInfo.push_back(str);

		return ;
	}

	void MapReaderMapper::ObjectParse(const std::string& mapName, std::vector <MapObject>& ObjectList, const std::string& str)
	{
		MapObject object;
		//memset(&object,0,sizeof(MapObject));
		int curEleNum = m_elementNum[mapName];
		m_elementNum[mapName] = curEleNum + 1;
		object.id = m_elementNum[mapName];

		std::istringstream istrstr(str.c_str());
		std::string strhead, strtmp;
		istrstr >> strhead;	//ÉáÈ¥Í·: "Cairn:"

		istrstr >> strtmp;
		if ("ForbiddenArea" == strtmp) {
			object.type = ForbiddenArea;
			object.mark = Marker_ForbiddenArea;
		} else if ("ForbiddenLine" == strtmp) {
			object.type = ForbiddenLine;
			object.mark = Marker_ForbiddenLine;
		} else if ("RobotHome" == strtmp) {
			object.type = RobotHome;
			object.mark = Marker_HomePoint;
		} else if ("Dock" == strtmp) {
			object.type = Dock;
			object.mark = Marker_Dock;
		} else if ("Goal" == strtmp) {
			object.type = Goal;
			object.mark = Marker_Goal;
		} else if ("GoalWithHeading" == strtmp) {
			object.type = GoalWithHeading;
			object.mark = Marker_Goal;
		} else if ("Sim.BoxObstacle" == strtmp) {
			object.type = SimBoxObstacle;
			object.mark = Marker_SimBoxObstacle;
		} else if ("PathPoint" == strtmp){
			object.type = PathPoint;
			object.mark = Marker_PathPoint;
		} else if("SpecalArea" == strtmp ){
			object.type = SpecalArea;
			object.mark = Marker_SpecalArea;
		}else if("LandMark" == strtmp)
		{
			object.type = LandMark;
			object.mark = Marker_LandMark;
		}else if("Route" == strtmp ){
			object.type = Route;
			object.mark = Marker_Route;
		}else {
			object.type = None;
			object.mark = Marker_None;
		}

		if(object.type == Route )
		{
			istrstr >> object.startName ;
			istrstr >> object.endName;
			istrstr >> object.taskMode;
			istrstr >> object.toPointDist;
			istrstr >> object.toPointAngle;
			istrstr >> object.isAvoidance;
			istrstr >> object.SecurityDist;
			istrstr >> object.MaxSpeed;
			ObjectList.push_back(object);
			return;
		}

		object.tagname = strtmp;

		int x,y;
		istrstr >> x >> y;
		object.center = Point(x,y);
		if( object.type == LandMark )
		{
			ObjectList.push_back(object);
			return;
		}
		istrstr >> object.dir;

		std::string str1,str2,stricon;
		istrstr>>str1>>str2>>stricon;

		char* desc = (char*)str1.c_str();
		*(desc+str1.length()-1) = 0;
		desc += 1;
		object.description = std::string(desc);
		char* tTagname = (char*)stricon.c_str();
		*(tTagname+stricon.length()-1) = 0;
		tTagname += 1;
		object.tagname = std::string(tTagname);

		if((object.type == RobotHome)||(object.type == GoalWithHeading)||(object.type == PathPoint)||(object.type == Goal))
		{
			int function = -1;
			bool isRight = false;
			bool isDown = false;
			istrstr>>function>>isRight>>isDown;
			object.function = function;
			object.isRight =  isRight;
			object.isDown = isDown;
		}

		if( object.type != ForbiddenArea 
			&& object.type != SpecalArea 
			&& object.type != ForbiddenLine
			&& object.type != SimBoxObstacle )
		{
			ObjectList.push_back(object);
			return;
		}

		int x1,y1,x2,y2;
		istrstr >> x1 >> y1 >> x2 >> y2;
		object.line = Line(Point(x1,y1), Point(x2,y2));
		ObjectList.push_back(object);

		return;
	}

	void MapReaderMapper::Save(const std::string & mapName)
	{
		SaveAs(mapName,m_curMapName);
	}

	void MapReaderMapper::SaveAs(const std::string & mapName,const std::string & fileName)
	{
		std::fstream infile;
		Range2D rect,lineRect;
		std::vector <std::string> strsList;
		std::vector <MapObject> objectList;
		std::vector <Point> pointList;
		std::vector <Line> lineList;
		std::vector <std::string> UiInfolist;
		std::vector <std::string> Stringlist;
		Point point;
		Line line;
		std::locale::global(std::locale(""));
		infile.open(fileName.c_str(),std::ios::out);
		std::locale::global(std::locale("C"));

		if (!infile.is_open()) {
			std::cout << "can not open file : " << fileName << std::endl ;
			return ;
		} else {
			std::cout << "Succeed in opening file : " << fileName << std::endl;
		}
		strsList = m_MapInfolist[mapName];
		objectList = m_Objectlist[mapName];
		pointList = m_Pointlist[mapName];
		lineList = m_Linelist[mapName];
		rect = m_pointRange[mapName];
		lineRect = m_lineRange[mapName];
		UiInfolist = m_UiInfolist[mapName];
		Stringlist = m_StringList[mapName];

		infile << "2D-Map ";
		for (int i = 0;i<Stringlist.size();i++)
		{
			infile << Stringlist.at(i)<<" ";
		}
		infile << std::endl;
		infile << "MinPos: " << rect.x_min << " " << rect.y_min <<std::endl ;
		infile << "MaxPos: " << rect.x_max << " " << rect.y_max <<std::endl ;
		infile << "NumPoints: " << pointList.size() <<std::endl ;
		infile << "Resolution: " << m_resolution[mapName] <<std::endl ;
		infile << "LineMinPos: " << lineRect.x_min << " " << lineRect.y_min <<std::endl ;
		infile << "LineMaxPos: " << lineRect.x_max << " " << lineRect.y_max <<std::endl ; 
		infile << "NumLines: " << lineList.size() <<std::endl ;
		infile << "UINumLines: " << m_UINumLines[mapName] <<std::endl ;
		infile << "UINumCurves: " << m_UINumCurves[mapName] <<std::endl ;
		infile << "UINumRects: " << m_UINumRects[mapName] <<std::endl ;
		infile << "UINumPolys: " << m_UINumPolys[mapName] <<std::endl ;
		infile << "UINumCircles: " << m_UINumCircles[mapName] <<std::endl ;
		infile << "UINumEclipse: " << m_UINumEclipse[mapName] <<std::endl ;
		for(int i = 0 ; i < strsList.size() ; i++)
		{
			//infile << "MapInfo: " << strsList.at(i) << std::endl;
		}

		for(int i = 0 ; i < objectList.size() ; i++)
		{
			GetObjectDate(objectList.at(i),infile);
		}
		for(int i = 0 ; i < UiInfolist.size() ; i++)
		{
			infile << UiInfolist.at(i) << std::endl;
		}
		if(lineList.size() > 0)
			infile << "LINES" << std::endl;
		for(int i = 0 ; i < lineList.size() ; i++)
		{
			line = lineList.at(i);
			infile << line.p1.x << " "
				<< line.p1.y << " "
				<< line.p2.x << " "
				<< line.p2.y << std::endl;
		}
		infile << "DATA" << std::endl;
		for(int i = 0 ; i < pointList.size() ; i++)
		{
			point = pointList.at(i);
			infile << point.x << " "
				<< point.y << std::endl;
		}

		infile.close();
	}

	std::string MapReaderMapper::getstringFormInt( const int n )
	{
		std::stringstream newstr;
		newstr<<n;
		return newstr.str();
	}

	std::string MapReaderMapper::getstringFormDouble( const double n )
	{
		std::stringstream newstr;
		newstr<<n;
		return newstr.str();
	}

	void MapReaderMapper::GetObjectDate(MapObject object, std::fstream &file)
	{
		std::string str;
		std::string num;
		str += "Cairn: ";
		switch(object.type)
		{
		case ForbiddenArea:
			str += "ForbiddenArea ";
			break;
		case ForbiddenLine:
			str += "ForbiddenLine ";
			break;
		case RobotHome:
			str += "RobotHome ";
			break;
		case Dock:
			str += "Dock ";
			break;
		case Goal:
			str += "Goal ";
			break;
		case GoalWithHeading:
			str += "GoalWithHeading ";
			break;
		case SimBoxObstacle:
			str += "Sim.BoxObstacle ";
			break;
		case PathPoint:
			str += "PathPoint ";
			break;
		case SpecalArea:
			str +="SpecalArea ";
			break;
		case LandMark:
			str +="LandMark ";
			break;
		case Route:
			str +="Route ";
			break;
		case None:
			break;
		default:
			break;
		}
		if( object.type == Route )
		{
			str += object.startName ;
			str += " ";
			str += object.endName ;
			str += " ";
			str += getstringFormInt(object.taskMode);
			str += " ";
			str += getstringFormDouble(object.toPointDist);
			str += " ";
			str += getstringFormDouble(object.toPointAngle);
			str += " ";
			str += getstringFormInt((int)object.isAvoidance);
			str += " ";
			str += getstringFormDouble(object.SecurityDist);
			str += " ";
			str += getstringFormDouble(object.MaxSpeed);
			file << str;
			file << std::endl;
			return;
		}
		str += getstringFormDouble(object.center.x);
		str += " ";
		str += getstringFormDouble(object.center.y);
		if( object.type == LandMark )
		{
			file << str;
			file << std::endl;
			return;
		}
		str += " ";
		str += getstringFormDouble(object.dir);
		str += " ";
		str += " \"";
		str += object.description;
		str += "\" " ;
		str += "ICON \"" ;
		file << str;
		str.clear();
		file << object.tagname;
		//STD_OUT_DEBUG("object.tagname",object.tagname);
		file <<"\" ";
		if((object.type == RobotHome)||(object.type == GoalWithHeading)||(object.type == PathPoint)||(object.type == Goal))
		{
			str += getstringFormInt(object.function);
			str += " ";
			str += getstringFormInt(object.isRight);
			str += " ";
			str += getstringFormInt(object.isDown);
			file << str;
		}
		if( object.type != ForbiddenArea
			&& object.type != SpecalArea
			&& object.type != ForbiddenLine
			&& object.type != SimBoxObstacle )
		{
			file << std::endl;
			return;
		}
		str += getstringFormDouble(object.line.p1.x);
		str += " ";
		str += getstringFormDouble(object.line.p1.y);
		str += " ";
		str += getstringFormDouble(object.line.p2.x);
		str += " ";
		str += getstringFormDouble(object.line.p2.y);
		file << str;
		file << std::endl;
		return;
	}

	bool MapReaderMapper::GetObjectCenterByName( const std::string& mapName, const std::string& name, Point& center, double& dir )
	{
		bool exist = false;
		const std::vector <MapObject>& Objectlist = m_Objectlist[mapName];
		for(std::vector< MapObject >::const_iterator it = Objectlist.begin(); it != Objectlist.end(); ++it)
		{
			if (it->tagname == name) {
				center = it->center;
				dir = deg2rad(normalizeDeg(it->dir));
				exist = true;		
				break;
			}
		}

		return exist;
	}

	bool MapReaderMapper::GetObjectByName( const std::string& mapName, const std::string& name, MapObject& obj)
	{
		bool exist = false;
		const std::vector <MapObject>& Objectlist = m_Objectlist[mapName];
		for(std::vector< MapObject >::const_iterator it = Objectlist.begin(); it != Objectlist.end(); ++it)
		{
			if (it->tagname == name) {
				obj = *it;
				exist = true;		
				break;
			}
		}

		return exist;
	}

	std::vector <std::string> MapReaderMapper::GetStingListByName( const std::string& mapName)
	{
		return m_StringList[mapName];
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	void MapReaderMapper::CheckOK(const std::string& mapName)
	{
		return ;
	}

	void MapReaderMapper::SetPointlist(const std::string& mapName,std::vector< Point >& points)
	{
		m_Pointlist[mapName] = points;
	}
	void MapReaderMapper::SetLinelist(const std::string& mapName,std::vector< Line >& lines)
	{
		m_Linelist[mapName] = lines;
	}

	void MapReaderMapper::SetObjectlist(const std::string& mapName,std::vector< MapObject >& objects)
	{
		m_Objectlist[mapName] = objects;
	}

	void MapReaderMapper::SetStingListByName( const std::string& mapName, std::vector <std::string> stringlist)
	{
		m_StringList[mapName] = stringlist;
	}

	void MapReaderMapper::SetPointRange(const std::string& mapName,const Range2D & rect)
	{
		m_pointRange[mapName] = rect;
	}

	int MapReaderMapper::GetResolution(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_resolution[mapName];
	}

	const Range2D& MapReaderMapper::GetPointRange(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_pointRange[mapName];
	}

	int MapReaderMapper::GetPointNum(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_numPoints[mapName];
	}

	const Range2D& MapReaderMapper::GetLineRange(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_lineRange[mapName];
	}

	int MapReaderMapper::GetLineNum(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_numLines[mapName];
	}

	const std::vector< Point >& MapReaderMapper::GetPointlist( const std::string& mapName )
	{
		CheckOK(mapName);

		return m_Pointlist[mapName];
	}

	const std::vector< Line >& MapReaderMapper::GetLinelist( const std::string& mapName )
	{
		CheckOK(mapName);

		return m_Linelist[mapName];
	}

	const std::vector< MapObject >& MapReaderMapper::GetObjectlist(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_Objectlist[mapName];
	}

	const std::vector< std::string >& MapReaderMapper::GetMapInfolist(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_MapInfolist[mapName];
	}

	int MapReaderMapper::GetElementNum(const std::string& mapName)
	{
		CheckOK(mapName);

		return m_elementNum[mapName];
	}

}
