/** \file
	\brief Parsing the *.map maps
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/
#ifndef NRF_MAPREADER_H
#define NRF_MAPREADER_H

//////////////////////////////////////////////////////////////////////////
// include files
#include <vector>
#include "map_misc.h"
#include <common/types.h>
#include <common/singleton.h>
#include <map>

namespace NJRobot
{

	/** \brief class for reading and parsing *.map */
	class MapReaderMapper {
	public:
		/**\brief Constructor*/
		MapReaderMapper();

		/**\brief Destroyer*/
		~MapReaderMapper();

		/**\brief Initialize*/
		void Initialize();

		/**\brief Reset*/
		void Reset();

		/**\brief Read in a map file */
		void ReadIn(const std::string& mapName);
		/**\brief Is map file loaded in?*/
		bool IsMapLoaded(const std::string& mapName);
		bool IsMapLoaded();

		int GetResolution(const std::string& mapName);
		const Range2D& GetPointRange(const std::string& mapName);
		int GetPointNum(const std::string& mapName);
		const Range2D& GetLineRange(const std::string& mapName);
		int GetLineNum(const std::string& mapName);
		const std::vector< Point >& GetPointlist(const std::string& mapName);
		const std::vector< Point >& GetPointlist_m(const std::string& mapName){return m_Pointlist_m[mapName];}
		const std::vector< Line >& GetLinelist(const std::string& mapName);
		const std::vector< MapObject >& GetObjectlist(const std::string& mapName);
		const std::vector< std::string >& GetMapInfolist(const std::string& mapName);
		bool GetObjectCenterByName(const std::string& mapName, const std::string& name, Point& center, double& dir);
		int GetElementNum(const std::string& mapName);
		void SetPointlist(const std::string& mapName,std::vector< Point > &points);
		void SetPointRange(const std::string& mapName,const Range2D& rect);
		void SetLinelist(const std::string& mapName,std::vector< Line >& lines);
		void SetObjectlist(const std::string& mapName,std::vector< MapObject >& objects);
		void SaveAs(const std::string & mapName,const std::string & fileName);
		void Save(const std::string & mapName);
		void SetLineRange(const std::string& mapName,const Range2D& rect){m_lineRange[mapName] = rect ;}
		bool GetObjectByName( const std::string& mapName, const std::string& name, MapObject& obj);
		void SetStingListByName( const std::string& mapName, std::vector <std::string> stringlist);
		std::vector <std::string> GetStingListByName( const std::string& mapName);

	private:
		/// Check 
		void CheckOK(const std::string& mapName);

		/// Parser
		void Parse(const std::string& mapName);
		void StoreMapInfo(const std::string& mapName, std::vector <std::string>& mapInfo, const std::string& str);
		void ObjectParse(const std::string& mapName, std::vector <MapObject>& object, const std::string& str);
		void GetObjectDate(MapObject object, std::fstream &file);
		std::string getstringFormInt( const int n );
		std::string getstringFormDouble( const double n );
		/// Members
		bool												m_isLoaded;
		std::string											m_curMapName;
		std::map <std::string, bool>						m_existMap;
		std::map <std::string, int>							m_resolution;
		std::map <std::string, Range2D>						m_pointRange;
		std::map <std::string, int>							m_numPoints;
		std::map <std::string, std::vector <Point> >	    m_Pointlist;
		std::map <std::string, std::vector <Point> >	    m_Pointlist_m;
		std::map <std::string, Range2D>						m_lineRange;
		std::map <std::string, int>							m_numLines;
		std::map <std::string, std::vector <Line> >			m_Linelist;
		std::map <std::string, std::vector <std::string> >	m_MapInfolist;
		std::map <std::string, std::vector <MapObject> >	m_Objectlist;
		std::map <std::string, int>							m_elementNum;
		std::map <std::string, int>							m_UINumLines;
		std::map <std::string, int>							m_UINumCurves;
		std::map <std::string, int>							m_UINumRects;
		std::map <std::string, int>							m_UINumPolys;
		std::map <std::string, int>							m_UINumCircles;
		std::map <std::string, int>							m_UINumEclipse;
		std::map <std::string, std::vector <std::string> >	m_UiInfolist;
		std::map <std::string, std::vector <std::string> >  m_StringList;
	};

	typedef NormalSingleton< MapReaderMapper > NRF_MapReaderMapper;	// singleton model


}

#endif // ~NRF_MAPREADER_H