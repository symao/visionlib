/** \file
	\brief Provide interface to read and write XML configuration
	\author NanJiang Robot Inc.
	\date 2016-02-26
	\version 0.0.1
*/

#ifndef NJ_PARAM_READER_H
#define NJ_PARAM_READER_H

#include <common/definition.h>
#include <common/singleton.h>
#include <common/std_out.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <typeinfo>

namespace NJRobot
{

enum ParamStatus {
	eParamNone,
	eParamDefault,
	eParamBuiltin,
	eParamChanged 
}; 

//////////////////////////////////////////////////////////////////////////
// define the interface for parameter User-Interface
class ParamReaderUI {
public:
	virtual void showParam(const std::string& paramHeader, 
							const std::string& paramName, 
							const std::string& paramValue, 
							ParamStatus status) = 0;
};

template < typename ValueType >
void convertStringToValue(const std::string& str, void* a)
{
	ValueType& v = *((ValueType*)a);
	std::stringstream valueStream;
	valueStream << str;
	valueStream >> v;

	return ;
}
typedef void (*Converter)(const std::string&, void*);

//////////////////////////////////////////////////////////////////////////
// define the class for parameters storing and reading
class ParamReader {
	// pair structure for address
	struct AddressPair {
		AddressPair(void* a, Converter c) : address(a),converter(c) {}
		void* address;
		Converter converter;
	};

	// key-value pair for parameters
	struct ParamPair {
		ParamPair(const std::string& k,const std::string& v, const ParamStatus& s)
				: key(k),value(v),status(s) {}
		bool operator == (const std::string& k) const { return key == k; }
		std::string key;
		std::string value;
		ParamStatus status;
		std::vector< AddressPair > vars;
	};
	typedef std::vector< ParamPair > ParamPairList;
	
	// parameter pair list
	struct HeaderPair {
		HeaderPair(const std::string& k) : key(k) {}
		bool operator == (const std::string& k) const { return key == k; }
		std::string key;
		ParamPairList params;
	};
	typedef std::vector< HeaderPair > HeaderPairList;

public:
	/// Constructor
	ParamReader();

	/// Destructor
	~ParamReader();

	/// Interface for getting parameter
	template < typename ParamType >
	ParamType getParam(const std::string& paramHeader, const std::string& paramName, ParamType& defaultValue)
	{
		if(_param_loaded==false)
		{
			COUT_WARN("ParamReader","No params loaded.");
			return defaultValue;
		}
		addDefault(paramHeader, paramName, valueToString(defaultValue));
		HeaderPairList& headerList = _headerPairList;
		HeaderPairList::iterator header = std::find(headerList.begin(),headerList.end(), paramHeader);
		if (header != headerList.end()) {		// if find the header
			ParamPairList& paramList = header->params;
			ParamPairList::iterator param = std::find(paramList.begin(),paramList.end(), paramName);
			if (param != paramList.end()) {		// if find the param
				param->vars.push_back(AddressPair(&defaultValue, convertStringToValue< ParamType >));
				return stringToValue< ParamType >(param->value);
			}else{
				COUT_WARN("ParamReader","ParamName '"<<paramName<<"' not found");
			}
		}else{
			COUT_WARN("ParamReader","Header '"<<paramHeader<<"' not found");
		}

		return defaultValue;
		//setParam(paramHeader, paramName, valueToString(defaultValue), eParamBuiltin);
		//return getParam(paramHeader, paramName, defaultValue);
	}

	/// Interface for storing parameter
	template < typename ParamType >
	void saveParam(const std::string& paramHeader, const std::string& paramName, const ParamType& value)
	{
		setParam(paramHeader, paramName, valueToString(value), eParamChanged);
	}

	/// Interface for loading parameters
	void readParams(const char* strFileName = 0);
	
	/// Interface for saving parameters
	void saveParams(const char* strFileName = 0);
	
	void registerUI(ParamReaderUI* pPRUI) { _pParamReaderUI = pPRUI; }
	
	void setParam(const std::string& paramHeader,const std::string& paramName,const std::string& paramValue, ParamStatus status);

protected:
	void saveParams(const HeaderPairList& headerPairList, const std::string& fileName, bool onlyChanged);
	
	template < typename ParamType >
	ParamType stringToValue(const std::string& str)
	{
		ParamType returnValue;

		//ÕâÀïÌí¼ÓÁËÒ»¶Î£¬µ±½«×Ö·û´®×ª»»Îª×Ö·û´®µÄÊ±ºò£¬sstreamÖ»»áÈ¡¿Õ¸ñÇ°µÄ²¿·Ö¶ø²»»á¿½±´Õû¸ö×Ö·û´®
		if(typeid(returnValue)==typeid(str))
		{
			returnValue = *(ParamType *)(&str);
		}
		else
		{
			std::stringstream valueStream;
			valueStream << str;
			valueStream >> returnValue;
		}
		return returnValue;
	}
	
	template < typename ParamType >
	std::string valueToString(const ParamType& value)
	{
		std::stringstream valueStream;
		valueStream << value;
		std::string strValue;
		valueStream >> strValue;

		return strValue;
	}

	void addDefault(const std::string& paramHeader, const std::string& paramName, const std::string& paramValue)
	{
		HeaderPairList& headerList = _defaultHeaderPairList;
		HeaderPairList::iterator header = std::find(headerList.begin(),headerList.end(), paramHeader);
		if (header != headerList.end()) { 
			header->params.push_back(ParamPair(paramName, paramValue, eParamBuiltin));
		} else {
			HeaderPair header_pair(paramHeader);
			header_pair.params.push_back(ParamPair(paramName, paramValue, eParamBuiltin));
			headerList.push_back(header_pair);
		}
	}

private:
	HeaderPairList			_headerPairList;
	HeaderPairList			_defaultHeaderPairList;
	std::string				_paramFileName;
	ParamReaderUI*			_pParamReaderUI;
	bool					_param_loaded;
};
typedef NormalSingleton< ParamReader > NJ_ParamReader;

/**
	\brief define some functional macros for interacting with SParamReader
	\code
		LOAD_PARAM_FROM_XML("./myxlmfile")
		int paramName;
		DECLARE_PARAM_READER_BEGIN(ClassName)
		READ_PARAM(paramName)
		DECLARE_PARAM_READER_END
	\endcode
*/

#define LOAD_PARAM_FROM_XML(XmlFile) {NJ_ParamReader::Instance()->readParams(XmlFile);}
#define DECLARE_PARAM_READER_BEGIN(ClassName) {\
	static bool paramRead = false;\
	if (!paramRead) {\
	ParamReader* pReader = NJ_ParamReader::Instance();\
	std::string strClassName(#ClassName);
#define READ_PARAM(ParamName) \
	ParamName = pReader->getParam(strClassName, #ParamName, ParamName);
#define DECLARE_PARAM_READER_END \
	paramRead = true;\
	}}

#define DECLARE_PARAM_SAVER_BEGIN(ClassName) \
{ \
	ParamReader* pReader = NJ_ParamReader::Instance();\
	std::string strClassName(#ClassName);
#define SAVE_PARAM(ParamName) \
	pReader->saveParam(strClassName, #ParamName, ParamName);
#define DECLARE_PARAM_SAVER_END \
	pReader->saveParams(); \
}


}


#endif // ~NJ_PARAM_READER_H
