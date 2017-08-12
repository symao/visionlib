#include <io/param_reader.h>

#include <tinyxml.h>

#include <common/singleton.h>
#include <common/std_out.h>

#include <iostream>
#include <fstream>
#include <algorithm>

namespace NJRobot
{


ParamReader::ParamReader()
: _pParamReaderUI(0)
, _param_loaded(false)
{

}

ParamReader::~ParamReader()
{

}
void ParamReader::saveParams(const char* strFileName)
{
	if (strFileName) {
		saveParams(_headerPairList, strFileName, true);
	}

	return ;
}

void ParamReader::readParams(const char* strFileName)
{
	std::string file_name;
	ParamStatus status;
	if (strFileName) {
		file_name = std::string(strFileName);
		status = eParamChanged;
	} else {
		status = eParamDefault;
	}
	if(_paramFileName == file_name){return;}  //重复读取会出现问题

	TiXmlDocument paramFile(file_name.c_str());
	if (! paramFile.LoadFile()) {
		STD_OUT_DEBUG(file_name,"tiny loading failed!");
		_param_loaded = false;
		return;
	}
	_paramFileName = file_name;
	_param_loaded = true;

	TiXmlNode* projectHeaderNode = paramFile.FirstChild();
	while (projectHeaderNode) {
		if (projectHeaderNode->Type() == TiXmlNode::TINYXML_ELEMENT) {
			TiXmlNode* paramHeaderNode = projectHeaderNode->FirstChild();
			while (paramHeaderNode) {
				if (paramHeaderNode->Type() == TiXmlNode::TINYXML_ELEMENT) {
					std::string paramHeader = paramHeaderNode->Value();
					TiXmlNode* paramNameNode = paramHeaderNode->FirstChild();
					while (paramNameNode) {
						if (paramNameNode->Type() == TiXmlNode::TINYXML_ELEMENT) {
							std::string paramName = paramNameNode->Value();
							TiXmlNode* paramValueNode = paramNameNode->FirstChild();
							if (paramValueNode) {
								std::string paramValue = paramValueNode->Value();
								setParam(paramHeader, paramName, paramValue, status);
							}
						}
						paramNameNode = paramNameNode->NextSibling();
					}
				}
				paramHeaderNode = paramHeaderNode->NextSibling();
			}
		}
		projectHeaderNode = projectHeaderNode->NextSibling();
	}

	return ;
}
void ParamReader::saveParams(const HeaderPairList& headerPairList, const std::string& file_name, bool onlyChanged)
{
	std::ofstream outFile(file_name.c_str());

	for (HeaderPairList::const_iterator header = headerPairList.begin(); header != headerPairList.end(); ++ header) {
		if (onlyChanged) {
			bool hasChangedParam = false;
			for (ParamPairList::const_iterator param=header->params.begin(); param != header->params.end(); ++ param) {
				if (param->status == eParamChanged) {
					hasChangedParam = true;
					break;
				}
			}

			if (! hasChangedParam ) {
				continue;
			}
		}

		for (ParamPairList::const_iterator param=header->params.begin(); param != header->params.end(); ++ param) {
			if (! (onlyChanged && param->status != eParamChanged)) {
			}
		}
	}

	return ;
}

void ParamReader::setParam(const std::string& paramHeader,const std::string& paramName,const std::string& paramValue, ParamStatus status)
{
	HeaderPairList& headerList = _headerPairList;
	HeaderPairList::iterator header = std::find(headerList.begin(),headerList.end(), paramHeader);
	if (header != headerList.end()) {
		ParamPairList::iterator param = std::find(header->params.begin(), header->params.end(), paramName);
		if (param != header->params.end()) {
			param->value = paramValue;
			param->status = status;
			for (std::vector< AddressPair >::iterator v = param->vars.begin(); v != param->vars.end(); ++ v) {
				v->converter(paramValue, v->address);
			}
		} else {
			header->params.push_back(ParamPair(paramName, paramValue, status));
		}
	} else {
		HeaderPair header_pair(paramHeader);
		header_pair.params.push_back(ParamPair(paramName, paramValue, status));
		headerList.push_back(header_pair);
	}

	if (_pParamReaderUI) {
		_pParamReaderUI->showParam(paramHeader, paramName, paramValue, status);
	}

	return ;
}

}
