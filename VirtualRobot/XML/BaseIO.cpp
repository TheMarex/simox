
#include "BaseIO.h"
#include "../RobotFactory.h"
#include "../RobotNodeSet.h"
#include "../RuntimeEnvironment.h"
#include "../VirtualRobotException.h"
#include "../EndEffector/EndEffector.h"
#include "../EndEffector/EndEffectorActor.h"
#include "../Nodes/RobotNodeFactory.h"
#include "../Nodes/RobotNodeFixedFactory.h"
#include "../Transformation/DHParameter.h"
#include "../Visualization/VisualizationFactory.h"
#include "rapidxml.hpp"
#include <boost/pointer_cast.hpp>
#include <boost/filesystem.hpp>


namespace VirtualRobot {


boost::mutex BaseIO::mutex;

BaseIO::BaseIO()
{
}

BaseIO::~BaseIO()
{
}

bool BaseIO::isTrue(const char* s)
{
	std::string e = getLowerCase(s);
	if (e == "true" || e=="1" || e=="yes")
		return true;
	return false;
}


/**
 * This method converts \p s into a float value and returns it.
 * If \p s is NULL or not a string representation of a float
 * a VirtualRobot::VirtualRobotException is thrown.
 *
 * \param s the string to convert to float
 *
 * \return the passed in float value
 */
float BaseIO::convertToFloat(const char* s)
{
	THROW_VR_EXCEPTION_IF(NULL == s, "Passing Null string to convertToFloat()");
	std::stringstream floatStream;
	floatStream << std::string(s);
	float result;

	if (!(floatStream >> result))
	{
		THROW_VR_EXCEPTION("The string can not be parsed into a float value");
	}

	return result;
}


/**
 * This method gets the attribute \p attributeName from xml_node \p xmlNode and
 * returns its value as float.
 * If an error occurs (NULL data, missing attribute, conversion failed)
 * a VirtualRobot::VirtualRobotException is thrown.
 */
float BaseIO::getFloatByAttributeName(rapidxml::xml_node<char>* xmlNode, const std::string& attributeName)
{
	THROW_VR_EXCEPTION_IF(!xmlNode, "getFloatByAttributeName got NULL data");
	rapidxml::xml_attribute<> *attr = xmlNode->first_attribute(attributeName.c_str(), 0, false);
	THROW_VR_EXCEPTION_IF(!attr, "The node <" << xmlNode->name() << "> does not contain an attribute named " << attributeName);
	return convertToFloat(attr->value());
}

/**
 * This method gets an optional attribute \p attributeName from xml_node \p xmlNode and
 * returns its value as float. 
 * When no attribute \p attributeName is present the \p standardValue is returned.
 * 
 */
float BaseIO::getOptionalFloatByAttributeName(rapidxml::xml_node<char>* xmlNode, const std::string& attributeName, float standardValue)
{
	THROW_VR_EXCEPTION_IF(!xmlNode, "getFloatByAttributeName got NULL data");
	rapidxml::xml_attribute<> *attr = xmlNode->first_attribute(attributeName.c_str(), 0, false);
	if (!attr)
		return standardValue;
	return convertToFloat(attr->value());
}



/**
 * This method processes <Transform> tags.
 * If \p transformXMLNode is NULL (e.g. the tag does not exist) \p transform
 * is set to contain the identity matrix.
 */
void BaseIO::processTransformNode(rapidxml::xml_node<char> *transformXMLNode, const std::string &tagName, Eigen::Matrix4f &transform)
{
	if (!transformXMLNode)
	{
		transform = Eigen::Matrix4f::Identity();
		return;
	}

	rapidxml::xml_node<> *trXMLNode = transformXMLNode->first_node("transform",0,false);
	THROW_VR_EXCEPTION_IF(!trXMLNode, "transformation node does not specify a <transform> tag")

	bool rotation = false;
	bool translation = false;

	// Homogeneous Matrix 4x4
	rapidxml::xml_node<> *matrixXMLNode = trXMLNode->first_node("matrix4x4",0,false);
	if (matrixXMLNode)
	{
		rapidxml::xml_node<> *row1XMLNode = matrixXMLNode->first_node("row1",0,false);
		rapidxml::xml_node<> *row2XMLNode = matrixXMLNode->first_node("row2",0,false);
		rapidxml::xml_node<> *row3XMLNode = matrixXMLNode->first_node("row3",0,false);
		rapidxml::xml_node<> *row4XMLNode = matrixXMLNode->first_node("row4",0,false);
		if (row1XMLNode)
		{
			transform(0, 0) = getFloatByAttributeName(row1XMLNode, "c1");
			transform(0, 1) = getFloatByAttributeName(row1XMLNode, "c2");
			transform(0, 2) = getFloatByAttributeName(row1XMLNode, "c3");
			transform(0, 3) = getFloatByAttributeName(row1XMLNode, "c4");
		}

		if (row2XMLNode)
		{
			transform(1, 0) = getFloatByAttributeName(row2XMLNode, "c1");
			transform(1, 1) = getFloatByAttributeName(row2XMLNode, "c2");
			transform(1, 2) = getFloatByAttributeName(row2XMLNode, "c3");
			transform(1, 3) = getFloatByAttributeName(row2XMLNode, "c4");
		}

		if (row3XMLNode)
		{
			transform(2, 0) = getFloatByAttributeName(row3XMLNode, "c1");
			transform(2, 1) = getFloatByAttributeName(row3XMLNode, "c2");
			transform(2, 2) = getFloatByAttributeName(row3XMLNode, "c3");
			transform(2, 3) = getFloatByAttributeName(row3XMLNode, "c4");
		}

		if (row4XMLNode)
		{
			transform(3, 0) = getFloatByAttributeName(row4XMLNode, "c1");
			transform(3, 1) = getFloatByAttributeName(row4XMLNode, "c2");
			transform(3, 2) = getFloatByAttributeName(row4XMLNode, "c3");
			transform(3, 3) = getFloatByAttributeName(row4XMLNode, "c4");
		}
		rotation = true;
		translation = true;
	}

	// Rotation Matrix 3x3
	matrixXMLNode = trXMLNode->first_node("matrix3x3",0,false);
	THROW_VR_EXCEPTION_IF((rotation && matrixXMLNode), "Multiple rotations defined in <Transformation> tag: " << tagName << ". Ignoring matrix3x3 node." << endl);

	if (matrixXMLNode)
	{
		
		rapidxml::xml_node<> *row1XMLNode = matrixXMLNode->first_node("row1",0,false);
		rapidxml::xml_node<> *row2XMLNode = matrixXMLNode->first_node("row2",0,false);
		rapidxml::xml_node<> *row3XMLNode = matrixXMLNode->first_node("row3",0,false);
		if (row1XMLNode)
		{
			transform(0, 0) = getFloatByAttributeName(row1XMLNode, "c1");
			transform(0, 1) = getFloatByAttributeName(row1XMLNode, "c2");
			transform(0, 2) = getFloatByAttributeName(row1XMLNode, "c3");
		}
		if (row2XMLNode)
		{
			transform(1, 0) = getFloatByAttributeName(row2XMLNode, "c1");
			transform(1, 1) = getFloatByAttributeName(row2XMLNode, "c2");
			transform(1, 2) = getFloatByAttributeName(row2XMLNode, "c3");
		}

		if (row3XMLNode)
		{
			transform(2, 0) = getFloatByAttributeName(row3XMLNode, "c1");
			transform(2, 1) = getFloatByAttributeName(row3XMLNode, "c2");
			transform(2, 2) = getFloatByAttributeName(row3XMLNode, "c3");
		}
		rotation = true;
	}

	// ROLL PITCH YAW
	rapidxml::xml_node<> *rpyXMLNode = trXMLNode->first_node("rollpitchyaw",0,false);
	THROW_VR_EXCEPTION_IF((rpyXMLNode && rotation), "Multiple rotations defined in <Transformation> tag: " << tagName << "! Ignoring rpy node." << endl);

	if (rpyXMLNode)
	{
		float r,p,y;
		r = p = y = 0.0f;
		r = getFloatByAttributeName(rpyXMLNode, "roll");
		p = getFloatByAttributeName(rpyXMLNode, "pitch");
		y = getFloatByAttributeName(rpyXMLNode, "yaw");

		Units u = getUnitsAttribute(rpyXMLNode,Units::eAngle);
		if (u.isDegree())
		{
			r = r/180.0f * (float)M_PI;
			p = p/180.0f * (float)M_PI;
			y = y/180.0f * (float)M_PI;
		}
		MathTools::rpy2eigen4f(r,p,y,transform);
		rotation = true;
	}

	// Quaternions
	rapidxml::xml_node<> *quatXMLNode = trXMLNode->first_node("quaternion",0,false);
	THROW_VR_EXCEPTION_IF((quatXMLNode && rotation), "Multiple rotations defined in <Transformation> tag: " << tagName << "! Ignoring quaternion node." << endl);

	if (quatXMLNode)
	{
		float x,y,z,w;
		x = y = z = w = 0.0f;
		x = getFloatByAttributeName(quatXMLNode, "x");
		y = getFloatByAttributeName(quatXMLNode, "y");
		z = getFloatByAttributeName(quatXMLNode, "z");
		w = getFloatByAttributeName(quatXMLNode, "w");
		Eigen::Matrix4f r = MathTools::quat2eigen4f(x,y,z,w);
		transform.block(0,0,3,3) = r.block(0,0,3,3);
		//MathTools::quat2eigen4f(x,y,z,w,transform);
		rotation = true;
	}
	// Translation
	rapidxml::xml_node<> *translationXMLNode = trXMLNode->first_node("translation",0,false);
	THROW_VR_EXCEPTION_IF((translationXMLNode && translation), "Multiple translations defined in <Transformation> tag: " << tagName << "! Ignoring translation node." << endl);

	if (translationXMLNode)
	{
		transform(0,3) = getFloatByAttributeName(translationXMLNode, "x");
		transform(1,3) = getFloatByAttributeName(translationXMLNode, "y");
		transform(2,3) = getFloatByAttributeName(translationXMLNode, "z");
		translation = true;
	}
}


/**
 * This method processes the unit or units attribute of xml_node \p node.
 *
 * \return instance of VirtualRobot::Units
 */
Units BaseIO::getUnitsAttribute(rapidxml::xml_node<char> *node, Units::UnitsType u)
{
	THROW_VR_EXCEPTION_IF(!node, "NULL data for getUnitsAttribute().")
	rapidxml::xml_attribute<> *attr = node->first_attribute("unit", 0, false);
	if (!attr)
		attr = node->first_attribute("units", 0, false);
	THROW_VR_EXCEPTION_IF(!attr, "Tag <" << node->name() << "> is missing a unit/units attribute.")

	Units unitsAttribute(getLowerCase(attr->value()));
	switch (u)
	{
	case Units::eAngle:
		{
			THROW_VR_EXCEPTION_IF(!unitsAttribute.isAngle(), "Wrong <Units> tag! Expecting angle type." << endl);
		}
		break;
	case Units::eLength:
		{
			THROW_VR_EXCEPTION_IF(!unitsAttribute.isLength(), "Wrong <Units> tag! Expecting length type." << endl);
		}
		break;
	case Units::eWeight:
		{
			THROW_VR_EXCEPTION_IF(!unitsAttribute.isWeight(), "Wrong <Units> tag! Expecting weight type." << endl);
		}
		break;
	default:
		break;
	}

	return unitsAttribute;
}


/**
 * This method creates a std::string from the parameter \p c and calls
 * VirtualRobot::BaseIO::getLowerCase(std::string) on the created string.
 * Afterwards the transformed string is returned.
 */
std::string BaseIO::getLowerCase(const char* c)
{
	THROW_VR_EXCEPTION_IF(NULL == c, "Passing Null string to getLowerCase()");
	std::string res = c;
	getLowerCase(res);
	return res;
}

/**
 * This method applies tolower() to all characters in the string \p aString.
 */
void BaseIO::getLowerCase(std::string& aString)
{
	std::transform(aString.begin(), aString.end(), aString.begin(), tolower);
}


/**
 * This method processes the \p parentNode Tag and extracts a list of <Node name="xyz"/> tags.
 * All other child tags raise a VirtualRobot::VirtualRobotException.
 * The resulting nodes are stored in \p nodeList.
 *
 * If the parameter \p clearList is true all elements from \p nodeList are removed.
 */
void BaseIO::processNodeList(rapidxml::xml_node<char>* parentNode, RobotPtr robot, std::vector<RobotNodePtr>& nodeList, bool clearList /*= true*/)
{
	if (clearList)
	{
		nodeList.clear();
	}
	std::string parentName = processNameAttribute(parentNode,true);
	rapidxml::xml_node<>* node = parentNode->first_node();
	while (node)
	{
		std::string nodeName = getLowerCase(node->name());
		if (nodeName == "node")
		{
			std::string nodeNameAttr = processNameAttribute(node);
			THROW_VR_EXCEPTION_IF(nodeNameAttr.empty(), "Missing name attribute for <Node> belonging to Robot node set " << parentName);
			RobotNodePtr robotNode = robot->getRobotNode(nodeNameAttr);
			THROW_VR_EXCEPTION_IF(!robotNode, "<node> tag with name '" << nodeNameAttr << "' not present in the current robot");
			nodeList.push_back(robotNode);
		} else
		{
			THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in <RobotNodeSet> with name " << parentName);
		}

		node = node->next_sibling();
	}
}

/**
* This method takes a rapidxml::xml_node and returns the value of the
* first tag it finds with name \p atributeName.
* If an error occurs an exception is thrown
* If more than one name attribute is found an exception is thrown.
* If no attribute can be found 0.0 is returned.
*
* \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
*
* \return the value of the attribute or 0.0 if no attribute was found
*/
float BaseIO::processFloatAttribute(const std::string &attributeName, rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
{
	THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute of NULL node" << endl);

	bool result = false;
	float value = 0.0f;
	std::string nodeNameAttr("");
	rapidxml::xml_attribute<char>* attr = node->first_attribute();
	while (attr)
	{
		std::string name = getLowerCase(attr->name());
		if (attributeName == name)
		{
			THROW_VR_EXCEPTION_IF(result, "<" << node->name() << "> tag contains multiple attributes with name " << attributeName);
			value = convertToFloat(attr->value());
			result = true;
		} else
		{
			if (!allowOtherAttributes)
				THROW_VR_EXCEPTION("<" << node->name() << "> tag contains unknown attribute: " << attr->name());
		}

		attr = attr->next_attribute();
	}
	return value;
}

/**
	* This method takes a rapidxml::xml_node and returns the value of the
	* first tag it finds with name \p atributeName.
	* If an error occurs a message is logged to the console and "" is
	* returned.
	* If more than one name attribute is found an exception is thrown.
	* If no attribute can be found 0.0 is returned.
	*
	* \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
	*
	* \return the value of the name attribute or an empty string on error
	*/
std::string BaseIO::processStringAttribute(const std::string &attributeName, rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
{
	THROW_VR_EXCEPTION_IF(!node, "Can not process name attribute of NULL node" << endl);

	bool result = false;
	std::string value;
	std::string nodeNameAttr("");
	rapidxml::xml_attribute<char>* attr = node->first_attribute();
	while (attr)
	{
		std::string name = getLowerCase(attr->name());
		if (attributeName == name)
		{
			THROW_VR_EXCEPTION_IF(result, "<" << node->name() << "> tag contains multiple attributes with name " << attributeName);
			value = attr->value();
			result = true;
		} else
		{
			if (!allowOtherAttributes)
				THROW_VR_EXCEPTION("<" << node->name() << "> tag contains unknown attribute: " << attr->name());
		}

		attr = attr->next_attribute();
	}
	return value;
}

void BaseIO::makeAbsolutePath( const std::string &basePath, std::string &filename )
{
	if (filename.empty())
		return;

	boost::filesystem::path filenameNew(filename);
	boost::filesystem::path filenameBasePath(basePath);

	boost::filesystem::path filenameNewComplete = boost::filesystem::operator/(filenameBasePath,filenameNew);
	filename = filenameNewComplete.string();
}

void BaseIO::makeRelativePath( const std::string &basePath, std::string &filename )
{
	if (filename.empty())
		return;

	bool found = true;
	boost::filesystem::path diffpath;
	boost::filesystem::path tmppath = filename;
	while(tmppath != basePath) {
		diffpath = tmppath.stem() / diffpath;
		tmppath = tmppath.parent_path();
		if (tmppath.empty())
		{
			// no relative path found, take complete path
			diffpath = filename;
			found = false;
			break;
		}
	}

	boost::filesystem::path origPath = filename;
	std::string res = diffpath.string();
	if (found && origPath.extension()!="")
	{
		std::string ext = origPath.extension().c_str(); // should work with with V3 and V2
		res += ext;
	}

	filename = res;

}



/**
 * This method takes a rapidxml::xml_node and returns the value of the
 * first name tag it finds.
 * If an error occurs a message is logged to the console and an empty string is
 * returned.
 * If more than one name attribute is found an exception is thrown.
 *
 * \warning Do NOT use this method if there are several attributes which need to be processed (or set \p allowOtherAttributes to silently ignore other attributes)
 *
 * \return the value of the name attribute or an empty string on error
 */
std::string BaseIO::processNameAttribute(rapidxml::xml_node<char>* node, bool allowOtherAttributes /* = false */)
{
	std::string nameStr("name");
	return processStringAttribute(nameStr,node,allowOtherAttributes);
}



VisualizationNodePtr BaseIO::processVisualizationTag(rapidxml::xml_node<char> *visuXMLNode, const std::string &tagName, const std::string &basePath, bool &useAsColModel)
{
	bool enableVisu = true;
	bool coordAxis = false;
	float coordAxisFactor = 1.0f;
	std::string coordAxisText = "";
	std::string visuCoordType = "";
	useAsColModel = false;
	std::string visuFileType = "";
	std::string visuFile = "";
	rapidxml::xml_attribute<> *attr;
	VisualizationNodePtr visualizationNode;

	if (!visuXMLNode)
		return visualizationNode;

	attr = visuXMLNode->first_attribute("enable", 0, false);
	if (attr)
	{
		enableVisu = isTrue(attr->value());
	}
	if (enableVisu)
	{
		rapidxml::xml_node<> *visuFileXMLNode = visuXMLNode->first_node("file",0,false);
		if (visuFileXMLNode)
		{
			attr = visuFileXMLNode->first_attribute("type", 0, false);
			THROW_VR_EXCEPTION_IF(!attr, "Missing 'type' attribute in <Visualization> tag of node " << tagName << "." << endl)
				visuFileType = attr->value();
			getLowerCase(visuFileType);
			visuFile = processFileNode(visuFileXMLNode,basePath);
			//visuFile = visuFileXMLNode->value();
			//makeAbsolutePath(basePath,visuFile);
		}
		if (visuFile!="")
		{
			VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(visuFileType, NULL);
			if (visualizationFactory)
				visualizationNode = visualizationFactory->getVisualizationFromFile(visuFile);
			else
				VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Node <" << tagName << ">" << endl;
		}

		rapidxml::xml_node<> *coordXMLNode = visuXMLNode->first_node("coordinateaxis",0,false);
		if (coordXMLNode)
		{
			attr = coordXMLNode->first_attribute("enable", 0, false);
			if (attr)
			{
				coordAxis = isTrue(attr->value());
			}
			if (coordAxis)
			{

				coordAxisFactor = getOptionalFloatByAttributeName(coordXMLNode, "scaling", 1.0f);

				attr = coordXMLNode->first_attribute("text", 0, false);
				if (attr)
				{
					coordAxisText = attr->value();
				} else
					coordAxisText = tagName;

				attr = coordXMLNode->first_attribute("type", 0, false);
				THROW_VR_EXCEPTION_IF(!attr, "Missing 'type' attribute in <CoordinateAxis> tag of node " << tagName << "." << endl)
					visuCoordType = attr->value();
				getLowerCase(visuCoordType);
				VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(visuCoordType, NULL);

				if (!visualizationNode)
				{
					// create dummy visu
					if (visualizationFactory)
						visualizationNode = visualizationFactory->createVisualization();
					else
						VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << endl;
				} else
				{
					THROW_VR_EXCEPTION_IF(visuCoordType.compare(visuFileType)!=0, "Different 'type' attributes not supported for <CoordinateAxis> tag and <File> tag of node " << tagName << "." << endl);
				}
				if (visualizationNode && visualizationFactory)
				{
					VisualizationNodePtr coordVisu = visualizationFactory->createCoordSystem(coordAxisFactor,&coordAxisText);
					visualizationNode->attachVisualization("CoordinateSystem",coordVisu);
					//visualizationNode->showCoordinateSystem(true,coordAxisFactor,&coordAxisText);
				} else
					VR_WARNING << "VisualizationFactory of type '" << visuFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << endl;

			}
		}
		rapidxml::xml_node<> *useColModel = visuXMLNode->first_node("useascollisionmodel",0,false);
		if (useColModel)
		{
			useAsColModel = true;
		}
	}
	return visualizationNode;
}


CollisionModelPtr BaseIO::processCollisionTag(rapidxml::xml_node<char> *colXMLNode, const std::string &tagName, const std::string &basePath)
{
	rapidxml::xml_attribute<> *attr;
	std::string collisionFile = "";
	std::string collisionFileType = "";
	CollisionModelPtr collisionModel;
	bool enableCol = true;
	bool bbox = false;

	attr = colXMLNode->first_attribute("enable", 0, false);
	if (attr)
	{
		enableCol = isTrue(attr->value());
	}
	if (enableCol)
	{
		rapidxml::xml_node<> *colFileXMLNode = colXMLNode->first_node("file",0,false);
		if (colFileXMLNode)
		{
			attr = colFileXMLNode->first_attribute("type", 0, false);
			THROW_VR_EXCEPTION_IF(!attr, "Expecting 'type' attribute in <Collisionmodel> tag of node " << tagName << "." << endl);

			collisionFileType = attr->value();
			getLowerCase(collisionFileType);
			collisionFile = processFileNode(colFileXMLNode,basePath);
		
			attr = colFileXMLNode->first_attribute("boundingbox", 0, false);
			if (attr)
			{
				bbox = isTrue(attr->value());
			}

		}
		if (collisionFile!="")
		{
			VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(collisionFileType, NULL);
			VisualizationNodePtr visualizationNode;
			if (visualizationFactory)
				visualizationNode = visualizationFactory->getVisualizationFromFile(collisionFile, bbox);
			else
				VR_WARNING << "VisualizationFactory of type '" << collisionFileType << "' not present. Ignoring Visualization data in Robot Node <" << tagName << ">" << endl;
			if (visualizationNode)
			{
				std::string colModelName = tagName;
				colModelName += "_ColModel";
				// todo: ID?
				collisionModel.reset(new CollisionModel(visualizationNode,colModelName,CollisionCheckerPtr()));
			}
		}
	}
	return collisionModel;
}


void BaseIO::processPhysicsTag(rapidxml::xml_node<char> *physicsXMLNode, const std::string &nodeName, SceneObject::Physics &physics)
{
	THROW_VR_EXCEPTION_IF(!physicsXMLNode, "NULL data for physicsXMLNode in processPhysicsNode()");
	rapidxml::xml_attribute<> *attr;
	rapidxml::xml_node<> *massXMLNode = physicsXMLNode->first_node("mass",0,false);
	if (massXMLNode)
	{
		physics.massKg = getFloatByAttributeName(massXMLNode,"value");
		Units unit = getUnitsAttribute(massXMLNode,Units::eWeight);
		if (unit.isGram())
			physics.massKg *= 0.001f;

	} else
	{
		VR_WARNING << "Expecting mass tag for physics node in <" << nodeName << ">." << endl;
		physics.massKg = 0.0f;
	}
	rapidxml::xml_node<> *comXMLNode = physicsXMLNode->first_node("com",0,false);
	if (comXMLNode)
	{

		attr = comXMLNode->first_attribute("location", 0, false);
		if (attr)
		{
			std::string loc = attr->value();
			getLowerCase(loc);
			if (loc=="visualizationbboxcenter")
			{
				physics.comLocation = SceneObject::Physics::eVisuBBoxCenter;
			} else if (loc=="custom" || loc=="joint")
			{
				physics.comLocation = SceneObject::Physics::eCustom;
			} else
			{
				THROW_VR_EXCEPTION ("Unsupported Physics <CoM> tag attribute:" << loc);
			}
		}
		if (physics.comLocation == SceneObject::Physics::eCustom)
		{
			physics.localCoM(0) = getOptionalFloatByAttributeName(comXMLNode,"x",0.0f);
			physics.localCoM(1) = getOptionalFloatByAttributeName(comXMLNode,"y",0.0f);
			physics.localCoM(2) = getOptionalFloatByAttributeName(comXMLNode,"z",0.0f);
		}
	} 
}

std::string BaseIO::processFileNode( rapidxml::xml_node<char> *fileNode, const std::string &basePath )
{
	THROW_VR_EXCEPTION_IF(!fileNode, "NULL data");
	std::string fileName = fileNode->value();
	THROW_VR_EXCEPTION_IF(fileName.empty(), "Invalid file defined in FILE tag");
	bool relative = true;
	std::string pathStr("path");
	std::string pathAttribute = processStringAttribute(pathStr,fileNode,true);
	if (!pathAttribute.empty())
	{
		pathAttribute = getLowerCase(pathAttribute.c_str());
		if (pathAttribute=="relative")
			makeAbsolutePath(basePath,fileName);
		else if (pathAttribute!="absolute")
		{
			THROW_VR_EXCEPTION ("Unknown path attribute in <File> tag:" << pathAttribute)
		}
	} else
	{
		
		// check file absolute
		boost::filesystem::path fn(fileName);
		if (boost::filesystem::exists(fn))
			return fileName;
		// check file relative
		std::string absFileName = fileName;
		makeAbsolutePath(basePath,absFileName);
		fn = absFileName;
		if (boost::filesystem::exists(fn))
			return absFileName;
		// check file in data paths
		absFileName = fileName;
		if (RuntimeEnvironment::getDataFileAbsolute(absFileName))
			return absFileName;
		
		VR_ERROR << "Could not determine valid filename from " << fileName << endl;
	}
	return fileName;
}

bool BaseIO::processConfigurationNode(rapidxml::xml_node<char>* configXMLNode, std::vector< std::vector< RobotConfig::Configuration > > &configDefinitions, std::vector< std::string > &configNames )
{
	THROW_VR_EXCEPTION_IF(!configXMLNode, "NULL data in processConfigurationNode");
	std::string name = processNameAttribute(configXMLNode,true);
	THROW_VR_EXCEPTION_IF(name.empty(), "Expecting a name in configuration tag");
	std::vector< RobotConfig::Configuration > configs;


	rapidxml::xml_node<>* node = configXMLNode->first_node();
	while (node)
	{
		std::string nodeName = getLowerCase(node->name());
		if (nodeName == "node")
		{
			RobotConfig::Configuration c;
			c.name = processNameAttribute(node,true);
			THROW_VR_EXCEPTION_IF(c.name.empty(), "Expecting a name in configuration tag '" << name << "'.");
			c.value = getFloatByAttributeName(node,"value");
			if (getUnitsAttribute(node,Units::eAngle).isDegree())
			{
				c.value = c.value/180.0f * (float)M_PI;
			}
			configs.push_back(c);
		} else
		{
			THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in scene's Configuration definition wiuth name '" << name << "'." << endl);
		}

		node = node->next_sibling();
	}

	configDefinitions.push_back(configs);
	configNames.push_back(name);
	return true;
}


RobotNodeSetPtr BaseIO::processRobotNodeSet(rapidxml::xml_node<char>* setXMLNode, RobotPtr robo, const std::string& robotRootNode, int& robotNodeSetCounter )
{
	THROW_VR_EXCEPTION_IF(!setXMLNode, "NULL data for setXMLNode");

	std::string nodeSetName;
	std::string rootNodeName;
	std::string tcpName;

	// get name and root
	rapidxml::xml_attribute<> *attr = setXMLNode->first_attribute();
	while (attr)
	{
		std::string name = getLowerCase(attr->name());
		if (name=="name")
		{
			THROW_VR_EXCEPTION_IF(!nodeSetName.empty(), "Robot node set contains multiple definitions of attribute name. First value of name is: " << nodeSetName);
			nodeSetName = attr->value();
		} else if (name=="kinematicroot")
		{
			THROW_VR_EXCEPTION_IF(!rootNodeName.empty(), "Robot node set contains multiple definitions of attribute kinematicroot. First value of kinematicroot is: " << rootNodeName);
			rootNodeName = attr->value();
		} else if (name=="tcp")
		{
			THROW_VR_EXCEPTION_IF(!tcpName.empty(), "Robot node set contains multiple definitions of attribute tcp. First value of tcpis: " << tcpName);
			tcpName = attr->value();
		}
		attr = attr->next_attribute();
	}

	if (nodeSetName.empty())
	{
		std::stringstream ss;
		ss << robo->getType() << "_RobotNodeSet_" << robotNodeSetCounter;
		nodeSetName = ss.str();
		robotNodeSetCounter++;
		VR_WARNING << "RobotNodeSet definition expects attribute 'name'. Setting name to " << nodeSetName << endl;
	}
	if (rootNodeName.empty())
	{
		rootNodeName = robotRootNode;
	}

	std::vector<RobotNodePtr> nodeList;
	processNodeList(setXMLNode, robo, nodeList);

	RobotNodePtr kinRoot;
	if (!rootNodeName.empty())
		kinRoot = robo->getRobotNode(rootNodeName);
	RobotNodePtr tcp;
	if (!tcpName.empty())
		tcp = robo->getRobotNode(tcpName);

	RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robo, nodeSetName, nodeList, kinRoot, tcp, true);

	return rns;
}


} // namespace VirtualRobot
