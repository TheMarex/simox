
#include "RobotNode.h"
#include "../Robot.h"
#include "../VirtualRobotException.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/Visualization.h"
#include "../Visualization/TriMeshModel.h"
#include <cmath>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>

#include <boost/math/special_functions/fpclassify.hpp>

#include <Eigen/Core>

#include "ConditionedLock.h"

namespace VirtualRobot {

RobotNode::RobotNode(	RobotWeakPtr rob, 
						const std::string &name,
						const std::vector<std::string> &childrenNames,
						float jointLimitLo,
						float jointLimitHi,
						VisualizationNodePtr visualization, 
						CollisionModelPtr collisionModel,
						float jointValueOffset,
						const SceneObject::Physics &p,
						CollisionCheckerPtr colChecker) 
						: SceneObject(name,visualization,collisionModel,p,colChecker),
						use_mutex(true)


{
	maxVelocity = 0.0f;
	maxAcceleration = 0.0f;
	maxTorque = 0.0f;
	robot = rob;
	this->childrenNames = childrenNames;
	this->jointLimitLo = jointLimitLo;
	this->jointLimitHi = jointLimitHi;
	this->jointValueOffset = jointValueOffset;
	preJointTransformation = Eigen::Matrix4f::Identity();
	postJointTransformation = Eigen::Matrix4f::Identity();
	optionalDHParameter.isSet = false;
	globalPosePostJoint = Eigen::Matrix4f::Identity();
	jointValue = 0.0f;
	children.clear();
	initialized = false;
}


RobotNode::~RobotNode()
{
	reset();

	// not needed here
	// when robot is destroyed all references to this RobotNode are also destroyed
	//RobotPtr rob = robot.lock();
	//if (rob)
	//	rob->deregisterRobotNode(shared_from_this());
}

void RobotNode::reset()
{
	SceneObject::reset();
	childrenNames.clear();
	jointValueOffset = 0.0f;
	jointLimitLo = 0.0f;
	jointLimitHi = (float)M_PI;
	jointValueOffset = 0.0f;
	preJointTransformation = Eigen::Matrix4f::Identity();
	postJointTransformation = Eigen::Matrix4f::Identity();
	optionalDHParameter.isSet = false;
	globalPosePostJoint = Eigen::Matrix4f::Identity();
	jointValue = 0.0f;
	children.clear();
	parent.reset();
	initialized = false;
}

bool RobotNode::initialize(RobotNodePtr parent, bool initializeChildren)
{
	RobotPtr rob = robot.lock();
	THROW_VR_EXCEPTION_IF(!rob, "Could not init RobotNode without robot" );

	if (rob)
	{
		if (!rob->hasRobotNode(shared_from_this()))
				rob->registerRobotNode(shared_from_this());
	}

	// process parent
	this->parent = parent;
	if (parent)
	{
		if (!parent->hasChildNode(shared_from_this()))
			parent->addChildNode(shared_from_this());
	}

	// process children
	children.clear();
	for (unsigned int i=0; i<this->getChildrenNames().size(); i++)
	{
		RobotNodePtr n = rob->getRobotNode(this->getChildrenNames()[i]);
		if (n)
		{
			children.push_back(n);
			if (initializeChildren)
				n->initialize(shared_from_this(), true);
		}
	}

	// update visualization of coordinate systems
	if (visualizationModel && visualizationModel->hasAttachedVisualization("CoordinateSystem"))
	{
		VisualizationNodePtr v = visualizationModel->getAttachedVisualization("CoordinateSystem");
		// this is a little hack: The globalPose is used to set the "local" position of the attached Visualization:
		// Since the attached visualizations are already positioned at the global pose of the visualizationModel, 
		// we just need the local postJointTransform
		v->setGlobalPose(postJointTransformation);
	}
	updateTransformationMatrices();

	return SceneObject::initialize();	
}


RobotPtr RobotNode::getRobot()
{
	RobotPtr result(robot);
	return result;
}

void RobotNode::setJointValue(float q, bool updateTransformations /*= true*/,
                              bool clampToLimits /*= true*/)
{
	WriteLock lock(mutex,use_mutex);

	VR_ASSERT_MESSAGE( initialized, "Not initialized");
	VR_ASSERT_MESSAGE( (!boost::math::isnan(q) && !boost::math::isinf(q)) ,"Not a valid number...");
	
	//std::cout << "######## Setting Joint to: " << q << " degrees" << std::endl;

	if (q < jointLimitLo)
	{
		THROW_VR_EXCEPTION_IF(!clampToLimits, "Joint limits violated, joint:" << name << ", min:" << jointLimitLo << ", max:" << jointLimitHi << ", q:" << q);
		q = jointLimitLo;
	}
	if (q > jointLimitHi)
	{
		THROW_VR_EXCEPTION_IF(!clampToLimits, "Joint limits violated, joint:" << name << ", min:" << jointLimitLo << ", max:" << jointLimitHi << ", q:" << q);
		q = jointLimitHi;
	}

	jointValue = q;
	if (updateTransformations)
		applyJointValue();

}

void RobotNode::updateTransformationMatrices()
{

}
void RobotNode::updateTransformationMatrices(const Eigen::Matrix4f &globalPose)
{

}

void RobotNode::setPostJointTransformation(const Eigen::Matrix4f &trafo) {
	WriteLock lock(this->mutex,use_mutex);
	postJointTransformation = trafo;
}

void RobotNode::setPreJointTransformation(const Eigen::Matrix4f &trafo) {
	WriteLock lock(this->mutex,use_mutex);
	preJointTransformation = trafo;
}

void RobotNode::applyJointValue()
{
	THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

	updateTransformationMatrices();
	
	std::vector< RobotNodePtr > children = this->getChildren(); //Stefan
	for (std::vector< RobotNodePtr >::iterator i = children.begin(); i!= children.end(); i++ )
		(*i)->applyJointValue();
	//std::for_each(children.begin(), children.end(), boost::mem_fn(&RobotNode::applyJointValue));
}

void RobotNode::applyJointValue(const Eigen::Matrix4f &globalPose)
{
	THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

	updateTransformationMatrices(globalPose);

	std::vector< RobotNodePtr > children = this->getChildren(); //Stefan
	for (std::vector< RobotNodePtr >::iterator i = children.begin(); i!= children.end(); i++ )
		(*i)->applyJointValue();
	//std::for_each(children.begin(), children.end(), boost::mem_fn(&RobotNode::applyJointValue));
}

void RobotNode::collectAllRobotNodes( std::vector< RobotNodePtr > &storeNodes )
{
	storeNodes.push_back(shared_from_this());

	// call ::collectAllRobotNodes() on all members of the children vector
	// with the parameter bound to storeNodes
	std::vector< RobotNodePtr > children = this->getChildren(); //Stefan
	std::for_each(children.begin(), children.end(), boost::bind(&RobotNode::collectAllRobotNodes, _1, boost::ref(storeNodes)));
}

float RobotNode::getJointValue() const
{
	ReadLock lock(this->mutex,use_mutex);
	return jointValue;
}

void RobotNode::respectJointLimits( float &jointValue ) const
{
	WriteLock lock(this->mutex,use_mutex);
	if (jointValue < jointLimitLo)
		jointValue = jointLimitLo;
	if (jointValue > jointLimitHi)
		jointValue = jointLimitHi;
}

bool RobotNode::checkJointLimits( float jointValue, bool verbose ) const
{
	ReadLock lock(this->mutex,use_mutex);
	bool res = true;
	if (jointValue < jointLimitLo)
		res = false;
	if (jointValue > jointLimitHi)
		res = false;
	if (!res && verbose)
			VR_INFO << "Joint: " << getName() << ": joint value (" << jointValue << ") is out of joint boundaries (lo:" << jointLimitLo << ", hi: " << jointLimitHi <<")" << endl;
	return res;
}
void RobotNode::setGlobalPose( const Eigen::Matrix4f &pose )
{
	THROW_VR_EXCEPTION("Use setJointValues to control the position of a RobotNode");
}

void RobotNode::setThreadsafe(bool mode){
	this->use_mutex = mode;
}


void RobotNode::print( bool printChildren, bool printDecoration ) const
{
	ReadLock lock(this->mutex,use_mutex);
	if (printDecoration)
		cout << "******** RobotNode ********" << endl;
	cout << "* Thread-safe: " << use_mutex << endl;
	cout << "* Name: " << name << endl;
	cout << "* Parent: " << this->getParentName() << endl;
	cout << "* Children: ";
	if (this->getChildrenNames().size() == 0)
		cout << " -- " << endl;
	for (unsigned int i = 0; i < this->getChildrenNames().size(); i++)
		cout << this->getChildrenNames()[i] << ", ";
	cout << endl;
	cout << "* Mass: ";
	if (physics.massKg<=0)
		cout << "<not set>" << endl;
	else 
		cout << physics.massKg << " kg" << endl;
	cout << "* CoM:" << physics.localCoM(0) << ", " << physics.localCoM(1) << ", " << physics.localCoM(2) << endl;
	cout << "* Limits: Lo:" << jointLimitLo << ", Hi:" << jointLimitHi << endl;
	std::cout << "* max velocity " << maxVelocity  << " [m/s]" << std::endl;
	std::cout << "* max acceleration " << maxAcceleration  << " [m/s^2]" << std::endl;
	std::cout << "* max torque " << maxTorque  << " [Nm]" << std::endl;
	cout << "* jointValue: " << this->getJointValue() << ", jointValueOffset: " << jointValueOffset << endl;
	if (optionalDHParameter.isSet)
	{
		cout << "* DH parameters: ";
		cout << " a:" << optionalDHParameter.aMM() << ", d:" << optionalDHParameter.dMM() << ", alpha:" << optionalDHParameter.alphaRadian() << ", theta:" << optionalDHParameter.thetaRadian() << endl;
	} else
		cout << "* DH parameters: not specified." << endl;
	cout << "* visualisation model: " <<endl;
	if (visualizationModel)
		visualizationModel->print();
	else
		cout << "  No visualization model" << endl;
	cout << "* collision model: " << endl;
	if (collisionModel)
		collisionModel->print();
	else
		cout << "  No collision model" << endl;

	if (initialized)
		cout << "* initialized: true" << endl;
	else
		cout << "* initialized: false" << endl;

	{ // scope1
		std::ostringstream sos;
		sos << std::setiosflags(std::ios::fixed);
		sos << "* preJointTransformation:" << endl << preJointTransformation << endl; //Stefan
		sos << "* postJointTransformation:" << endl << postJointTransformation << endl; //Stefan
		sos << "* globalPose:" << endl << getGlobalPose() << endl;
		cout << sos.str() << endl;
	} // scope1

	if (printDecoration)
		cout << "******** End RobotNode ********" << endl;

	if (printChildren)
	{
		std::vector< RobotNodePtr > children = this->getChildren(); //Stefan
		for (unsigned int i = 0; i < children.size(); i++)
			children[i]->print(true, true);
	}
}

void RobotNode::addChildNode( RobotNodePtr child )
{
	if (!child)
		return;
	if (!this->hasChildNode(child->getName()))
		childrenNames.push_back(child->getName());

	if (!this->hasChildNode(child) && initialized)
		this->children.push_back(child);
}

bool RobotNode::hasChildNode( const RobotNodePtr child, bool recursive ) const
{
	std::vector< RobotNodePtr > children = this->getChildren(); //Stefan
	for (unsigned int i = 0; i < children.size(); i++)
	{
		if (children[i] == child)
			return true;
		if (recursive)
		{
			if (children[i]->hasChildNode(child, true))
				return true;
		}	
	}
	return false;
}

bool RobotNode::hasChildNode( const std::string &child, bool recursive ) const
{
	RobotPtr rob(robot);
	for (unsigned int i=0; i<this->getChildrenNames().size(); i++)
	{
	THROW_VR_EXCEPTION_IF(!rob, "no robot" );

		if (this->getChildrenNames()[i] == child)
			return true;
		if (recursive)
		{
			RobotNodePtr p = rob->getRobotNode(this->getChildrenNames()[i]);
			if (p && p->hasChildNode(child))
				return true;
		}	
	}
	return false;
}

RobotNodePtr RobotNode::clone( RobotPtr newRobot, bool cloneChildren, RobotNodePtr initializeWithParent, CollisionCheckerPtr colChecker )
{
	ReadLock lock(this->mutex,use_mutex);
	if (!newRobot)
	{
		VR_ERROR << "Attempting to clone RobotNode for invalid robot";
		return RobotNodePtr();
	}

	std::vector< std::string > clonedChildrenNames;
	if (cloneChildren)
		clonedChildrenNames = this->getChildrenNames();

	VisualizationNodePtr clonedVisualizationNode;
	if (visualizationModel)
		clonedVisualizationNode = visualizationModel->clone();
	CollisionModelPtr clonedCollisionModel;
	if (collisionModel)
		clonedCollisionModel = collisionModel->clone(colChecker);

	RobotNodePtr result = _clone(newRobot, clonedChildrenNames, clonedVisualizationNode, clonedCollisionModel);

	if (!result)
	{
		VR_ERROR << "Cloning failed.." << endl;
		return result;
	}

	if (!cloneChildren)
	{
		if (initializeWithParent)
			result->initialize(initializeWithParent);
	} else
	{
		std::vector< RobotNodePtr > children = this->getChildren(); //Stefan
		for (unsigned int i=0; i<children.size(); i++)
		{
			RobotNodePtr c = children[i]->clone(newRobot,true,RobotNodePtr(),colChecker);
			if (c)
			{
				result->addChildNode(c);
			}
		}
	}

	result->setMaxVelocity(maxVelocity);
	result->setMaxAcceleration(maxAcceleration);
	result->setMaxTorque(maxTorque);
	newRobot->registerRobotNode(result);

	if (initializeWithParent)
		result->initialize(initializeWithParent, true);

	return result;
}


float RobotNode::getJointLimitLo()
{
	ReadLock lock(this->mutex,use_mutex);
	return jointLimitLo;
}

float RobotNode::getJointLimitHi()
{
	ReadLock lock(this->mutex,use_mutex);
	return jointLimitHi;
}
	
bool RobotNode::isTranslationalJoint() const
{
	return false;
}

bool RobotNode::isRotationalJoint() const
{
	return false;
}


void RobotNode::showCoordinateSystem( bool enable, float scaling, std::string *text, const std::string &visualizationType)
{
	if (!enable && !visualizationModel)
		return; // nothing to do

	if (!ensureVisualization(visualizationType))
		return;

	std::string coordName = name;
	if (text)
		coordName = *text;
	if (visualizationModel->hasAttachedVisualization("CoordinateSystem"))
	{
		visualizationModel->detachVisualization("CoordinateSystem");
	}
	if (enable)
	{
		VisualizationFactoryPtr visualizationFactory;
		if (visualizationType.empty())
			visualizationFactory = VisualizationFactory::first(NULL);
		else
			visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
		if (!visualizationFactory)
		{
			VR_WARNING << "No visualization factory for name " << visualizationType << endl;
			return;
		}
		// create coord visu
		VisualizationNodePtr visualizationNode = visualizationFactory->createCoordSystem(scaling,&coordName);
		// this is a little hack: The globalPose is used to set the "local" position of the attached Visualization:
		// Since the attached visualizations are already positioned at the global pose of the visualizationModel, 
		// we just need the local postJointTransform
		if (visualizationNode)
		{
			visualizationNode->setGlobalPose(postJointTransformation);
			visualizationModel->attachVisualization("CoordinateSystem",visualizationNode);
		}
	}
}

void RobotNode::showStructure( bool enable, const std::string &visualizationType)
{
	ReadLock lock(this->mutex,use_mutex);
	if (!enable && !visualizationModel)
		return; // nothing to do

	if (!ensureVisualization(visualizationType))
		return;
	std::string attachName1("RobotNodeStructurePre");
	std::string attachName2("RobotNodeStructureJoint");
	std::string attachName3("RobotNodeStructurePost");
	visualizationModel->detachVisualization(attachName1);
	visualizationModel->detachVisualization(attachName2);
	visualizationModel->detachVisualization(attachName3);
	if (enable)
	{
		VisualizationFactoryPtr visualizationFactory;
		if (visualizationType.empty())
			visualizationFactory = VisualizationFactory::first(NULL);
		else
			visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
		if (!visualizationFactory)
		{
			VR_WARNING << "No visualization factory for name " << visualizationType << endl;
			return;
		}

		// create visu
		Eigen::Matrix4f i = Eigen::Matrix4f::Identity();

		if (!preJointTransformation.isIdentity())
		{
			VisualizationNodePtr visualizationNode1 = visualizationFactory->createLine(preJointTransformation.inverse(),i);
			if (visualizationNode1)
				visualizationModel->attachVisualization(attachName1,visualizationNode1);
		}
		VisualizationNodePtr visualizationNode2 = visualizationFactory->createSphere(5.0f);
		if (visualizationNode2)
			visualizationModel->attachVisualization(attachName2,visualizationNode2);
		if (!postJointTransformation.isIdentity())
		{
			VisualizationNodePtr visualizationNode3 = visualizationFactory->createLine(i,postJointTransformation,3);
			if (visualizationNode3)
				visualizationModel->attachVisualization(attachName3,visualizationNode3);
		}
	}
}

std::vector<RobotNodePtr> RobotNode::getAllParents( RobotNodeSetPtr rns )
{
	std::vector<RobotNodePtr> result;
	if (!rns)
		return result;
	std::vector<RobotNodePtr> rn = rns->getAllRobotNodes();

	for (unsigned int i=0; i<rn.size(); i++)
	{
		if (rn[i]->hasChildNode(shared_from_this(),true))
			result.push_back(rn[i]);
	}
	return result;
}


VirtualRobot::RobotNodePtr RobotNode::getParent()
{
	RobotNodePtr p = parent.lock();
	return p;
}

void RobotNode::setJointLimits( float lo, float hi )
{
	WriteLock lock(mutex,use_mutex);
	jointLimitLo = lo;
	jointLimitHi = hi;
}

void RobotNode::setMaxTorque( float maxTo )
{
	maxTorque = maxTo;
}

void RobotNode::setMaxAcceleration( float maxAcc )
{
	maxAcceleration = maxAcc;
}

void RobotNode::setMaxVelocity( float maxVel )
{
	maxVelocity = maxVel;
}

float RobotNode::getMaxVelocity()
{
	return maxVelocity;
}

float RobotNode::getMaxAcceleration()
{
	return maxAcceleration;
}

float RobotNode::getMaxTorque()
{
	return maxTorque;
}


} // namespace VirtualRobot
