
#include "RobotNode.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Visualization/Visualization.h"
#include "../Visualization/TriMeshModel.h"
#include <cmath>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>

#include <boost/math/special_functions/fpclassify.hpp>

#include <Eigen/Core>

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
						: SceneObject(name,visualization,collisionModel,p,colChecker)


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
	// not needed here
	// when robot is destroyed all references to this RobotNode are also destroyed
	//RobotPtr rob = robot.lock();
	//if (rob)
	//	rob->deregisterRobotNode(shared_from_this());
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


RobotPtr RobotNode::getRobot() const
{
	RobotPtr result(robot);
	return result;
}

void RobotNode::setJointValue(float q)
{
	RobotPtr r = getRobot();
	VR_ASSERT(r);
	WriteLockPtr lock = r->getWriteLock();
	setJointValueNoUpdate(q);
	applyJointValue();
}

void RobotNode::setJointValueNoUpdate(float q)
{
	VR_ASSERT_MESSAGE( initialized, "Not initialized");
	VR_ASSERT_MESSAGE( (!boost::math::isnan(q) && !boost::math::isinf(q)) ,"Not a valid number...");
	
	//std::cout << "######## Setting Joint to: " << q << " degrees" << std::endl;

	if (q < jointLimitLo)
	{
		q = jointLimitLo;
	}
	if (q > jointLimitHi)
	{
		q = jointLimitHi;
	}
	jointValue = q;
}

void RobotNode::updateTransformationMatrices()
{

}
void RobotNode::updateTransformationMatrices(const Eigen::Matrix4f &globalPose)
{

}

void RobotNode::applyJointValue()
{
	THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

	updateTransformationMatrices();
	
	std::vector< RobotNodePtr > children = this->getChildren(); 
	for (std::vector< RobotNodePtr >::iterator i = children.begin(); i!= children.end(); i++ )
		(*i)->applyJointValue();
}

void RobotNode::applyJointValue(const Eigen::Matrix4f &globalPose)
{
	THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

	updateTransformationMatrices(globalPose);

	std::vector< RobotNodePtr > children = this->getChildren(); 
	for (std::vector< RobotNodePtr >::iterator i = children.begin(); i!= children.end(); i++ )
		(*i)->applyJointValue();
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
	ReadLockPtr lock = getRobot()->getReadLock();
	return jointValue;
}

void RobotNode::respectJointLimits( float &jointValue ) const
{
	if (jointValue < jointLimitLo)
		jointValue = jointLimitLo;
	if (jointValue > jointLimitHi)
		jointValue = jointLimitHi;
}

bool RobotNode::checkJointLimits( float jointValue, bool verbose ) const
{
	ReadLockPtr lock = getRobot()->getReadLock();
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

void RobotNode::print( bool printChildren, bool printDecoration ) const
{
	ReadLockPtr lock = getRobot()->getReadLock();
	if (printDecoration)
		cout << "******** RobotNode ********" << endl;
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
	cout << "* visualization model: " <<endl;
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
	std::vector< RobotNodePtr > children = this->getChildren();
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
		VR_ASSERT(rob);

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
	ReadLockPtr lock = getRobot()->getReadLock();
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

	RobotNodePtr result = _clone(newRobot, clonedChildrenNames, clonedVisualizationNode, clonedCollisionModel,colChecker);

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
	ReadLockPtr lock = getRobot()->getReadLock();
	return jointLimitLo;
}

float RobotNode::getJointLimitHi()
{
	ReadLockPtr lock = getRobot()->getReadLock();
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
	ReadLockPtr lock = getRobot()->getReadLock();
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

void RobotNode::updateVisualizationPose( const Eigen::Matrix4f &globalPose, float jointValue, bool updateChildren )
{
	updateVisualizationPose(globalPose,updateChildren);
	this->jointValue = jointValue;
}
void RobotNode::updateVisualizationPose( const Eigen::Matrix4f &globalPose, bool updateChildren )
{
	// check if we are a root node
	if (!getParent())
	{
		RobotPtr rob = getRobot();
		if (rob && rob->getRootNode() == shared_from_this())
		{
			Eigen::Matrix4f gpPre = getPreJointTransformation().inverse() * globalPose;
			rob->setGlobalPose(gpPre, false);
		}
	}
	{
		this->globalPose = globalPose;// * getPreJointTransformation();
		globalPosePostJoint = this->globalPose*getPostJointTransformation();
	}
	// update collision and visualization model
	// here we do not consider the postJointTransformation, since it already defines the transformation to the next joint.
	SceneObject::setGlobalPose(this->globalPose);

	if (updateChildren)
	{
		std::vector< RobotNodePtr > children = this->getChildren(); 
		for (std::vector< RobotNodePtr >::iterator i = children.begin(); i!= children.end(); i++ )
			(*i)->applyJointValue();
	}
}

Eigen::Matrix4f RobotNode::getGlobalPoseJoint() const
{
	ReadLockPtr lock = getRobot()->getReadLock();
	return globalPose;
}

Eigen::Matrix4f RobotNode::getGlobalPoseVisualization() const
{
	ReadLockPtr lock = getRobot()->getReadLock();
	return globalPose;
}

Eigen::Matrix4f RobotNode::getGlobalPose() const
{
	ReadLockPtr lock = getRobot()->getReadLock();
	return globalPosePostJoint;
}


} // namespace VirtualRobot
