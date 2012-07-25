
#include "SimoxMotionState.h"
#include "BulletEngine.h"


using namespace VirtualRobot;

namespace SimDynamics
{


SimoxMotionState::SimoxMotionState( VirtualRobot::SceneObjectPtr sceneObject)
{
	this->sceneObject = sceneObject;
	initalGlobalPose.setIdentity();
	if (sceneObject)
	{
		initalGlobalPose = sceneObject->getGlobalPoseVisualization();
	}
    _transform.setIdentity();
	_graphicsTransfrom.setIdentity();
	_comOffset.setIdentity();
	Eigen::Vector3f com = sceneObject->getCoMLocal();
	RobotNodePtr rn = boost::dynamic_pointer_cast<RobotNode>(sceneObject);
	if (rn)
	{
		// we are operating on a RobotNode, so we need a RobtoNodeActuator to move it later on
		robotNodeActuator.reset(new RobotNodeActuator(rn));

		// localcom is given in coord sytsem of robotnode (== endpoint of internal transformation)
		// we need com in visualization coord system (== without postjointtransform)

		Eigen::Matrix4f t;
		t.setIdentity();
		t.block(0,3,3,1)=rn->getCoMGlobal();
		t = rn->getGlobalPoseVisualization().inverse() * t;
		com = t.block(0,3,3,1);
	}
	_setCOM(com);
	setGlobalPose(initalGlobalPose);
}

SimoxMotionState::~SimoxMotionState()
{

}

void SimoxMotionState::setWorldTransform(const btTransform& worldPose)
{
    // Check callbacks
    if( callbacks.size() > 0 )
    {
        std::vector<SimoxMotionStateCallback*>::iterator it;
        for( it = callbacks.begin(); it != callbacks.end(); it++ )
                (*it)->poseChanged( worldPose );
    }

    // _transform is the Bullet pose, used in getWorldTransform().
    _transform = worldPose; // com position
	m_graphicsWorldTrans = _transform; // this is used for debug drawing
	_graphicsTransfrom = _transform;
	//_graphicsTransfrom.getOrigin();// -= _comOffset.getOrigin(); // com adjusted
	setGlobalPoseSimox( BulletEngine::getPoseEigen(_graphicsTransfrom) );
}

void SimoxMotionState::getWorldTransform(btTransform& worldTrans ) const
{
    worldTrans = _transform;
}

void SimoxMotionState::setGlobalPoseSimox( const Eigen::Matrix4f& worldPose )
{
	if (!sceneObject)
		return;


	// worldPose -> local visualization frame
	Eigen::Matrix4f localPose = sceneObject->getGlobalPoseVisualization().inverse() * worldPose;

	// com as matrix4f
	/*Eigen::Matrix4f comLocal;
	comLocal.setIdentity();
	comLocal.block(0,3,3,1) = -com;*/

	// apply com
	//Eigen::Matrix4f localPoseAdjusted =  localPose * comLocal;
	Eigen::Matrix4f localPoseAdjusted =  localPose;
	localPoseAdjusted.block(0,3,3,1) -= com;

	Eigen::Matrix4f resPose = sceneObject->getGlobalPoseVisualization() * localPoseAdjusted;

	//Eigen::Matrix4f resPose = worldPose;
	// assuming we get the com adjusted pose
	//Eigen::Matrix4f gp = BulletEngine::getPoseEigen(worldPose);
	// Determine pose of simox model
	/*
	Eigen::Matrix4f comLocal = Eigen::Matrix4f::Identity();
	comLocal.block(0,3,3,1) = -com;
    Eigen::Matrix4f resPose = comLocal * gp;*/
	

	if (robotNodeActuator)
	{
		// we assume that all models are handled by Bullet, so we do not need to update children
		robotNodeActuator->updateVisualizationPose(resPose,false); 
	} else
	{
		sceneObject->setGlobalPose(resPose);
	}
}

void SimoxMotionState::setGlobalPose( const Eigen::Matrix4f &pose )
{
    initalGlobalPose = pose;
	/* conervt to local coord system, apply comoffset and convert back*/
	Eigen::Matrix4f poseLocal = sceneObject->getGlobalPoseVisualization().inverse() * pose;
	poseLocal.block(0,3,3,1) += com;
	Eigen::Matrix4f poseGlobal = sceneObject->getGlobalPoseVisualization() * poseLocal;
	m_startWorldTrans = BulletEngine::getPoseBullet(poseGlobal);
	//m_startWorldTrans.getOrigin() -= _comOffset.getOrigin();
    updateTransform();  
}



std::vector<SimoxMotionStateCallback*> SimoxMotionState::getCallbacks()
{
    return callbacks;
}

void SimoxMotionState::_setCOM( const Eigen::Vector3f& com )
{
	this->com = com;
	Eigen::Matrix4f comM = Eigen::Matrix4f::Identity();
	comM.block(0,3,3,1) = -com;
	_comOffset = BulletEngine::getPoseBullet(comM);
}

void SimoxMotionState::setCOM( const Eigen::Vector3f& com )
{
	_setCOM(com);
	updateTransform();
}

Eigen::Vector3f SimoxMotionState::getCOM() const
{
	return com;
}

void SimoxMotionState::updateTransform()
{
	//Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
	//m.block(0,3,3,1) = com;
    setWorldTransform(m_startWorldTrans);
}

}
