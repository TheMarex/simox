
#include "SceneObject.h"
#include "CollisionDetection/CollisionModel.h"
#include "CollisionDetection/CollisionChecker.h"
#include "Visualization/TriMeshModel.h"
#include "Visualization/VisualizationFactory.h"
#include "Visualization/Visualization.h"
#include "VirtualRobotException.h"
#include "Robot.h"
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

namespace VirtualRobot {


SceneObject::SceneObject( const std::string &name, VisualizationNodePtr visualization /*= VisualizationNodePtr()*/, CollisionModelPtr collisionModel /*= CollisionModelPtr()*/, const Physics &p /*= Physics()*/, CollisionCheckerPtr colChecker /*= CollisionCheckerPtr()*/ )
{
	this->name = name;
	this->visualizationModel = visualization;
	this->collisionModel = collisionModel;
	
	this->physics = p;
	this->globalPose = Eigen::Matrix4f::Identity();
	this->initialized = false;
	updateVisualization = true;
	if (visualization)
		updateVisualization = visualization->getUpdateVisualizationStatus();
	if (!colChecker)
	{
		this->collisionChecker = CollisionChecker::getGlobalCollisionChecker();
	} else
	{
		this->collisionChecker = colChecker;
	}
	setGlobalPose(Eigen::Matrix4f::Identity());
}


SceneObject::~SceneObject()
{
	for (size_t i=0;i<children.size();i++)
		children[i]->detachedFromParent();
}


bool SceneObject::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr> &children)
{
	// parent
	if (parent)
	{
		if (!parent->hasChild(shared_from_this()))
			parent->attachChild(shared_from_this());
	} 
	this->parent = parent;

	// children
	for (size_t i=0;i<children.size();i++)
	{
		if (!this->hasChild(children[i]))
			this->attachChild(children[i]);
	}
	// init all children
	std::vector<SceneObjectPtr> c = getChildren();
	for (size_t i=0;i<c.size();i++)
	{
		c[i]->initialize(shared_from_this());
	}

	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
	if (parent)
		m = parent->getGlobalPose();

	initialized = true;
	updatePose(m);
	return initializePhysics();	
}

void SceneObject::setGlobalPose( const Eigen::Matrix4f &pose )
{
	SceneObjectPtr p = getParent();
	if (p)
	{
		VR_ERROR << "Could not set pose of <" << name << ">. The object is attached to <" << p->getName() << ">" << endl;
		return;
	}
	updatePose(pose);
}


void SceneObject::updatePose( bool updateChildren)
{
	Eigen::Matrix4f gp = getGlobalPoseVisualization();
	if (visualizationModel)
		visualizationModel->setGlobalPose(gp);
	if (collisionModel)
		collisionModel->setGlobalPose(gp);
	if (updateChildren)
		for (size_t i=0;i<children.size();i++)
		{
			children[i]->updatePose(getGlobalPose());
		}
}

void SceneObject::updatePose( const Eigen::Matrix4f &parentPose, bool updateChildren )
{
	globalPose = parentPose;
	updatePose(updateChildren);
}

std::string SceneObject::getName() const
{
	return name;
}

VirtualRobot::CollisionModelPtr SceneObject::getCollisionModel()
{
	return collisionModel;
}

VirtualRobot::CollisionCheckerPtr SceneObject::getCollisionChecker()
{
	return collisionChecker;
}

VirtualRobot::VisualizationNodePtr SceneObject::getVisualization(SceneObject::VisualizationType visuType)
{
	if (visuType == SceneObject::Full)
		return visualizationModel;
	else 
	{
		if (collisionModel)
		{
			if (visuType == SceneObject::Collision)
				return collisionModel->getVisualization();
			else
				return collisionModel->getModelDataVisualization();
		}
		else
		{
			//VR_WARNING << "<" << name << "> No collision model present ..." << endl;
			return VisualizationNodePtr();
		}
	}
}

bool SceneObject::ensureVisualization(const std::string &visualizationType)
{
	if (visualizationModel)
		return true;


	VisualizationFactoryPtr visualizationFactory;
	if (visualizationType=="")
	{
		visualizationFactory = VisualizationFactory::first(NULL);
	} else
	{
		visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
	}

	if (!visualizationFactory)
	{
		VR_WARNING << "VisualizationFactory of type '" << visualizationType << "' not present. Could not create visualization data in Robot Node <" << name << ">" << endl;
		return false;
	}

	// create dummy visualization
	setVisualization(visualizationFactory->createVisualization());
	return true;
}

void SceneObject::showCoordinateSystem( bool enable, float scaling, std::string *text)
{
	if (!enable && !visualizationModel)
		return; // nothing to do

	if (!ensureVisualization())
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
		VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
		if (!visualizationFactory)
		{
			VR_ERROR << " Could not determine a valid VisualizationFactory!!" << endl;
			return;
		}
		// create coord visu
		VisualizationNodePtr visualizationNode = visualizationFactory->createCoordSystem(scaling,&coordName);
		visualizationModel->attachVisualization("CoordinateSystem",visualizationNode);
	}
}

void SceneObject::showPhysicsInformation( bool enableCoM, bool enableInertial, VisualizationNodePtr comModel)
{
	if (!enableCoM && !enableInertial && !visualizationModel)
		return; // nothing to do

	if (!visualizationModel)
		return;

	if (physics.massKg<=0)
	{	
		// no valid physics information
		// check for valid model
		TriMeshModelPtr tm = visualizationModel->getTriMeshModel();
		if (!tm || tm->vertices.size()<2)
			return;
	}

	if (visualizationModel->hasAttachedVisualization("__CoMLocation"))
	{
		visualizationModel->detachVisualization("__CoMLocation");
	}
	if (visualizationModel->hasAttachedVisualization("__InertiaMatrix"))
	{
		visualizationModel->detachVisualization("__InertiaMatrix");
	}
	VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);

	if (enableCoM && visualizationFactory)
	{
		// create visu
		if (!comModel)
		{
			VisualizationNodePtr comModel1 = visualizationFactory->createSphere(7.05f,1.0f,0.2f,0.2f);
			VisualizationNodePtr comModel2 = visualizationFactory->createBox(10.0f,10.0f,10.0f,0.2f,0.2f,1.0f);
			std::vector<VisualizationNodePtr> v;
			v.push_back(comModel1);
			v.push_back(comModel2);
			comModel = visualizationFactory->createUnitedVisualization(v);
		}

		if (comModel)
		{
			std::stringstream ss;
			ss << "COM:" << getName();
			std::string t = ss.str();
			VisualizationNodePtr vText = visualizationFactory->createText(t,true,1.0f,VisualizationFactory::Color::Blue(),0,10.0f,0);
			std::vector<VisualizationNodePtr> v;
			v.push_back(comModel);
			v.push_back(vText);
			comModel = visualizationFactory->createUnitedVisualization(v);
		}
		VisualizationNodePtr comModelClone = comModel->clone();
		Eigen::Vector3f l = getCoMLocal() * 0.001f;
		cout << "COM:" << l << endl;
		Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
		m.block(0,3,3,1) = l;
		visualizationFactory->applyDisplacement(comModelClone, m);

		visualizationModel->attachVisualization("__CoMLocation",comModelClone);
	}
	if (enableInertial && visualizationFactory)
	{
		VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
		if (!visualizationFactory)
		{
			VR_ERROR << " Could not determine a valid VisualizationFactory!!" << endl;
			return;
		}
		// create inertia visu
		cout << "INERTIA MATRIX:" << endl << physics.intertiaMatrix << endl;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(physics.intertiaMatrix);
		if (eigensolver.info() == Eigen::Success) 
		{
			/*cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;
			cout << "Here's a matrix whose columns are eigenvectors of A \n"
				<< "corresponding to these eigenvalues:\n"
				<< eigensolver.eigenvectors() << endl;*/
			Eigen::Vector3f v1 = eigensolver.eigenvectors().block(0,0,3,1);
			Eigen::Vector3f v2 = eigensolver.eigenvectors().block(0,1,3,1);
			Eigen::Vector3f v3 = eigensolver.eigenvectors().block(0,2,3,1);
			/*cout << "v1:" << v1 << endl;
			cout << "v2:" << v2 << endl;
			cout << "v3:" << v3 << endl;*/

			float xl = (float)(eigensolver.eigenvalues().rows()>0?eigensolver.eigenvalues()(0):1e-6);
			float yl = (float)(eigensolver.eigenvalues().rows()>1?eigensolver.eigenvalues()(1):1e-6);
			float zl = (float)(eigensolver.eigenvalues().rows()>2?eigensolver.eigenvalues()(2):1e-6);
			float le = sqrtf(xl*xl + yl*yl + zl*zl);
			float maxSize = le>1e-8?50.0f/le:5000.0f;

			VisualizationNodePtr inertiaVisu = visualizationFactory->createEllipse(xl*maxSize,yl*maxSize,zl*maxSize,true);

			Eigen::Vector3f l = getCoMLocal() * 0.001f;
			//cout << "COM:" << l << endl;
			Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
			m.block(0,3,3,1) = l;
			m.block(0,0,3,3) = eigensolver.eigenvectors().block(0,0,3,3); // rotate according to EV
			visualizationFactory->applyDisplacement(inertiaVisu, m);

			visualizationModel->attachVisualization("__InertiaMatrix",inertiaVisu);

		} else
		{
			VR_INFO << " Could not determine eigenvectors of inertia matrix" << endl;
		}
	}
}

bool SceneObject::showCoordinateSystemState()
{
	if (visualizationModel)
		return visualizationModel->hasAttachedVisualization("CoordinateSystem");
	else
		return false;
}

void SceneObject::setUpdateVisualization( bool enable )
{
	updateVisualization = enable;
	if (visualizationModel)
		visualizationModel->setUpdateVisualization(enable);
	if (collisionModel)
		collisionModel->setUpdateVisualization(enable);
}

bool SceneObject::getUpdateVisualizationStatus()
{
	return updateVisualization;
}

void SceneObject::setVisualization(VisualizationNodePtr visualization)
{
	visualizationModel = visualization;
	if (visualizationModel)
	{
		visualizationModel->setUpdateVisualization(updateVisualization);
		visualizationModel->setGlobalPose(globalPose);
	}
}

void SceneObject::setCollisionModel(CollisionModelPtr colModel)
{
	collisionModel = colModel;
	if (collisionModel)
	{
		collisionModel->setUpdateVisualization(updateVisualization);
		collisionModel->setGlobalPose(globalPose);
	}
}


Eigen::Matrix4f SceneObject::toLocalCoordinateSystem(const Eigen::Matrix4f &poseGlobal) const
{
	return getGlobalPose().inverse() * poseGlobal;
}


Eigen::Matrix4f SceneObject::toGlobalCoordinateSystem(const Eigen::Matrix4f &poseLocal) const
{
	return getGlobalPose() * poseLocal;
}

Eigen::Vector3f SceneObject::toLocalCoordinateSystemVec(const Eigen::Vector3f &positionGlobal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0,3,3,1)=positionGlobal;
	t = toLocalCoordinateSystem(t);
	Eigen::Vector3f result = t.block(0,3,3,1);
	return result;
}


Eigen::Vector3f SceneObject::toGlobalCoordinateSystemVec(const Eigen::Vector3f &positionLocal) const
{
	Eigen::Matrix4f t;
	t.setIdentity();
	t.block(0,3,3,1)=positionLocal;
	t = toGlobalCoordinateSystem(t);
	Eigen::Vector3f result = t.block(0,3,3,1);
	return result;
}

Eigen::Matrix4f SceneObject::getTransformationTo(const SceneObjectPtr otherObject)
{
    return getGlobalPose().inverse() * otherObject->getGlobalPose();
}

Eigen::Matrix4f SceneObject::getTransformationFrom(const SceneObjectPtr otherObject)
{
    return otherObject->getGlobalPose().inverse() * getGlobalPose();
}

Eigen::Matrix4f SceneObject::transformTo(const SceneObjectPtr otherObject, const Eigen::Matrix4f &poseInOtherCoordSystem)
{
        Eigen::Matrix4f m = getTransformationTo(otherObject);
        return m * poseInOtherCoordSystem;
}

Eigen::Vector3f SceneObject::transformTo(const SceneObjectPtr otherObject, const Eigen::Vector3f &positionInOtherCoordSystem)
{
        Eigen::Matrix4f m = getTransformationTo(otherObject);
        Eigen::Vector4f tmp4 = Eigen::Vector4f::Zero();
        tmp4.segment(0,3) = positionInOtherCoordSystem;
        Eigen::Vector4f res = m * tmp4;
        Eigen::Vector3f res3(res[0],res[1],res[2]);
        return res3;
}

void SceneObject::setupVisualization( bool showVisualization, bool showAttachedVisualizations )
{
	if (visualizationModel)
		visualizationModel->setupVisualization(showVisualization,showAttachedVisualizations);

}

int SceneObject::getNumFaces(bool collisionModel /*=false*/)
{
	if (collisionModel)
	{
		if (this->collisionModel)
			return this->collisionModel->getNumFaces();
		else
			return 0;
	} else 
	{
		if (visualizationModel)
			return visualizationModel->getNumFaces();
		else
			return 0;
	}
}

Eigen::Matrix4f SceneObject::getGlobalPose() const
{
	 return globalPose;
}

Eigen::Matrix4f SceneObject::getGlobalPoseVisualization() const
{
	 return globalPose;
}

Eigen::Vector3f SceneObject::getCoMLocal()
{
	return physics.localCoM;
}

Eigen::Vector3f SceneObject::getCoMGlobal()
{
	Eigen::Vector3f result = getCoMLocal();
	return toGlobalCoordinateSystemVec(result);
}

float SceneObject::getMass()
{
	return physics.massKg;
}

bool SceneObject::initializePhysics()
{
		// check if physics node's CoM location has to be calculated
	if (physics.comLocation == SceneObject::Physics::eVisuBBoxCenter)
	{
		/*if (!visualizationModel && !collisionModel)
		{
			VR_WARNING << getName() << ": Physics tag CoM is set to eVisuBBoxCenter, but no visualization model is loaded, setting CoM to local position (0/0/0)" << endl;
		} else*/
		if (visualizationModel || collisionModel)
		{
			TriMeshModelPtr tm;
			// since globalPose and visualization's global pose may differ, we transform com to local coord system (globalpose)
			Eigen::Matrix4f posVisu;
			if (collisionModel)
			{
				tm = collisionModel->getTriMeshModel();
				posVisu = collisionModel->getGlobalPose();
			} else
			{
				tm = visualizationModel->getTriMeshModel();
				posVisu = visualizationModel->getGlobalPose();
			} 
			if (!tm)
			{
				VR_WARNING << "Could not create trimeshmodel for CoM computation, setting CoM to local position (0/0/0)" << endl;
			} else
			{
				Eigen::Vector3f minS,maxS;
				tm->getSize( minS,maxS );
				physics.localCoM = minS + (maxS - minS) * 0.5f;	



				// trimeshmodel is without any pose transformations, so apply visu global pose
				Eigen::Matrix4f visuComGlobal;
				visuComGlobal.setIdentity();
				visuComGlobal.block(0,3,3,1)=physics.localCoM;
				visuComGlobal = posVisu * visuComGlobal;

				// transform to this object's global pose (this might be a different one, e.g. when postJointTransformations are considered)
				Eigen::Matrix4f comLocal = toLocalCoordinateSystem(visuComGlobal);
				physics.localCoM = comLocal.block(0,3,3,1);
			}
		}

	}
	return true;
}

void SceneObject::print( bool printChildren, bool printDecoration /*= true*/ ) const
{
	if (printDecoration)
		cout << "**** SceneObject ****" << endl;

	cout << " * Name: " << name << endl;
	cout << " * GlobalPose: " << endl << globalPose << endl;
	cout << " * Mass: ";
	if (physics.massKg<=0)
		cout << "<not set>" << endl;
	else 
		cout << physics.massKg << " kg" << endl;
	cout << " * CoM:" << physics.localCoM(0) << ", " << physics.localCoM(1) << ", " << physics.localCoM(2) << endl;
	cout << " * Visualization:" << endl;
	if (visualizationModel)
		visualizationModel->print();
	else
		cout << "<not set>" << endl;
	cout << " * Update visualization status: ";
	if (updateVisualization)
		cout << "enabled" << endl;
	else
		cout << "disabled" << endl;
	cout << " * Collision Model:" << endl;
	if (collisionModel)
		collisionModel->print();
	else
		cout << "<not set>" << endl;
	std::vector<SceneObjectPtr> children = this->getChildren();
	if (children.size()>0)
	{
		cout << " * Children:" << endl;
		for (size_t i=0;i<children.size();i++)
			cout << " ** " << children[i]->getName();
	}
	if (printDecoration)
		cout << endl;

	if (printChildren)
	{
		for (unsigned int i = 0; i < children.size(); i++)
			children[i]->print(true, true);
	}
}

void SceneObject::showBoundingBox( bool enable, bool wireframe )
{
	if (!enable && !visualizationModel)
		return; // nothing to do

	if (!ensureVisualization())
		return;


	if (visualizationModel->hasAttachedVisualization("BoundingBox"))
	{
		visualizationModel->detachVisualization("BoundingBox");
	}
	if (enable)
	{
		VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);
		if (!visualizationFactory)
		{
			VR_ERROR << " Could not determine a valid VisualizationFactory!!" << endl;
			return;
		}
		// create bbox visu
		if (collisionModel)
		{
			BoundingBox bbox = collisionModel->getBoundingBox(false);
			VisualizationNodePtr visualizationNode = visualizationFactory->createBoundingBox(bbox, wireframe);
			visualizationModel->attachVisualization("BoundingBox",visualizationNode);
		}
	}
}


void SceneObject::highlight (VisualizationPtr visualization, bool enable)
{
	if (!visualization)
		return;

	if (getVisualization(Full) && visualization->isVisualizationNodeRegistered(getVisualization(Full)))
		visualization->highlight(getVisualization(Full),enable);
	if (getVisualization(Collision) && visualization->isVisualizationNodeRegistered(getVisualization(Collision)))
		visualization->highlight(getVisualization(Collision),enable);
	if (getVisualization(CollisionData) && visualization->isVisualizationNodeRegistered(getVisualization(CollisionData)))
		visualization->highlight(getVisualization(CollisionData),enable);
}

void SceneObject::setName( const std::string &name )
{
	this->name = name;
}

SceneObject* SceneObject::_clone( const std::string &name, CollisionCheckerPtr colChecker ) const
{
	VisualizationNodePtr clonedVisualizationNode;
	if (visualizationModel)
		clonedVisualizationNode = visualizationModel->clone();
	CollisionModelPtr clonedCollisionModel;
	if (collisionModel)
		clonedCollisionModel = collisionModel->clone(colChecker);

	SceneObject* result = new SceneObject(name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker);

	if (!result)
	{
		VR_ERROR << "Cloning failed.." << endl;
		return result;
	}

	result->setGlobalPose(getGlobalPose());

	return result;
}


Eigen::Matrix3f SceneObject::getInertiaMatrix()
{
	return physics.intertiaMatrix;
}

std::string SceneObject::getSceneObjectXMLString(const std::string &basePath, int tabs)
{
	std::stringstream ss;
	std::string t = "\t";
	std::string pre = "";
	for (int i=0;i<tabs;i++)
		pre += "\t";

	if (visualizationModel)
	{
		ss << visualizationModel->getXMLString(basePath,tabs);
	}

	if (collisionModel && collisionModel->getVisualization())
	{
		ss << collisionModel->getXMLString(basePath,tabs);
	}
	Eigen::Matrix4f gp = getGlobalPose();
	if (!gp.isIdentity())
	{
		ss << pre << "<GlobalPose>\n";
		ss << pre << "\t<Transform>\n";
		ss << MathTools::getTransformXMLString(gp,tabs+2);
		ss << pre << "\t</Transform>\n";
		ss << pre << "</GlobalPose>\n";
	}
	if (physics.isSet())
	{
		ss << physics.getXMLString(tabs);
	}

	return ss.str();
}

void SceneObject::setMass( float m )
{
	physics.massKg = m;
}

void SceneObject::setInertiaMatrix( const Eigen::Matrix3f &im )
{
	physics.intertiaMatrix = im;
}

bool SceneObject::hasChild( SceneObjectPtr child, bool recursive ) const
{
	VR_ASSERT(child);
	for (size_t i=0;i<children.size();i++)
	{
		if (children[i] == child)
			return true;
		if (recursive && children[i]->hasChild(child,true))
			return true;
	}
	return false;
}

bool SceneObject::hasChild(const std::string &childName, bool recursive ) const
{
	for (size_t i=0;i<children.size();i++)
	{
		if (children[i]->getName() == childName)
			return true;
		if (recursive && children[i]->hasChild(childName,true))
			return true;
	}
	return false;
}

void SceneObject::detachChild( SceneObjectPtr child )
{
	VR_ASSERT(child);
	if (!hasChild(child))
	{
		VR_WARNING << " Trying to detach a not attached object: " << getName() << "->" << child->getName() << endl;
		return;
	}
	children.erase(std::find(children.begin(), children.end(), child));
	child->detachedFromParent();
}

bool SceneObject::attachChild( SceneObjectPtr child )
{
	VR_ASSERT(child);
	if (hasChild(child))
	{
		VR_WARNING << " Trying to attach already attached object: " << getName() << "->" << child->getName() << endl;
		return true; // no error
	}
	if (child->hasParent())
	{
		VR_WARNING << " Trying to attach object that has already a parent: " << getName() << "->" << child->getName() << ", child's parent:" << child->getParent()->getName() << endl;
		return false;
	}
	if (hasChild(child->getName()))
	{
		VR_ERROR << " Trying to attach object with name: " << child->getName() << " to " << getName() << ", but a child with same name is already present!" << endl;
		return false;
	}
	children.push_back(child);
	child->attached(shared_from_this());
	return true;
}

bool SceneObject::hasParent()
{
	return (getParent());
}

VirtualRobot::SceneObjectPtr SceneObject::getParent() const
{
	SceneObjectPtr p = parent.lock();
	return p;
}

void SceneObject::detachedFromParent( )
{
	this->parent.reset();
}

void SceneObject::attached( SceneObjectPtr parent )
{
	VR_ASSERT(parent);
	SceneObjectPtr p = getParent();
	THROW_VR_EXCEPTION_IF( p && p!=parent ,"SceneObject already attached to a different parent");
	this->parent = parent;
}



std::vector<std::string> SceneObject::getIgnoredCollisionModels()
{
    return physics.ignoreCollisions;
}

} // namespace
