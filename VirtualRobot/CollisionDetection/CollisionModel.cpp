
#include "CollisionModel.h"
#include "CollisionChecker.h"
#include "../Visualization/TriMeshModel.h"
#include "../Visualization/VisualizationNode.h"
#include <algorithm>

namespace VirtualRobot {

CollisionModel::CollisionModel(VisualizationNodePtr visu, const std::string &name, CollisionCheckerPtr colChecker, int id)
{
	globalPose = Eigen::Matrix4f::Identity();
	this->id = id;

	this->name = name;

	this->colChecker = colChecker;
	if (!this->colChecker)
		this->colChecker = CollisionChecker::getGlobalCollisionChecker();

	if (!this->colChecker)
	{
		VR_WARNING << "no col checker..." << endl;
	}

	TriMeshModelPtr model;
	visualization = visu;
	updateVisualization = true;
	if (visualization)
	{
		model = visualization->getTriMeshModel();
		if (model)
			bbox = model->boundingBox;
	}
#if defined(VR_COLLISION_DETECTION_PQP)
	collisionModelImplementation.reset(new CollisionModelPQP(model, colChecker,id));
#else
	collisionModelImplementation.reset(new CollisionModelDummy(model, colChecker,id));
#endif
}


CollisionModel::~CollisionModel()
{
	destroyData();
}


void CollisionModel::destroyData()
{
	if (collisionModelImplementation)
		collisionModelImplementation->destroyData();
}


std::string CollisionModel::getName()
{
	return name;
}

void CollisionModel::setGlobalPose(const Eigen::Matrix4f &m)
{
	globalPose = m;
	collisionModelImplementation->setGlobalPose(m);
	if (visualization && updateVisualization)
		visualization->setGlobalPose(m);
}

VirtualRobot::CollisionModelPtr CollisionModel::clone(CollisionCheckerPtr colChecker)
{
	VisualizationNodePtr visuNew;
	if (visualization)
		visuNew = visualization->clone();

	std::string nameNew = name;
	int idNew = id;

	CollisionModelPtr p(new CollisionModel(visuNew,nameNew,colChecker,idNew));
	p->setGlobalPose(getGlobalPose());
	p->setUpdateVisualization(getUpdateVisualizationStatus());
	return p;
}

int CollisionModel::getId()
{
	return id;
}

void CollisionModel::setUpdateVisualization( bool enable )
{
	updateVisualization = enable;
}

bool CollisionModel::getUpdateVisualizationStatus()
{
	return updateVisualization;
}

VisualizationNodePtr CollisionModel::getVisualization()
{
	return visualization;
}

void CollisionModel::print()
{
	collisionModelImplementation->print();
	if (visualization)
	{
		visualization->print();
	}
}

int CollisionModel::getNumFaces()
{
	if (!visualization)
		return 0;
	return visualization->getNumFaces();

}

std::vector< Eigen::Vector3f > CollisionModel::getModelVeticesGlobal()
{
	std::vector< Eigen::Vector3f > result;
	TriMeshModelPtr model = collisionModelImplementation->getTriMeshModel();
	if (!model)
		return result;

	Eigen::Matrix4f t;
	t.setIdentity();

	for (std::vector<Eigen::Vector3f >::iterator i=model->vertices.begin(); i!=model->vertices.end(); i++)
	{
		t.block(0,3,3,1)=*i;
		t = globalPose * t;
		result.push_back(t.block(0,3,3,1));
	}
	return result;
}

BoundingBox CollisionModel::getBoundingBox( bool global /*= true*/ )
{
	if (global)
	{

		std::vector<Eigen::Vector3f> pts = bbox.getPoints();
		for (size_t i=0;i<pts.size();i++)
			pts[i] = MathTools::transformPosition(pts[i],globalPose);

		BoundingBox result(pts);
		return result;
	}
	return bbox;
}

VirtualRobot::VisualizationNodePtr CollisionModel::getModelDataVisualization()
{
	if (!modelVisualization && visualization)
	{
		TriMeshModelPtr model = collisionModelImplementation->getTriMeshModel();
		if (model)
		{
			std::string type = visualization->getType();
			VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(type, NULL);
			if (visualizationFactory)
				modelVisualization = visualizationFactory->createTriMeshModelVisualization(model, true, globalPose);
		}
	}
	return modelVisualization;
}



/*
void CollisionModel::GetAABB( SbBox3f& store_aabb )
{
	if (!m_pIVModel)
		return;
	SbViewportRegion vpreg;
	SoGetBoundingBoxAction bboxAction(vpreg);
	store_aabb.makeEmpty();
	bboxAction.apply(m_pIVModel);
	store_aabb.extendBy(bboxAction.getBoundingBox());
}

void CollisionModel::GetOOBB(SbXfBox3f& store_oobb)
{
	if (!m_pIVModel)
		return;
	SbViewportRegion vpreg;
	SoGetBoundingBoxAction bboxAction(vpreg);
	bboxAction.apply(m_pIVModel);
	store_oobb = bboxAction.getXfBoundingBox();
}
*/
/*
int CollisionModel::BuildColModel(std::map<SoNode*,int> &mIVIDMapping)
{
	if (!colChecker)
		return 0;
	if (m_pIVModel)
	{
		m_pIVModel->unref();
	}
	m_pIVModel = new SoSeparator();
	m_pIVModel->ref();
	SoMatrixTransform *pMaTr = new SoMatrixTransform();
	m_pIVModel->addChild(pMaTr);

	SoSeparator *pIVModel = new SoSeparator();
	int nodeCount = 0;
	//int idNr = s_nID;
	m_vCollisionModelIds.clear();
	int trNr = collisionModelImplementation->BuildColModel(mIVIDMapping,m_vCollisionModelIds,pIVModel);
	m_pIVModel->addChild(pIVModel);

	SbMatrix mat;
	mat.makeIdentity();
	SetGlobalPose(mat);

	return trNr;
}
*/

} // namespace VirtualRobot

