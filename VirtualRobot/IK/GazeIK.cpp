#include "GazeIK.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <algorithm>
#include <float.h>

using namespace VirtualRobot;
using namespace std;


namespace VirtualRobot
{


GazeIK::GazeIK(RobotNodeSetPtr rns, RobotNodePrismaticPtr virtualTranslationJoint) 
: rns(rns),virtualTranslationJoint(virtualTranslationJoint)
{
	nodes = rns->getAllRobotNodes();
	VR_ASSERT(nodes.size() > 0);
	VR_ASSERT(virtualTranslationJoint);
	VR_ASSERT(virtualTranslationJoint->getParent());
	VR_ASSERT(rns->hasRobotNode(virtualTranslationJoint));
	enableJLA = true;
	maxLoops = 30; // nr of seeds for gradient descent 
	maxPosError = 5.0f; //mm
    maxGradientDecentSteps = 100;
	verbose = false;
	setupIK();
}

void GazeIK::setup(float maxPosError, int maxLoops, int maxGradientDecentSteps)
{
	this->maxLoops = maxLoops;
	this->maxPosError = maxPosError;
    this->maxGradientDecentSteps = maxGradientDecentSteps;
	setupIK();
}

void GazeIK::enableJointLimitAvoidance(bool enable)
{
	enableJLA = enable;
	setupIK();
}

void GazeIK::setupIK()
{
	if (!rns || !rns->getTCP() || !virtualTranslationJoint)
		return;

	ikSolver.reset(new HierarchicalIK(rns));
	ikSolver->setVerbose(verbose);

	// 1. gaze
	ikGaze.reset(new DifferentialIK(rns , RobotNodePtr(), VirtualRobot::JacobiProvider::eSVDDamped));
	ikGaze->setVerbose(verbose);

	//Eigen::VectorXf js(rns->getSize());
	//js.setConstant(1.0f);
	//js(js.rows() - 1) = 0.1f;
	//js.normalize();
	//ikGaze->setJointScaling(js);
	//ikGaze->convertModelScalingtoM(true);


	// 2. jl avoidance
	ikJointLimits.reset();
	if (enableJLA)
		ikJointLimits.reset(new JointLimitAvoidanceJacobi(rns, VirtualRobot::JacobiProvider::eSVDDamped));
}


Eigen::VectorXf GazeIK::computeStep(Eigen::Vector3f goal, float stepSize)
{
	VR_ASSERT (ikSolver && ikGaze && virtualTranslationJoint);

	std::vector<HierarchicalIK::JacobiDefinition> jacobies;

	Eigen::Matrix4f currentPose = rns->getTCP()->getGlobalPose(); // the "tcp" of the gaze RNS -> the gaze point which is moved via a virtual translational joint
	Eigen::Matrix4f g;
	g.setIdentity();
	g.block(0,3,3,1) = goal;
	ikGaze->setGoal(g, rns->getTCP(), IKSolver::Position);
	ikGaze->setMaxPositionStep(20.0f);
	Eigen::VectorXf deltaGaze = ikGaze->getDelta(currentPose, g);
	if (verbose)
		VR_INFO << "ikGaze delta:\n" << deltaGaze.head(3) << endl;
	HierarchicalIK::JacobiDefinition jd;
	jd.jacProvider = ikGaze;
	jd.delta = deltaGaze.head(3);
	jacobies.push_back(jd);

	// 2. jl avoidance
	if (ikJointLimits)
	{
		HierarchicalIK::JacobiDefinition jd2;
		jd2.jacProvider = ikJointLimits;
		jd2.delta = ikJointLimits->getErrorVector();
		jacobies.push_back(jd2);
	}

	//compute step

	Eigen::VectorXf delta = ikSolver->computeStep(jacobies, stepSize);
    return delta;
	/*Eigen::VectorXf jv(delta.rows());
	rns->getJointValues(jv);
	if (verbose)
	{
		VR_INFO << "delta joints:\n" << delta << endl;
	}
	jv += delta;
	if (verbose)
	{
		VR_INFO << "current joint values:\n" << jv << endl;
	}
	rns->setJointValues(jv);
	return true;*/
}

bool GazeIK::solve(Eigen::Vector3f goal, float stepSize)
{
    if (!ikSolver || !ikGaze || !virtualTranslationJoint)
        return false;

    // initialize the virtualGazeJoint with a guess 
    float v = (goal - virtualTranslationJoint->getParent()->getGlobalPose().block(0, 3, 3, 1)).norm();
    virtualTranslationJoint->setJointValue(v);

    // first run: start with current joint angles
    if (trySolve(goal, stepSize))
        return true;

    // if here we failed
    for (int i=1; i<maxLoops; i++)
    {
        // set rotational joints randomly
        setJointsRandom();

        // update translational joint with initial guess
        float v = (goal - virtualTranslationJoint->getParent()->getGlobalPose().block(0, 3, 3, 1)).norm();
        virtualTranslationJoint->setJointValue(v);

        // check if there is a gradient to the solution
        if (trySolve(goal, stepSize))
            return true;
    }
	return false;
}


void GazeIK::setJointsRandom()
{
    if (!rns)
        return;
    std::vector<float> jv;
    float rn = 1.0f / (float)RAND_MAX;
    for (unsigned int i=0; i<rns->getSize(); i++)
    {
        RobotNodePtr ro =  rns->getNode(i);
        float v = 0.0f;
        if (ro->isRotationalJoint())
        {
            float r = (float)rand() * rn;
            v = ro->getJointLimitLo() + (ro->getJointLimitHi() - ro->getJointLimitLo()) * r;
        }
        jv.push_back(v);
    }
    RobotPtr rob = rns->getRobot();
    rob->setJointValues(rns,jv); 
}

float GazeIK::getCurrentError(Eigen::Vector3f goal)
{
    if (!rns)
        return 0.0f;
    Eigen::Vector3f position = goal - rns->getTCP()->getGlobalPose().block(0,3,3,1);
    return position.norm();
}

bool GazeIK::checkTolerances(Eigen::Vector3f goal)
{
    return (getCurrentError(goal) <= maxPosError);
}

void GazeIK::applyJLA(Eigen::Vector3f goal, int steps, float stepSize)
{
    float minJLAChange = 1e-6f;
    std::vector<float> jv(nodes.size(),0.0f);
    std::vector<float> jvBest = rns->getJointValues();
    int step = 0;
    while (step<steps)
    {
        Eigen::VectorXf dTheta = this->computeStep(goal,stepSize);
        for (unsigned int i=0; i<nodes.size();i++)
        {
            jv[i] = (nodes[i]->getJointValue() + dTheta[i]);

            // sanity check
            if (boost::math::isnan(jv[i]) || boost::math::isinf(jv[i]))
            {
                rns->setJointValues(jvBest);
                VR_WARNING << "Aborting, invalid joint value (nan)" << endl;
                return;
            }
        }
        rns->setJointValues(jv);
        if (!checkTolerances(goal))
        {
            // reset to last valid setup
            rns->setJointValues(jvBest);
            return;
        }
        float d = dTheta.norm();
        if (d<minJLAChange)
        {
            if (verbose)
                VR_INFO << "Could not improve result any more with joint limit avoidance tasks (dTheta.norm()=" << d << "), loop:" << step << endl;
            return;
        }
        jvBest = jv;
        step++;
    }
}

bool GazeIK::trySolve(Eigen::Vector3f goal, float stepSize)
{
    VR_ASSERT(rns);
    RobotPtr robot = rns->getRobot();
    VR_ASSERT(robot);
    bool checkImprovement = true;
    int jlaSteps = 15;
    float minumChange = 1e-5f;
    std::vector<float> jv(nodes.size(),0.0f);
    std::vector<float> jvBest = rns->getJointValues();
    int step = 0;
    checkTolerances(goal);

    while (step<maxGradientDecentSteps)
    {
        Eigen::VectorXf dTheta = this->computeStep(goal,stepSize);

        for (unsigned int i=0; i<nodes.size();i++)
        {
            jv[i] = (nodes[i]->getJointValue() + dTheta[i]);

            // sanity check
            if (boost::math::isnan(jv[i]) || boost::math::isinf(jv[i]))
            {
                VR_WARNING << "Aborting, invalid joint value (nan)" << endl;
                return false;
            }
        }

        robot->setJointValues(rns,jv);

        // check tolerances
        if (checkTolerances(goal))
        {
            if (verbose)
                VR_INFO << "Tolerances ok, loop:" << step << endl;
            // try to improve pose by applying some joint limit avoidance steps
            applyJLA(goal, jlaSteps, stepSize);
            return true;
        }

        /*float posDist = getCurrentError(goal);
        if (checkImprovement && posDist>lastDist)
        {
            if (verbose)
                VR_INFO << "Could not improve result any more (current position error=" << posDist << ", last loop's error:" << lastDist << "), loop:" << step << endl;
            robot->setJointValues(rns,jvBest);
            return false;
        }*/
       
        float d = dTheta.norm();
        if (d<minumChange)
        {
            if (verbose)
                VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << "), loop:" << step << endl;
            return false;
        }
        jvBest = jv;
        step++;
    }
    if (verbose)
    {
        VR_INFO << "IK failed, loop:" << step << endl;
        VR_INFO << "pos error:" << getCurrentError(goal) << endl;
    }
    robot->setJointValues(rns, jvBest);
    return false;
}

float GazeIK::getMaxPosError()
{
	return maxPosError;
}

} // namespace VirtualRobot
