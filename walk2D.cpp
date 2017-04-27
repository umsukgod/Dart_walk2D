#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
using namespace dart::common;
using namespace dart::utils;
using namespace dart::gui;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace dart::math;


class Controller
{
	public :
	Controller(const SkeletonPtr& biped)
	: mBiped(biped), mSpeed(0.0)
	{
		int nDofs = mBiped->getNumDofs();

		mForces = Eigen::VectorXd::Zero(nDofs);

		mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
		mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

		for(std::size_t i = 0; i < 20; ++i)
		{
			mKp(i, i) = 0.0;
			mKd(i, i) = 0.0;
		}

		for(std::size_t i = 20; i < mBiped->getNumDofs(); ++i)
		{
			mKp(i, i) = 1000;
			mKd(i, i) = 50;
		}
		std::cout<<biped->getIndexOf(biped->getDof("j_spine"))<<std::endl;

		setTargetPositions(mBiped->getPositions());
	}

	void setTargetPositions(const Eigen::VectorXd& pose)
	{
		mTargetPositions = pose;
	}
	void clearForces()
	{
		mForces.setZero();
	}

	  /// Add commanind forces from Stable-PD controllers (Lesson 3 Answer)
	void addSPDForces()
	{
		Eigen::VectorXd q = mBiped->getPositions();
		Eigen::VectorXd dq = mBiped->getVelocities();

		Eigen::MatrixXd invM = (mBiped->getMassMatrix()
			+ mKd * mBiped->getTimeStep()).inverse();
		Eigen::VectorXd p =
		-mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
		Eigen::VectorXd d = -mKd * dq;
		Eigen::VectorXd qddot =
		invM * (-mBiped->getCoriolisAndGravityForces()
			+ p + d + mBiped->getConstraintForces());

		mForces += p + d - mKd * qddot * mBiped->getTimeStep();
		mBiped->setForces(mForces);
	}

protected:
	/// The biped Skeleton that we will be controlling
	SkeletonPtr mBiped;

	/// Joint forces for the biped (output of the Controller)
	Eigen::VectorXd mForces;

	/// Control gains for the proportional error terms in the PD controller
	Eigen::MatrixXd mKp;

	/// Control gains for the derivative error terms in the PD controller
	Eigen::MatrixXd mKd;

	/// Target positions for the PD controllers
	Eigen::VectorXd mTargetPositions;

	/// For velocity actuator: Current speed of the skateboard
	double mSpeed;
};

class MyWindow : public SimWindow
{
	public :
	MyWindow(const WorldPtr& world)
	: mForceCountDown(0), mPositiveSign(true)
	{
		setWorld(world);

		mController = dart::common::make_unique<Controller>(
			mWorld->getSkeleton("biped"));
	}

	void timeStepping() override
	{
		mController->clearForces();

		mController->addSPDForces();

		SimWindow::timeStepping();
	}
protected:
	std::unique_ptr<Controller> mController;

	int mForceCountDown;

	bool mPositiveSign;
};

SkeletonPtr createFloor()
{
	SkeletonPtr floor = Skeleton::create("floor");

	// Give the floor a body
	BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

	// Give the body a shape
	double floor_width = 10.0;
	double floor_height = 0.01;
	std::shared_ptr<BoxShape> box(
		new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
	auto shapeNode
	= body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
	shapeNode->getVisualAspect()->setColor(dart::Color::Black());

	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return floor;
}

SkeletonPtr loadBiped()
{
	WorldPtr world = SkelParser::readWorld("//home/gran/Dart/dart/data/skel/biped.skel");
	assert (world != nullptr);

	SkeletonPtr biped = world->getSkeleton("biped");

	for(size_t i=0;i<biped->getNumJoints();++i){
		biped->getJoint(i)->setPositionLimitEnforced(true);
	}

	biped->enableSelfCollisionCheck();

	return biped;
}




int main(int argc, char* argv[])
{
	SkeletonPtr floor = createFloor();
	SkeletonPtr biped = loadBiped();

	WorldPtr world = std::make_shared<World>();
	world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

	world->addSkeleton(floor);
	world->addSkeleton(biped);

	MyWindow window(world);

	glutInit(&argc, argv);
	window.initWindow(640, 480, "my Walk 2D");
	glutMainLoop();
}