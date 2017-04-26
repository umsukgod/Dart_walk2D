#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
using namespace dart::common;
using namespace dart::utils;
using namespace dart::gui;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace dart::math;


class MyWindow : public SimWindow
{
  public :
  MyWindow(const WorldPtr& world)
  : mForceCountDown(0), mPositiveSign(true)
  {
    setWorld(world);
  }

  void timeStepping() override
  {

    SimWindow::timeStepping();
  }
protected:
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


int main(int argc, char* argv[])
{
	SkeletonPtr floor = createFloor();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(floor);

	MyWindow window(world);

	glutInit(&argc, argv);
	window.initWindow(640, 480, "my Walk 2D");
	glutMainLoop();
}