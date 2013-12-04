#include "PtlOgrePhysicalBody.h"
#include "PtlWheelComponent.h"
#include "PtlChainComponent.h"
#include <math.h>

namespace Ptl
{

const float WheelBodyComponent::WHEEL_RADIUS;
const float WheelBodyComponent::WHEEL_WIDTH;
int WheelBodyComponent::mWheelElementsCnt = 0;

WheelBodyComponent::WheelBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mWheel(NULL),
				       mWheelHinge(NULL)
{
	char bodyName[15];
	double x_45, y_45;

	sprintf(bodyName, "Flywheel_%d", ++mWheelElementsCnt);
	mWheel = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "Flywheel.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("flywheel.bcs", 0),
						  40.0,
						  Ogre::Vector3(0, 0, 0),
						  1000.0,
						  1000.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mWheel->getCollisionObject());

	wheelBody->setFriction(1);
	wheelBody->setDamping(0.1f,0.1f);
	wheelBody->setFlags(0);
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	x_45 = WHEEL_RADIUS / (2.0 * tan(45.0 *M_PI/180.0));
	y_45 = WHEEL_RADIUS/2.0;
	printf(">>>>> M_PI: %2.2f, x: %2.2f\n", M_PI, x_45);

	mWorld->addRigidBody(wheelBody);

	btTransform frameInA;

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y - WHEEL_RADIUS, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_DOWN));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., -WHEEL_RADIUS, 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x - WHEEL_RADIUS, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_LEFT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(-WHEEL_RADIUS, 0., 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	/* left middle upper chain */
	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x - x_45, mPos.y + y_45, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_LEFT));
	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(-x_45, y_45, 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	/* left middle lower chain */
	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x - x_45, mPos.y - y_45, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_LEFT));
	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(-x_45, -y_45, 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y + WHEEL_RADIUS, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_UP));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., WHEEL_RADIUS, 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x + WHEEL_RADIUS, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_RIGHT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(WHEEL_RADIUS, 0., 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	/* right middle upper chain */
	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x + x_45, mPos.y + y_45, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_RIGHT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(x_45, y_45, 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);

	/* right middle lower chain */
	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x + x_45, mPos.y - y_45, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_RIGHT));
	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(x_45, -y_45, 0.));
	mChildComponents[mChildComponents.size() - 1]->attachTo(wheelBody, frameInA);
}

WheelBodyComponent::~WheelBodyComponent()
{
}

btRigidBody* WheelBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mWheel->getCollisionObject());
}

btTransform WheelBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0, 0, -WHEEL_WIDTH/2.0));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}
		
void WheelBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
	mWheelHinge = new btHingeConstraint(*getRootBody(),
					    *parentComponent,
					    getRootAnchor().getOrigin(),
					    parentAnchor.getOrigin(),
					    btVector3(0, 0, 1),
					    btVector3(1, 0, 0),
					    true);
	mWorld->addConstraint(mWheelHinge);
}

btHingeConstraint* WheelBodyComponent::getHinge()
{
	return mWheelHinge;
}

void WheelBodyComponent::setActivationState(int actState)
{
	btRigidBody* body;

	body = static_cast<btRigidBody*>(mWheel->getCollisionObject());
	body->setActivationState(actState);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->setActivationState(actState);
}

void WheelBodyComponent::switchToKinematic()
{
	btRigidBody *body;

	body = static_cast<btRigidBody*>(mWheel->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->setMassProps(0.0, btVector3(0,0,0));
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToKinematic();
}

void WheelBodyComponent::switchToDynamic()
{
	btRigidBody *body;
	btVector3 inertia(0, 0, 0);

	body = static_cast<btRigidBody*>(mWheel->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->getCollisionShape()->calculateLocalInertia(mWheel->getMass(), inertia);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	body->setMassProps(mWheel->getMass(), btVector3(0,0,0));
//	body->setActivationState(WANTS_DEACTIVATION);
	body->forceActivationState(ACTIVE_TAG);
	body->setDeactivationTime( 0.f );
	body->updateInertiaTensor();
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToDynamic();
}

};
