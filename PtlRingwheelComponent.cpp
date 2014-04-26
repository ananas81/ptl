#include "PtlOgrePhysicalBody.h"
#include "PtlRingwheelComponent.h"
#include "PtlChainComponent.h"
#include <math.h>

namespace Ptl
{

const float RingwheelBodyComponent::RINGWHEEL_RADIUS;
const float RingwheelBodyComponent::RINGWHEEL_WIDTH;
int RingwheelBodyComponent::mRingwheelElementsCnt = 0;

RingwheelBodyComponent::RingwheelBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mRingwheel(NULL),
				       mRingweight(NULL),
				       mRingwheelHinge(NULL)
{
	char bodyName[15];
	double x_45, y_45;

	sprintf(bodyName, "Ringwheel_%d", ++mRingwheelElementsCnt);
	mRingwheel = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/Ringwheel.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("resources/ringwheel.bcs", 0),
						  2.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	mRingweight = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "ringweight",
						  "resources/Ringweight.mesh",
						  Ogre::Vector3(aPos.x, aPos.y + RINGWHEEL_RADIUS - 2, aPos.z),
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::SPHERE),
						  20.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());

//	wheelBody->setFriction(1);
	wheelBody->setDamping(1.0,1.0);
	wheelBody->setFlags(0);
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	btRigidBody *ringweightBody = static_cast<btRigidBody*>(mRingweight->getCollisionObject());

//	ringweightBody->setFriction(1);
//	ringweightBody->setDamping(0.1f,0.1f);
	ringweightBody->setFlags(0);

	mWorld->addRigidBody(ringweightBody);
	mWorld->addRigidBody(wheelBody);

	mRearBlocker[0] = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "rearblocker",
						  "resources/rear_blocker.mesh",
						  Ogre::Vector3(aPos.x + 6.8, aPos.y + 14.62, aPos.z),
						  Ogre::Quaternion(0, 0, 1, 0),
						  //Ogre::Quaternion(0.995, 0.012, 0.1, 0.02),
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  0.1,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	btRigidBody *rearblocker1Body = static_cast<btRigidBody*>(mRearBlocker[0]->getCollisionObject());
	mWorld->addRigidBody(rearblocker1Body);

	mFrontBlocker[0] = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "frontblocker",
						  "resources/front_blocker.mesh",
						  Ogre::Vector3(aPos.x + 6.4, aPos.y + 12.94, aPos.z),
						  //Ogre::Quaternion(1, 0, 0, 0),
						  Ogre::Quaternion(0.000796, -0.252, -0.968, 0.000349),
						  new Ptl::BulletImporterShapeDispatcher("resources/front_blocker.bcs", 0),
						  0.1,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	btRigidBody *frontblocker1Body = static_cast<btRigidBody*>(mFrontBlocker[0]->getCollisionObject());
	mWorld->addRigidBody(frontblocker1Body);

	btTransform frameInA;
	btTransform frameInB;
	btGeneric6DofSpringConstraint* pGen6DOFSpring;

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(6.8, 14.62, 0.));

	frameInB = btTransform::getIdentity();
	frameInB.setOrigin(btVector3(0., 0., 0.));
	frameInB.setRotation(btQuaternion(-0.252, -0.968, 0.000349, 0.000796));

	pGen6DOFSpring = new btGeneric6DofSpringConstraint(*wheelBody, *rearblocker1Body, frameInA, frameInB, true);
	pGen6DOFSpring->setLinearUpperLimit(btVector3(0., 0., 0.));
	pGen6DOFSpring->setLinearLowerLimit(btVector3(0., 0., 0.));
	
	pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
	pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
	
	mWorld->addConstraint(pGen6DOFSpring, true);

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., 12.94, 0.));

	frameInB = btTransform::getIdentity();
	frameInB.setOrigin(btVector3(3., 0., 0.));

	pGen6DOFSpring = new btGeneric6DofSpringConstraint(*wheelBody, *frontblocker1Body, frameInA, frameInB, true);
	pGen6DOFSpring->setLinearUpperLimit(btVector3(0., 0., 0.));
	pGen6DOFSpring->setLinearLowerLimit(btVector3(0., 0., 0.));
	
	pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
	pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
	
	mWorld->addConstraint(pGen6DOFSpring, true);
}

RingwheelBodyComponent::~RingwheelBodyComponent()
{
}

btRigidBody* RingwheelBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
}

btTransform RingwheelBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0, 0, -RINGWHEEL_WIDTH/2.0));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}
		
void RingwheelBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
/*	mRingwheelHinge = new btHingeConstraint(*getRootBody(),
					    *parentComponent,
					    getRootAnchor().getOrigin(),
					    parentAnchor.getOrigin(),
					    btVector3(0, 0, 1),
					    btVector3(1, 0, 0),
					    true);*/
	mRingwheelHinge = new btHingeConstraint(*getRootBody(),
					    getRootAnchor().getOrigin(),
					    btVector3(0, 0, 1));
	mWorld->addConstraint(mRingwheelHinge);
}

btHingeConstraint* RingwheelBodyComponent::getHinge()
{
	return mRingwheelHinge;
}

void RingwheelBodyComponent::setActivationState(int actState)
{
	btRigidBody* body;

	body = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
	body->setActivationState(actState);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->setActivationState(actState);
}

void RingwheelBodyComponent::switchToKinematic()
{
	btRigidBody *body;

	body = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->setMassProps(0.0, btVector3(0,0,0));
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToKinematic();
}

void RingwheelBodyComponent::switchToDynamic()
{
	btRigidBody *body;
	btVector3 inertia(0, 0, 0);

	body = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->getCollisionShape()->calculateLocalInertia(mRingwheel->getMass(), inertia);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	body->setMassProps(mRingwheel->getMass(), btVector3(0,0,0));
//	body->setActivationState(WANTS_DEACTIVATION);
	body->forceActivationState(ACTIVE_TAG);
	body->setDeactivationTime( 0.f );
	body->updateInertiaTensor();
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToDynamic();
}

};
