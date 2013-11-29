#include "PtlOgrePhysicalBody.h"
#include "PtlRackComponent.h"
#include "PtlWheelComponent.h"

namespace Ptl
{

RackBodyComponent::RackBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mRack(NULL),
				       mFlywheel(NULL),
				       mRail(NULL)
{
	mRack = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "Rack",
						  "Rack.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  100.0,
						  Ogre::Vector3(0, 0, 0),
						  0.,
						  0.);

        mFlywheel = new Ptl::WheelBodyComponent(mSceneMgr,
                                                 mWorld,
                                                 Ogre::Vector3(mPos.x, mPos.y, mPos.z + 150.0),
                                                 Ogre::Quaternion(1, 0, 0, 0));


	btRigidBody *rackBody = static_cast<btRigidBody*>(mRack->getCollisionObject());
	rackBody->setLinearFactor(btVector3(1, 1, 1));

	mWorld->addRigidBody(rackBody);

	btTransform frameInA;
	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(150.0, 0, 0));

	mFlywheel->attachTo(rackBody, frameInA);
}

RackBodyComponent::~RackBodyComponent()
{
}

btRigidBody* RackBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mRack->getCollisionObject());
}

btTransform RackBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();
	btVector3 anchorPoint(0., -170., 0.);

	frame.setOrigin(anchorPoint);
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}

void RackBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
	mRail = new btGeneric6DofConstraint(*parentComponent, parentAnchor, true);
	mRail->setLinearUpperLimit(btVector3(0., 0.0, 300.0));
	mRail->setLinearLowerLimit(btVector3(0., 0.0, -300.0));
	mRail->setAngularUpperLimit(btVector3(0, 0, 0));
	mRail->setAngularLowerLimit(btVector3(0, 0, 0));

	mWorld->addConstraint(mRail);
}

void RackBodyComponent::setActivationState(int actState)
{
	btRigidBody* body;

	body = static_cast<btRigidBody*>(mRack->getCollisionObject());
	body->setActivationState(actState);
	mFlywheel->setActivationState(actState);
}

void RackBodyComponent::switchToKinematic()
{
	btRigidBody *body;

	body = static_cast<btRigidBody*>(mRack->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->setMassProps(0.0, btVector3(0,0,0));
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);
	mWorld->addRigidBody(body);

	mFlywheel->switchToKinematic();
}

void RackBodyComponent::switchToDynamic()
{
	btRigidBody *body;
	btVector3 inertia(0, 0, 0);

	body = static_cast<btRigidBody*>(mRack->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->getCollisionShape()->calculateLocalInertia(mRack->getMass(), inertia);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	body->setMassProps(mRack->getMass(), btVector3(0,0,0));
//	body->setActivationState(WANTS_DEACTIVATION);
	body->forceActivationState(ACTIVE_TAG);
	body->setDeactivationTime( 0.f );
	body->updateInertiaTensor();
	mWorld->addRigidBody(body);

	mFlywheel->switchToDynamic();
}

void RackBodyComponent::lockPosition(bool lock)
{
	btRigidBody *rackBody = static_cast<btRigidBody*>(mRack->getCollisionObject());
	btTransform rackTrans;
	btMotionState *motionState = rackBody->getMotionState();
	motionState->getWorldTransform(rackTrans);

	btVector3 origin = rackTrans.getOrigin();
	printf("movedRack: x: %2.2f, y: %2.2f. z: %2.2f\n", origin.getX(), origin.getY(), origin.getZ());

	mRail->getFrameOffsetA().setOrigin(origin);
	if (!lock)
	{
		mRail->setLinearUpperLimit(btVector3(0., 0., 0.));
		mRail->setLinearLowerLimit(btVector3(0., 0., 0.));
	}
	else
	{
		mRail->setLinearUpperLimit(btVector3(0., 0., 200.));
		mRail->setLinearLowerLimit(btVector3(0., 0., -200.));
	}
}

btGeneric6DofConstraint* RackBodyComponent::getRail()
{
	return mRail;
}

btHingeConstraint* RackBodyComponent::getHinge()
{
	return mFlywheel->getHinge();
}

};
