#include "PtlOgrePhysicalBody.h"
#include "PtlWheelComponent.h"
#include "PtlChainComponent.h"

namespace Ptl
{

WheelBodyComponent::WheelBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mWheel(NULL),
				       mWheelHinge(NULL)
{
	mRack = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "Rack",
						  "Rack.mesh",
						  Ogre::Vector3(mPos.x, mPos.y, mPos.z - 150),
						  Ogre::Quaternion(sqrt(0.5), 0 , -sqrt(0.5), 0),
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  100.0,
						  Ogre::Vector3(0, 0, 0),
						  0.,
						  0.);

	mWheel = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "Flywheel",
						  "Flywheel.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("flywheel.bcs", 0),
						  40.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mWheel->getCollisionObject());
	btRigidBody *rackBody = static_cast<btRigidBody*>(mRack->getCollisionObject());
	rackBody->setLinearFactor(btVector3(1, 1, 1));

	btTransform rackBottom = btTransform::getIdentity();
	rackBottom.setOrigin(btVector3(0., mPos.y-170.0, 0.));
	btGeneric6DofConstraint *dofConstr = new btGeneric6DofConstraint(*rackBody, rackBottom, true);
	rackConstr = dofConstr;

        btRotationalLimitMotor *dofRotMotor;

	dofConstr->setLinearUpperLimit(btVector3(0., 0.0, 100.0));
	dofConstr->setLinearLowerLimit(btVector3(0., 0.0, -100.0));
	dofConstr->setAngularUpperLimit(btVector3(0, 0, 0));
	dofConstr->setAngularLowerLimit(btVector3(0, 0, 0));

	mWheelHinge = new btHingeConstraint(*wheelBody,
					    *rackBody,
					    btVector3(0, 0, -3.64),
					    btVector3(150.0, 0, 0),
					    btVector3(0, 0, 1),
					    btVector3(1, 0, 0), true);

	wheelBody->setFriction(1);
	wheelBody->setDamping(0.1f,0.1f);
	wheelBody->setFlags(0);
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	mWorld->addRigidBody(wheelBody);
	mWorld->addRigidBody(rackBody);
	mWorld->addConstraint(mWheelHinge);              
	mWorld->addConstraint(dofConstr);

	btTransform frameInA;

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y - 38.5, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_DOWN));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., -38.5, 0.));

	mChildComponents[0]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x - 38.5, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_LEFT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(-38.5, 0., 0.));

	mChildComponents[1]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y + 38.5, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_UP));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., 38.5, 0.));

	mChildComponents[2]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x + 38.5, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_RIGHT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(38.5, 0., 0.));

	mChildComponents[3]->attachTo(wheelBody, frameInA);

	rackBody->setActivationState(DISABLE_DEACTIVATION);
	dofConstr->getFrameOffsetA().setRotation(btQuaternion(0, sqrt(0.5), 0, sqrt(0.5)));
}

WheelBodyComponent::~WheelBodyComponent()
{
}

btRigidBody* WheelBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mWheel->getCollisionObject());
}

btRigidBody* WheelBodyComponent::getRackBody()
{
	return static_cast<btRigidBody*>(mRack->getCollisionObject());
}

btTransform WheelBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0., 0., 0.));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}

void WheelBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
}

btHingeConstraint* WheelBodyComponent::getHinge()
{
	return mWheelHinge;
}

void WheelBodyComponent::setActivationState(int actState)
{
	btRigidBody* body;

	body = static_cast<btRigidBody*>(mRack->getCollisionObject());
	body->setActivationState(actState);
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

	body = static_cast<btRigidBody*>(mRack->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->setMassProps(0.0, btVector3(0,0,0));
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_NO_CONTACT_RESPONSE);
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

	body = static_cast<btRigidBody*>(mRack->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->getCollisionShape()->calculateLocalInertia(mRack->getMass(), inertia);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	body->setMassProps(mRack->getMass(), btVector3(0,0,0));
	//body->setActivationState(WANTS_DEACTIVATION);
	body->forceActivationState(ACTIVE_TAG);
	body->setDeactivationTime( 0.f );
	body->updateInertiaTensor();
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToDynamic();
}

void WheelBodyComponent::displace()
{
	static bool en = false;

	btRigidBody *rackBody = static_cast<btRigidBody*>(mRack->getCollisionObject());
	btTransform rackTrans;
	btMotionState *motionState = rackBody->getMotionState();
	motionState->getWorldTransform(rackTrans);

	btVector3 origin = rackTrans.getOrigin();
	printf("movedRack: x: %2.2f, y: %2.2f. z: %2.2f\n", origin.getX(), origin.getY(), origin.getZ());

	rackConstr->getFrameOffsetA().setOrigin(origin);
	if (!en)
	{
		rackConstr->setLinearUpperLimit(btVector3(0., 0., 0.));
		rackConstr->setLinearLowerLimit(btVector3(0., 0., 0.));
	}
	else
	{
		rackConstr->setLinearUpperLimit(btVector3(0., 0., 200.));
		rackConstr->setLinearLowerLimit(btVector3(0., 0., -200.));
	}

	en = !en;
}

};
