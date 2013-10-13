#include "PhysicalBody.h"

namespace Ptl
{

PhysicalBody::PhysicalBody(const btVector3& pos,
			   const btQuaternion& orient,
			   btMotionState *motionState,
			   btCollisionShape *colShape,
			   double mass,
			   const btVector3& inertia,
			   double friction,
			   double rollingFriction) :
			   mPos(pos),
			   mOrient(orient),
			   mMotionState(motionState),
			   mMass(mass),
			   mInertia(inertia),
			   mFriction(friction),
			   mRollingFriction(rollingFriction),
			   mCollisionShape(NULL),
			   mCollisionObject(NULL)
{
	init();
}

PhysicalBody::~PhysicalBody()
{
}

btCollisionObject* PhysicalBody::getCollisionObject() const
{
	return mCollisionObject;
}

void PhysicalBody::setCollisionShape(btCollisionShape* shape)
{
	mCollisionShape = shape;
}

void PhysicalBody::setMotionState(btMotionState* motionState)
{
	mMotionState = motionState;
}

void PhysicalBody::init()
{
	mShape->calculateLocalInertia(mMass, mInertia);
	btRigidBody::btRigidBodyConstructionInfo
					constrInfo(mMass,
						   mMotionState,
						   mCollisionShape,
						   mInertia);
	constrInfo.m_friction = mFriction;
	constrInfo.m_rollingFriction = mRollingFriction;
	mCollisionObject = new btRigidBody(mConstrInfo); 
}

};
