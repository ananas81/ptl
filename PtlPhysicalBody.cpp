#include "PtlPhysicalBody.h"

namespace Ptl
{

PhysicalBody::PhysicalBody(btMotionState *motionState,
			   btCollisionShape *colShape,
			   const btVector3& pos,
			   const btQuaternion& orient,
			   double mass,
			   const btVector3& inertia,
			   double friction,
			   double rollingFriction) :
			   mMotionState(motionState),
			   mCollisionShape(colShape),
			   mPos(pos),
			   mOrient(orient),
			   mMass(mass),
			   mInertia(inertia),
			   mFriction(friction),
			   mRollingFriction(rollingFriction),
			   mCollisionObject(NULL)
{
}

PhysicalBody::~PhysicalBody()
{
}

btCollisionObject* PhysicalBody::getCollisionObject() const
{
	return mCollisionObject;
}

void PhysicalBody::setMotionState(btMotionState* motionState)
{
	mMotionState = motionState;
}

void PhysicalBody::initPhysics()
{
	mCollisionShape->calculateLocalInertia(mMass, mInertia);
	btRigidBody::btRigidBodyConstructionInfo
					constrInfo(mMass,
						   mMotionState,
						   mCollisionShape,
						   mInertia);
	constrInfo.m_friction = mFriction;
	constrInfo.m_rollingFriction = mRollingFriction;
	mCollisionObject = new btRigidBody(constrInfo); 
}

};
