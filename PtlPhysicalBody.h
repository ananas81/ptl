#ifndef __PHYSICAL_BODY_H
	#define __PHYSICAL_BODY_H

#include <btBulletDynamicsCommon.h>
//#include <btCollisionObject.h>
//#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <OgreEntity.h>
#include <OgreSceneNode.h>
#include "PtlPerpetualCommon.h"

namespace Ptl
{

class PhysicalBody
{
	public:
//		class BodyCommand
//		{
//			virtual ~BodyCommand() = 0;
//		};
//
//		BodyCommand::~BodyCommand() {}

		virtual void initPhysics();

		PhysicalBody(btMotionState *motionState,
			     btCollisionShape *colShape,
			     const btVector3& pos,
			     const btQuaternion& orient,
			     double mass,
			     const btVector3& inertia,
			     double friction,
			     double rollingFriction);
		virtual ~PhysicalBody();
		virtual btCollisionObject* getCollisionObject() const;
		virtual void setMotionState(btMotionState *motionState);

	protected:
		btMotionState *mMotionState;
		btCollisionShape *mCollisionShape;
		btVector3 mPos;
		btQuaternion mOrient;
		double mMass;
		btVector3 mInertia;
		double mFriction;
		double mRollingFriction;
		btCollisionObject *mCollisionObject;
};

};
	
#endif //__PHYSICAL_BODY_H
