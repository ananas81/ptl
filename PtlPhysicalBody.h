#ifndef __PHYSICAL_BODY_DISPATCHER_H
	#define __PHYSICAL_BODY_DISPATCHER_H

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

		PhysicalBody(const btVector3& pos,
			     const btQuaternion& orient,
			     btMotionState *motionState,
			     btCollisionShape *colShape,
			     double mass,
			     const btVector3& inertia,
			     double friction,
			     double rollingFriction);
		virtual ~PhysicalBody();
		virtual btCollisionObject* getCollisionObject() const;
		virtual void setCollisionShape(btCollisionShape* shape);
		virtual void setMotionState(btMotionState *motionState);

	protected:
		btCollisionShape *mCollisionShape;
		btMotionState *mMotionState;

	private:
		btVector3 mPos;
		btQuaternion mOrient;
		double mMass;
		btVector3 mInertia;
		double mFriction;
		double mRollingFriction;
		btCollisionObject *mCollisionObject;
};

};
	
#endif //__PHYSICAL_BODY_DISPATCHER_H
