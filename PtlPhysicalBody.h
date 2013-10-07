#ifndef __PHYSICAL_BODY_DISPATCHER_H
	#define __PHYSICAL_BODY_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
//#include <btCollisionObject.h>
//#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <OgreEntity.h>
#include <OgreSceneNode.h>
#include "PtlPerpetualCommon.h"

namespace Perpetual
{

class PtlPhysicalBody
{
	public:
		class MotionState : public btMotionState {
			public:
			    MotionState(const btTransform &initialpos, Ogre::SceneNode *node);
			    virtual ~MotionState();
			    void setNode(Ogre::SceneNode *node);
			    virtual void getWorldTransform(btTransform &worldTrans) const;
			    virtual void setWorldTransform(const btTransform &worldTrans);
			protected:
			    Ogre::SceneNode *mVisibleObj;
			    btTransform mPos;
		};

//		class BodyCommand
//		{
//			virtual ~BodyCommand() = 0;
//		};
//
//		BodyCommand::~BodyCommand() {}

		virtual void init();

		PtlPhysicalBody(Ogre::Entity* visualObject,
				btCollisionShape* shape,
				btRigidBody::btRigidBodyConstructionInfo constrInfo,
				const btVector3& pos,
				const btQuaternion& quat,
				double mass,
				const btVector3& inertia);
		virtual ~PtlPhysicalBody();
		virtual btCollisionObject* getCollisionObject() const;
		virtual Ogre::Entity* getVisualObject() const;

	private:
		Ogre::Entity* mVisualObject;
		btCollisionShape *mShape;
		btRigidBody::btRigidBodyConstructionInfo mConstrInfo;
		btVector3 mPos;
		btQuaternion mQuat;
		double mMass;
		btVector3 mInertia;
		btCollisionObject *mCollisionObject;
		MotionState *mMotionState;
};

};
	
#endif //__PHYSICAL_BODY_DISPATCHER_H
