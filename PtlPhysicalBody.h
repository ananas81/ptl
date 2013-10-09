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

		PhysicalBody(Ogre::SceneManager* sceneMgr,
			     const std::string& bodyName,
			     const std::string& meshName,
			     const Ogre::Vector3& pos,
			     const Ogre::Quaternion& orient,
			     btCollisionShape* shape,
			     double mass,
			     const btVector3& inertia,
			     double friction,
			     double rollingFriction);
		virtual ~PhysicalBody();
		virtual btCollisionObject* getCollisionObject() const;
		virtual Ogre::Entity* getVisualObject() const;

	private:
		Ogre::SceneManager *mSceneManager;
		Ogre::SceneNode *mBodyNode;
		const std::string mBodyName;
		const std::string mMeshName;
		Ogre::Vector3 mPos;
		Ogre::Quaternion& mOrient;
		Ogre::Entity *mEntity;
		btCollisionShape *mShape;
		double mMass;
		btVector3 mInertia;
		btCollisionObject *mCollisionObject;
		MotionState *mMotionState;
		double mFriction;
		double mRollingFriction;
};

};
	
#endif //__PHYSICAL_BODY_DISPATCHER_H
