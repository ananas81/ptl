#ifndef __OGRE_PHYSICAL_BODY_H
	#define __OGRE_PHYSICAL_BODY_H

#include <btBulletDynamicsCommon.h>
//#include <btCollisionObject.h>
//#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <OgreEntity.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include "PtlPerpetualCommon.h"
#include "PtlPhysicalBody.h"
#include "PtlBulletImporterShapeDispatcher.h"
#include "PtlBtOgreShapeDispatcher.h"

namespace Ptl
{

class OgrePhysicalBody : public PhysicalBody
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
			    Ogre::SceneNode *mVisualObj;
			    btTransform mPos;
		};

		OgrePhysicalBody(Ogre::SceneManager* sceneMgr,
				 const std::string& bodyName,
				 const std::string& meshName,
				 const Ogre::Vector3& pos,
				 const Ogre::Quaternion& orient,
				 CollisionShapeDispatcher* shapeDispatcher,
				 double mass,
				 const Ogre::Vector3& inertia,
				 double friction,
				 double rollingFriction);

		OgrePhysicalBody(Ogre::SceneManager* sceneMgr,
				 const std::string& bodyName,
				 const std::string& meshName,
				 const Ogre::Vector3& pos,
				 const Ogre::Quaternion& orient,
				 btCollisionShape* colShape,
				 double mass,
				 const Ogre::Vector3& inertia,
				 double friction,
				 double rollingFriction);

		virtual ~OgrePhysicalBody();
		virtual btCollisionObject* getCollisionObject() const;
		virtual Ogre::Entity* getVisualObject() const;
		virtual Ogre::SceneNode* getBodyNode() const;

	private:
		Ogre::SceneManager *mSceneManager;
		const std::string mBodyName;
		const std::string mMeshName;
		Ogre::SceneNode *mBodyNode;
		Ogre::Entity *mEntity;
};

};
	
#endif //__OGRE_PHYSICAL_BODY_H
