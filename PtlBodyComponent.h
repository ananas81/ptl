#ifndef __BODY_COMPONENT_H
	#define __BODY_COMPONENT_H

#include <btBulletDynamicsCommon.h>
#include <OgreEntity.h>

namespace Ptl
{

class BodyComponent
{
	public:
		BodyComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     ) : 
				 mSceneMgr(aSceneMgr),
				 mWorld(aWorld),
				 mPos(aPos),
				 mOrient(aOrient) {}

		virtual ~BodyComponent() {}
		virtual btRigidBody* getRootBody() = 0;
		virtual btTransform getRootAnchor() = 0;
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor) = 0;
		virtual void setActivationState(int actState) = 0;
		virtual void addToWorld() = 0;
		virtual void removeFromWorld() = 0;

	protected:
		Ogre::SceneManager* mSceneMgr;
		btDiscreteDynamicsWorld* mWorld;
		Ogre::Vector3 mPos;
		Ogre::Quaternion mOrient;
};

};
	
#endif //__BODY_COMPONENT_H
