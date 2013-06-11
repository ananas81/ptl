/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "BaseApplication.h"
#include <btBulletDynamicsCommon.h>
#include <btBulletWorldImporter.h>
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
#include <stdio.h>


class MyMotionState : public btMotionState {
public:
    MyMotionState(const btTransform &initialpos, Ogre::SceneNode *node) {
        mVisibleobj = node;
        mPos1 = initialpos;
    }

    virtual ~MyMotionState() {
    }

    void setNode(Ogre::SceneNode *node) {
        mVisibleobj = node;
    }

    virtual void getWorldTransform(btTransform &worldTrans) const {
        worldTrans = mPos1;
	printf(">>>>>getWorldTransform\n");
    }

    virtual void setWorldTransform(const btTransform &worldTrans) {
	printf(">>>>>setWorldTransform1\n");
        if(NULL == mVisibleobj)
            return; // silently return before we set a node
	printf(">>>>>setWorldTransform2\n");
        btQuaternion rot = worldTrans.getRotation();
        mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
        btVector3 pos = worldTrans.getOrigin();
        mVisibleobj->setPosition(pos.x(), pos.y(), pos.z());
    }

protected:
    Ogre::SceneNode *mVisibleobj;
    btTransform mPos1;
};

class TutorialApplication : public BaseApplication
{
public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);

protected:
    virtual void createScene(void);
    virtual void createFrameListener(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent &evt);
    virtual bool nextLocation(void);
    virtual void preparePhysics(Ogre::Entity* entity, Ogre::SceneNode* node, Ogre::Entity* entity2, Ogre::SceneNode* node2);

    Ogre::Real mDistance;                  // The distance the object has left to travel
    Ogre::Vector3 mDirection;              // The direction the object is moving
    Ogre::Vector3 mDestination;            // The destination the object is moving towards
    Ogre::SceneNode *mNode;
    Ogre::Entity *mEntity;
    Ogre::SceneNode *mNode2;
    Ogre::Entity *mEntity2;
    Ogre::Real mWalkSpeed;                 // The speed at which the object is moving
    std::deque<Ogre::Vector3> mWalkList;   // The list of points we are walking to
    MyMotionState *mFallMotionState;
    MyMotionState *mStaticMotionState;
    btDiscreteDynamicsWorld* mWorld;
};

#endif // #ifndef __TutorialApplication_h_
