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
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include <stdio.h>
#include <CEGUISystem.h>
#include <CEGUISchemeManager.h>
#include <RendererModules/Ogre/CEGUIOgreRenderer.h>
#include <vector>
#include "DebugDraw.hpp"

class DebugDrawer : public CDebugDraw
{
	public:
		DebugDrawer(Ogre::SceneManager* Scene, btDynamicsWorld* World ) : CDebugDraw(Scene, World) {}

		void virtual setDebugMode( int DebugMode ) { CDebugDraw::setDebugMode(DebugMode); }
		virtual ~DebugDrawer() {}

};

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
//	printf(">>>>>getWorldTransform\n");
    }

    virtual void setWorldTransform(const btTransform &worldTrans) {
//	printf(">>>>>setWorldTransform1\n");
        if(NULL == mVisibleobj)
            return; // silently return before we set a node
//	printf(">>>>>setWorldTransform2\n");
        btQuaternion rot = worldTrans.getRotation();
//	printf("mVisibleobj: %p\n",  mVisibleobj);
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
    typedef std::pair<Ogre::SceneNode*, btRigidBody*> WorldObject;

    TutorialApplication(void);
    virtual ~TutorialApplication(void);

    enum QueryFlags
    {       
            NINJA_MASK = 1<<0, 
            ROBOT_MASK = 1<<1
    };

protected:
    virtual void createScene(void);
    virtual void createFrameListener(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent &evt);
    virtual bool nextLocation(void);
    virtual void preparePhysics(Ogre::Entity* entity, Ogre::SceneNode* node, Ogre::Entity* entity2, Ogre::SceneNode* node2);

    virtual bool mouseMoved(const OIS::MouseEvent& arg);
    virtual bool mousePressed(const OIS::MouseEvent& arg, OIS::MouseButtonID id);
    virtual bool mouseReleased(const OIS::MouseEvent& arg, OIS::MouseButtonID id);
    virtual bool keyPressed(const OIS::KeyEvent& arg);

    btRigidBody* getRigidBodyByNode(Ogre::SceneNode *node);

    static void
    pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep);


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
    btSoftBodyWorldInfo m_softBodyWorldInfo;
    Ogre::SceneNode *mRopeObjectNode;

    Ogre::SceneNode *mCurrentObject;        //pointer to our currently selected object
    Ogre::RaySceneQuery* mRayScnQuery;      //pointer to our ray scene query
    CEGUI::Renderer* mGUIRenderer;          //our CEGUI renderer
    bool bLMouseDown, bRMouseDown;

    btRigidBody *mFallRigidBody;
    btRigidBody *mStaticRigidBody;
    Ogre::Vector3 mCurHitPoint;
    btHingeConstraint *mWheelHinge;
    DebugDrawer *mDebugDrawer;

    std::vector<WorldObject*> mWorldObjects;
};

#endif // #ifndef __TutorialApplication_h_
