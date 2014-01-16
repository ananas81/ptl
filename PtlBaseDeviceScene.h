#ifndef __BASE_DEVICE_SCENE_H
	#define __BASE_DEVICE_SCENE_H

#include <OgreSceneManager.h>
#include <btBulletDynamicsCommon.h>

namespace Ptl
{

class BaseDeviceScene
{
	public:
		BodyComponent(btDiscreteDynamicsWorld* aWorld,
			      Ogre::SceneManager* aSceneMgr) :
				mWorld(aWorld)
				mSceneMgr(aSceneMgr) {}
		virtual ~BodyComponent() {}

		virtual void keyPressed(const OIS::KeyEvent& evt) = 0;

	protected:
		btDiscreteDynamicsWorld* mWorld;
		Ogre::SceneManager* mSceneMgr;
};

};
	
#endif //__BASE_DEVICE_SCENE_H
