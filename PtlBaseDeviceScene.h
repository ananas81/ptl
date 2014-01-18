#ifndef __BASE_DEVICE_SCENE_H
	#define __BASE_DEVICE_SCENE_H

#include <OgreSceneManager.h>
#include <btBulletDynamicsCommon.h>
#include <OISKeyboard.h>

namespace Ptl
{

class BaseDeviceScene
{
	public:
		BaseDeviceScene(btDiscreteDynamicsWorld* aWorld,
			      Ogre::SceneManager* aSceneMgr) :
				mWorld(aWorld),
				mSceneMgr(aSceneMgr) {}
		virtual ~BaseDeviceScene() {}

		virtual void createScene() = 0;
		virtual void keyPressed(const OIS::KeyEvent& evt) = 0;

	protected:
		btDiscreteDynamicsWorld* mWorld;
		Ogre::SceneManager* mSceneMgr;
};

};
	
#endif //__BASE_DEVICE_SCENE_H
