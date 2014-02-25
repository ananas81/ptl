#ifndef __PARALLEL_RINGWHEELS
	#define __PARALLEL_RINGWHEELS

#include "PtlBaseDeviceScene.h"
#include "PtlRingwheelComponent.h"

namespace Ptl
{

class ParallelRingwheels : public BaseDeviceScene
{
	public:
		ParallelRingwheels(btDiscreteDynamicsWorld* aWorld,
			      Ogre::SceneManager* aSceneMgr) :
				BaseDeviceScene(aWorld, aSceneMgr) {};
		virtual ~ParallelRingwheels() {};

		void createScene();
		virtual void keyPressed(const OIS::KeyEvent& evt);

	private:
	    Ptl::RingwheelBodyComponent *mRingwheel;
};

};
	
#endif //__PARALLEL_RINGWHEELS
