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
		virtual void postInit();

		static const int NUM_RINGWHEELS = 1;

	private:
		Ptl::RingwheelBodyComponent *mRingwheel[NUM_RINGWHEELS];
};

};
	
#endif //__PARALLEL_RINGWHEELS
