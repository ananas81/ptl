#ifndef __PARALLEL_RINGWHEELS
	#define __PARALLEL_RINGWHEELS

#include "PtlBaseDeviceScene.h"
#include "PtlRackComponent.h"

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
	    Ptl::RackBodyComponent *mRack1;
	    Ptl::RackBodyComponent *mRack2;
};

};
	
#endif //__PARALLEL_RINGWHEELS
