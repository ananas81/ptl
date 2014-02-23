#ifndef __PARALLEL_FLYWHEELS
	#define __PARALLEL_FLYWHEELS

#include "PtlBaseDeviceScene.h"
#include "PtlRackComponent.h"

namespace Ptl
{

class ParallelFlywheels : public BaseDeviceScene
{
	public:
		ParallelFlywheels(btDiscreteDynamicsWorld* aWorld,
			      Ogre::SceneManager* aSceneMgr) :
				BaseDeviceScene(aWorld, aSceneMgr) {};
		virtual ~ParallelFlywheels() {};

		void createScene();
		virtual void keyPressed(const OIS::KeyEvent& evt);

	private:
	    Ptl::RackBodyComponent *mRack1;
	    Ptl::RackBodyComponent *mRack2;
};

};
	
#endif //__PARALLEL_FLYWHEELS
