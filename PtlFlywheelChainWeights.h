#ifndef __FLYWHEEL_CHAIN_WEIGHTS
	#define __FLYWHEEL_CHAIN_WEIGHTS

#include "PtlBaseDeviceScene.h"
#include "PtlRackComponent.h"

namespace Ptl
{

class FlywheelChainWeights : public BaseDeviceScene
{
	public:
		FlywheelChainWeights(btDiscreteDynamicsWorld* aWorld,
			      Ogre::SceneManager* aSceneMgr) :
				BaseDeviceScene(aWorld, aSceneMgr) {};
		virtual ~FlywheelChainWeights() {};

		void createScene();
		virtual void keyPressed(const OIS::KeyEvent& evt);

	private:
	    Ptl::RackBodyComponent *mRack1;
	    Ptl::RackBodyComponent *mRack2;
};

};
	
#endif //__FLYWHEEL_CHAIN_WEIGHTS
