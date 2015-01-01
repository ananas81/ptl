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

		static const int NUM_RINGWHEELS = 3;
		static const float GEARWHEEL_RADIUS = 2.5;
		static const float GEARWHEEL_AXLE_LENGTH = 5.0;
		static const float FLYWHEEL_RADIUS = 50.0;

	private:
		OgrePhysicalBody *mFlywheel;
		OgrePhysicalBody *mRingwheelGear;
		btGeneric6DofConstraint *mRwgearAxleConstr;
		btHingeConstraint *mFlywheelHinge;
		btGearConstraint *mFlywheelGearConstr;
		Ptl::RingwheelBodyComponent *mRingwheel[NUM_RINGWHEELS];
		static int mFlywheelElementsCnt;
		static int mRingwheelGearElementsCnt;
};

};
	
#endif //__PARALLEL_RINGWHEELS
