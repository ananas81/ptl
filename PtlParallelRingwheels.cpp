#include "PtlParallelRingwheels.h"
#include "PtlPerpetualCommon.h"

namespace Ptl
{

void ParallelRingwheels::createScene()
{
	mRingwheel = new Ptl::RingwheelBodyComponent(mSceneMgr,
						mWorld,
						Ptl::Vector3(0, 100, 0),
						Ptl::Quaternion(1., 0., 0., 0.));

	mRingwheel->attachTo(mRingwheel->getRootBody(), mRingwheel->getRootAnchor());

//	mRack1->getRootBody()->setActivationState(DISABLE_DEACTIVATION);
//	mRack1->getRail()->getFrameOffsetA().setRotation(btQuaternion(0, sqrt(0.5), 0, sqrt(0.5)));
}

void ParallelRingwheels::keyPressed(const OIS::KeyEvent& evt)
{
	switch (evt.key)
	{
		case OIS::KC_1:
		{
			static bool motorOn = false;
			motorOn = !motorOn;
			if (motorOn)
				mRingwheel->getHinge()->enableAngularMotor(true, 1000, 500);
			else
				mRingwheel->getHinge()->enableMotor(false);
	
			break;
		}
//		case OIS::KC_2:
//		{
//			btRigidBody *rackBody = mRack1->getRootBody();
//			rackBody->setActivationState(DISABLE_DEACTIVATION);
//			rackBody->applyImpulse(btVector3(-2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
//			break;
//		}
//		case OIS::KC_3:
//		{
//			btRigidBody *rackBody = mRack1->getRootBody();
//			rackBody->setActivationState(DISABLE_DEACTIVATION);
//			rackBody->applyImpulse(btVector3(2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
//			break;
//		}
//		case OIS::KC_4:
//		{
//			static bool lock = true;
//			mRack1->lockPosition(lock);
//			lock = !lock;
//			break;
//		}
//		case OIS::KC_Z:
//		{
//			static bool motorOn = false;
//			motorOn = !motorOn;
//			if (motorOn)
//				mRack2->getHinge()->enableAngularMotor(true, 100000, 50000);
//			else
//				mRack2->getHinge()->enableMotor(false);
//	
//			break;
//		}
//		case OIS::KC_X:
//		{
//			btRigidBody *rackBody = mRack2->getRootBody();
//			rackBody->setActivationState(DISABLE_DEACTIVATION);
//			rackBody->applyImpulse(btVector3(-2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
//			break;
//		}
//		case OIS::KC_C:
//		{
//			btRigidBody *rackBody = mRack2->getRootBody();
//			rackBody->setActivationState(DISABLE_DEACTIVATION);
//			rackBody->applyImpulse(btVector3(2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
//			break;
//		}
//		case OIS::KC_V:
//		{
//			static bool lock = true;
//			mRack2->lockPosition(lock);
//			lock = !lock;
//			break;
//		}
		default:
			break;
	}
}

};
	

