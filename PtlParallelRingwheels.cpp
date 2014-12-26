#include "PtlParallelRingwheels.h"
#include "PtlPerpetualCommon.h"

namespace Ptl
{

void ParallelRingwheels::createScene()
{	
	int rwId = 0;

	mRingwheel[rwId] = new Ptl::RingwheelBodyComponent(mSceneMgr,
						mWorld,
						Ptl::Vector3(0., 100, 0.),
						Ptl::Quaternion(1., 0., 0., 0.));

	mRingwheel[rwId]->attachTo(mRingwheel[rwId]->getRootBody(), mRingwheel[rwId]->getRootAnchor());
	++rwId;

	for (rwId = 1; rwId < NUM_RINGWHEELS; ++rwId) {
		mRingwheel[rwId] = new Ptl::RingwheelBodyComponent(mSceneMgr,
							mWorld,
							Ptl::Vector3(0., 100., Ptl::RingwheelBodyComponent::RINGWHEEL_WIDTH * rwId),
							Ptl::Quaternion(1., 0., 0., 0.));
	
		mRingwheel[rwId]->attachTo(mRingwheel[rwId]->getRootBody(), mRingwheel[rwId]->getRootAnchor());
		mRingwheel[rwId]->attachTo(mRingwheel[rwId - 1]->getRootBody(), mRingwheel[rwId - 1]->getRootAnchor(1), 0);
	}
}

void ParallelRingwheels::postInit()
{
	int rwId;

	for (rwId = 0; rwId < NUM_RINGWHEELS; ++rwId)
		mRingwheel[rwId]->addRingweight();
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
//				mRingwheel[0]->getHinge()->enableAngularMotor(true, -800, 400);
				mRingwheel[0]->getHinge()->enableAngularMotor(true, -2000, 1000);
			else
				mRingwheel[0]->getHinge()->enableMotor(false);
	
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
	

