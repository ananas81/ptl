#include "PtlFlywheelChainWeights.h"

namespace Ptl
{

void FlywheelChainWeights::createScene()
{
	mRack1 = new Ptl::RackBodyComponent(mSceneMgr,
						mWorld,
						Ogre::Vector3(210., Ptl::RackBodyComponent::RACK_HEIGHT, Ptl::RackBodyComponent::RACK_ARM_LENGTH+40.),
						Ogre::Quaternion(sqrt(0.5), 0., -sqrt(0.5), 0.));

	mRack1->attachTo(mRack1->getRootBody(), mRack1->getRootAnchor());

	mRack1->getRootBody()->setActivationState(DISABLE_DEACTIVATION);
	mRack1->getRail()->getFrameOffsetA().setRotation(btQuaternion(0, sqrt(0.5), 0, sqrt(0.5)));

	mRack2 = new Ptl::RackBodyComponent(mSceneMgr,
						mWorld,
						Ogre::Vector3(-210., Ptl::RackBodyComponent::RACK_HEIGHT, -Ptl::RackBodyComponent::RACK_ARM_LENGTH-40.),
						Ogre::Quaternion(sqrt(0.5), 0., -sqrt(0.5), 0.));

	mRack2->attachTo(mRack2->getRootBody(), mRack2->getRootAnchor());
}

void FlywheelChainWeights::keyPressed(const OIS::KeyEvent& evt)
{
	switch (evt.key)
	{
		case OIS::KC_1:
		{
			static bool motorOn = false;
			motorOn = !motorOn;
			if (motorOn)
				mRack1->getHinge()->enableAngularMotor(true, 100000, 50000);
			else
				mRack1->getHinge()->enableMotor(false);
	
			break;
		}
		case OIS::KC_2:
		{
			btRigidBody *rackBody = mRack1->getRootBody();
			rackBody->setActivationState(DISABLE_DEACTIVATION);
			rackBody->applyImpulse(btVector3(-2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
			break;
		}
		case OIS::KC_3:
		{
			btRigidBody *rackBody = mRack1->getRootBody();
			rackBody->setActivationState(DISABLE_DEACTIVATION);
			rackBody->applyImpulse(btVector3(2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
			break;
		}
		case OIS::KC_4:
		{
			static bool lock = true;
			mRack1->lockPosition(lock);
			lock = !lock;
			break;
		}
		case OIS::KC_Z:
		{
			static bool motorOn = false;
			motorOn = !motorOn;
			if (motorOn)
				mRack2->getHinge()->enableAngularMotor(true, 100000, 50000);
			else
				mRack2->getHinge()->enableMotor(false);
	
			break;
		}
		case OIS::KC_X:
		{
			btRigidBody *rackBody = mRack2->getRootBody();
			rackBody->setActivationState(DISABLE_DEACTIVATION);
			rackBody->applyImpulse(btVector3(-2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
			break;
		}
		case OIS::KC_C:
		{
			btRigidBody *rackBody = mRack2->getRootBody();
			rackBody->setActivationState(DISABLE_DEACTIVATION);
			rackBody->applyImpulse(btVector3(2500, 0, 0), btVector3(0., -Ptl::RackBodyComponent::RACK_HEIGHT, 0.));
			break;
		}
		case OIS::KC_V:
		{
			static bool lock = true;
			mRack2->lockPosition(lock);
			lock = !lock;
			break;
		}
		default:
			break;
	}
}

};
	

