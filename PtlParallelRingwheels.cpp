#include "PtlParallelRingwheels.h"
#include "PtlPerpetualCommon.h"
#include "PtlOgrePhysicalBody.h"

#define POS_X	0.0
#define POS_Y	100.0
#define POS_Z	0.0

namespace Ptl
{

int ParallelRingwheels::mFlywheelElementsCnt = 0;
int ParallelRingwheels::mRingwheelGearElementsCnt = 0;

void ParallelRingwheels::createScene()
{	
	int rwId = 0;
	btTransform frameInA;
	btTransform frameInB;
	char bodyName[30];
	Ptl::Vector3 pos(POS_X, POS_Y, POS_Z);
	Ptl::Quaternion orient(1., 0., 0., 0.);

//	sprintf(bodyName, "Flywheel_%d", ++mFlywheelElementsCnt);
//	mFlywheel = new Ptl::OgrePhysicalBody(mSceneMgr,
//						  bodyName,
//						  "resources/flywheel_rw.mesh",
//						  Ptl::Vector3(pos.mX - GEARWHEEL_RADIUS - FLYWHEEL_RADIUS, pos.mY,
//							       pos.mZ - GEARWHEEL_AXLE_LENGTH - RingwheelBodyComponent::RINGWHEEL_WIDTH/2),
//						  orient,
//						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
//						  //new Ptl::BulletImporterShapeDispatcher("resources/flywheel_rw.bcs", 0),
//						  100.0,
//						  Ogre::Vector3(0, 0, 0),
//						  1.0,
//						  1.0);
//
//	btRigidBody *flywheelBody = static_cast<btRigidBody*>(mFlywheel->getCollisionObject());
//	flywheelBody->setActivationState(DISABLE_DEACTIVATION);
//
//	mWorld->addRigidBody(flywheelBody);
//
//	/* Add flywheel hinge */
//	mFlywheelHinge = new btHingeConstraint(*flywheelBody, btVector3(0., 0., 0.), btVector3(0., 0., 1.), true);
//	mWorld->addConstraint(mFlywheelHinge);
//
//	/* Add ringwheel gear */
//	sprintf(bodyName, "RingwheelGear_%d", ++mRingwheelGearElementsCnt);
//	mRingwheelGear = new Ptl::OgrePhysicalBody(mSceneMgr,
//						  bodyName,
//						  "resources/ringwheel_gear.mesh",
//						  Ptl::Vector3(pos.mX, pos.mY, pos.mZ + GEARWHEEL_AXLE_LENGTH),
//						  orient,
////						  new Ptl::BulletImporterShapeDispatcher("resources/ringwheel_gear.bcs", 0),
//						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
//						  1.0,
//						  Ogre::Vector3(0, 0, 0),
//						  1.0,
//						  1.0);
//
//	btRigidBody *rwgearBody = static_cast<btRigidBody*>(mRingwheelGear->getCollisionObject());
//	rwgearBody->setActivationState(DISABLE_DEACTIVATION);
//
//	mWorld->addRigidBody(rwgearBody);
//
//	/* Gear Constraint */
//	btVector3 axisA(0, 0, 1);
//	btVector3 axisB(0, 0, 1);
//
//	mFlywheelGearConstr = new btGearConstraint(*flywheelBody, *rwgearBody, axisA, axisB, 10.0);
//
//	mWorld->addConstraint(mFlywheelGearConstr);

	mRingwheel[rwId] = new Ptl::RingwheelBodyComponent(mSceneMgr,
						mWorld,
						pos,
						orient);

	mRingwheel[rwId]->attachTo(mRingwheel[rwId]->getRootBody(), mRingwheel[rwId]->getRootAnchor());
	++rwId;

//	/* Axle Constraint */
//	frameInA = btTransform::getIdentity();
//	frameInA.setOrigin(btVector3(0., 0., 0. + GEARWHEEL_AXLE_LENGTH));
//
//	mRwgearAxleConstr = new btGeneric6DofConstraint(*rwgearBody, *mRingwheel[0]->getRootBody(), frameInA, mRingwheel[0]->getRootAnchor(2), true);
//	mRwgearAxleConstr->setLinearUpperLimit(btVector3(0., 0., 0.));
//	mRwgearAxleConstr->setLinearLowerLimit(btVector3(0., 0., 0.));
//	mRwgearAxleConstr->setAngularUpperLimit(btVector3(0., 0., 0.));
//	mRwgearAxleConstr->setAngularLowerLimit(btVector3(0., 0., 0.));
//
//	mWorld->addConstraint(mRwgearAxleConstr);

	for (rwId = 1; rwId < NUM_RINGWHEELS; ++rwId) {
		mRingwheel[rwId] = new Ptl::RingwheelBodyComponent(mSceneMgr,
							mWorld,
							Ptl::Vector3(0., POS_Y, Ptl::RingwheelBodyComponent::RINGWHEEL_WIDTH * rwId),
							orient);
	
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
				mRingwheel[0]->getHinge()->enableAngularMotor(true, -1200, 300);
//				mRingwheel[0]->getHinge()->enableAngularMotor(true, -2000, 1000);
//				mFlywheelHinge->enableAngularMotor(true, -2000, 1000);
			else
				mRingwheel[0]->getHinge()->enableMotor(false);
//				mFlywheelHinge->enableMotor(false);
	
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
	

