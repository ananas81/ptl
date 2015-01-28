#include "PtlOgrePhysicalBody.h"
#include "PtlRingwheelGearComponent.h"
#include "PtlSlotBlockerComponent.h"
#include <math.h>

namespace Ptl
{

RingwheelGearBodyComponent::RingwheelGearBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       RingwheelBodyComponent(aSceneMgr, aWorld, aPos, aOrient, false)
{
	btTransform frameInA;
	btTransform frameInB;
	char bodyName[30];

	/* Create ringwheel */
	sprintf(bodyName, "Ringwheelgear_%d", ++mRingwheelElementsCnt);
	mRingwheel = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/ringwheel5_gear.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("resources/ringwheel5_gear.bcs", 0),
						  30.0,
						  Ogre::Vector3(0, 0, 0),
						  0.001,
						  0.001);


	btRigidBody *wheelBody = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());

//	wheelBody->setFriction(1);
	wheelBody->setDamping(0.5, 0.5);
	wheelBody->setFlags(0);
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	mWorld->addRigidBody(wheelBody);

	/* Blockers */
	Ptl::Quaternion rear_blocker_rot[] = { Ptl::Quaternion(0.99, 0., 0., 0.11),
					       Ptl::Quaternion(0.74, 0., 0., 0.67),
					       Ptl::Quaternion(0.20, 0., 0., 0.98),
					       Ptl::Quaternion(-0.41, 0., 0., 0.91),
					       Ptl::Quaternion(-0.87, 0., 0., 0.49) };
	Ptl::Quaternion front_blocker_rot[] = { Ptl::Quaternion(0.96, 0., 0., 0.29),
					        Ptl::Quaternion(0.60, 0., 0., 0.80),
					        Ptl::Quaternion(0.03, 0., 0., 1.0),
					        Ptl::Quaternion(-0.57, 0., 0., 0.82),
					        Ptl::Quaternion(-0.94, 0., 0., 0.33) };
	Ptl::Vector3 rear_blocker_dpos[] = { Ptl::Vector3(2.15, 15.85, 0.),
					     Ptl::Vector3(15.74, 2.86, 0.),	
					     Ptl::Vector3(7.58, -14.09, 0.),
					     Ptl::Vector3(-11.06, -11.56, 0.),
					     Ptl::Vector3(-14.41, 6.94, 0.) };
	Ptl::Vector3 front_blocker_dpos[] = { Ptl::Vector3(9.97, 12.50, 0.),
					     Ptl::Vector3(14.98, -5.62, 0.),
					     Ptl::Vector3(-0.72, -15.98, 0.),
					     Ptl::Vector3(-15.42, -4.26, 0.),
					     Ptl::Vector3(-8.81, 13.35, 0.) };

	btGeneric6DofSpringConstraint* pGen6DOFSpring;

	for (int i = 0; i < NUM_SECTIONS; ++i) {
		/* Create rearblocker */
		sprintf(bodyName, "RingwheelGearBlocker_%d", ++mBlockerElementsCnt);
		mRearBlocker[i] = new Ptl::OgrePhysicalBody(mSceneMgr,
							  bodyName,
							  "resources/rear_blocker.mesh",
							  aPos + rear_blocker_dpos[i],
							  rear_blocker_rot[i],
							  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
							  1.0,
							  Ogre::Vector3(0, 0, 0),
							  1.0,
							  1.0);
	
		btRigidBody *rearblockerBody = static_cast<btRigidBody*>(mRearBlocker[i]->getCollisionObject());

		rearblockerBody->setActivationState(DISABLE_DEACTIVATION);

		rearblockerBody->setDamping(0.1, 0.1);

		mWorld->addRigidBody(rearblockerBody);
	
		/* Attach rearblocker */
		frameInA = btTransform::getIdentity();
		frameInA.setOrigin(rear_blocker_dpos[i]);
	
		frameInB = btTransform::getIdentity();
		frameInB.setOrigin(btVector3(0., 0., 0.) + btVector3(-3., 0., 0.));
		frameInB.setRotation(rear_blocker_rot[i]);
	
		pGen6DOFSpring = new btGeneric6DofSpringConstraint(*wheelBody, *rearblockerBody, frameInA, frameInB, true);
		pGen6DOFSpring->setLinearUpperLimit(btVector3(0., 0., 0.));
		pGen6DOFSpring->setLinearLowerLimit(btVector3(0., 0., 0.));
		
		pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 1.0f));
		pGen6DOFSpring->setStiffness(5, 1000.0f);
		pGen6DOFSpring->enableSpring(5, true);
		
		mWorld->addConstraint(pGen6DOFSpring, true);
	
		/* Create frontblocker */
		sprintf(bodyName, "RingwheelGearBlocker_%d", ++mBlockerElementsCnt);
		mFrontBlocker[i] = new Ptl::OgrePhysicalBody(mSceneMgr,
							  bodyName,
							  "resources/front_blocker.mesh",
							  aPos + front_blocker_dpos[i],
							  front_blocker_rot[i],
							  new Ptl::BulletImporterShapeDispatcher("resources/front_blocker.bcs", 0),
							  0.1,
							  Ogre::Vector3(0, 0, 0),
							  1.0,
							  1.0);
	
		btRigidBody *frontblockerBody = static_cast<btRigidBody*>(mFrontBlocker[i]->getCollisionObject());
		frontblockerBody->setActivationState(DISABLE_DEACTIVATION);

		frontblockerBody->setDamping(0.5, 0.5);

		mWorld->addRigidBody(frontblockerBody);
	
		/* Attach frontblocker */
		frameInA = btTransform::getIdentity();
		frameInA.setOrigin(front_blocker_dpos[i]);
	
		frameInB = btTransform::getIdentity();
		frameInB.setRotation(front_blocker_rot[i]);
	
		pGen6DOFSpring = new btGeneric6DofSpringConstraint(*wheelBody, *frontblockerBody, frameInA, frameInB, true);
		pGen6DOFSpring->setLinearUpperLimit(btVector3(0., 0., 0.));
		pGen6DOFSpring->setLinearLowerLimit(btVector3(0., 0., 0.));
		
		pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, -1.5f));
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
		pGen6DOFSpring->setStiffness(5, 100.0f);
		pGen6DOFSpring->enableSpring(5, true);
		
		mWorld->addConstraint(pGen6DOFSpring, true);
	}

	/* Create sphere weight */
	sprintf(bodyName, "BlockerGear_%d", ++mBlockerElementsCnt);
	mLever = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/Lever.mesh",
						  Ogre::Vector3(aPos.x + 6, aPos.y + 15.54, aPos.z - 4.5),
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  20.0,
						  Ogre::Vector3(0, 0, 0),
						  0.001,
						  0.001);

	btRigidBody *leverBody = static_cast<btRigidBody*>(mLever->getCollisionObject());
	leverBody->setActivationState(DISABLE_DEACTIVATION);

	mWorld->addRigidBody(leverBody);

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(Ptl::Vector3(aPos.x, aPos.y + RINGWHEEL_RADIUS - 3.2, aPos.z - 1.0));

	mLeverConstr = new btGeneric6DofConstraint(*leverBody, frameInA, true);
	mLeverConstr->setLinearUpperLimit(btVector3(0., 0., 0.));
	mLeverConstr->setLinearLowerLimit(btVector3(0., 0., 0.));
	mLeverConstr->setAngularUpperLimit(btVector3(0., 0., 0.));
	mLeverConstr->setAngularLowerLimit(btVector3(0., 0., 0.));

	mWorld->addConstraint(mLeverConstr);
}

};

