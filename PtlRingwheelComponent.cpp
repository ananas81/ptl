#include "PtlOgrePhysicalBody.h"
#include "PtlRingwheelComponent.h"
#include "PtlChainComponent.h"
#include <math.h>

namespace Ptl
{

const float RingwheelBodyComponent::RINGWHEEL_RADIUS;
const float RingwheelBodyComponent::RINGWHEEL_WIDTH;
int RingwheelBodyComponent::mRingwheelElementsCnt = 0;

RingwheelBodyComponent::RingwheelBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mRingwheel(NULL),
				       mRingweight(NULL),
				       mRingwheelHinge(NULL)
{
	char bodyName[15];

	/* Crete ringwheel */
	sprintf(bodyName, "Ringwheel_%d", ++mRingwheelElementsCnt);
	mRingwheel = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/Ringwheel.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("resources/ringwheel.bcs", 0),
						  2.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	/* Crete sphere weight */
	mRingweight = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "ringweight",
						  "resources/Ringweight.mesh",
						  Ogre::Vector3(aPos.x, aPos.y + RINGWHEEL_RADIUS - 2, aPos.z),
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::SPHERE),
						  20.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());

//	wheelBody->setFriction(1);
	wheelBody->setDamping(1.0,1.0);
	wheelBody->setFlags(0);
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	btRigidBody *ringweightBody = static_cast<btRigidBody*>(mRingweight->getCollisionObject());

//	ringweightBody->setFriction(1);
//	ringweightBody->setDamping(0.1f,0.1f);
	ringweightBody->setFlags(0);

	mWorld->addRigidBody(ringweightBody);
	mWorld->addRigidBody(wheelBody);

	Ptl::Quaternion rear_blocker_rot[] = { Ptl::Quaternion(0.000796, -0.252, -0.968, 0.000349) };
	Ptl::Quaternion front_blocker_rot[] = { Ptl::Quaternion(0.002, 0.0436, 0.999, -0.00052) };
	Ptl::Vector3 rear_blocker_dpos[] = { Ptl::Vector3(6.8, 14.62, 0.) };
	Ptl::Vector3 front_blocker_dpos[] = { Ptl::Vector3(3.33, 15.65, 0.) };

	for (int i = 0; i < 1; ++i) {
		/* Create rearblocker */
		mRearBlocker[0] = new Ptl::OgrePhysicalBody(mSceneMgr,
							  "rearblocker",
							  "resources/rear_blocker.mesh",
							  aPos + rear_blocker_dpos[i],
							  rear_blocker_rot[i],
							  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
							  0.1,
							  Ogre::Vector3(0, 0, 0),
							  1.0,
							  1.0);
	
		btRigidBody *rearblockerBody = static_cast<btRigidBody*>(mRearBlocker[0]->getCollisionObject());
		mWorld->addRigidBody(rearblockerBody);
	
		/* Create frontblocker */
		mFrontBlocker[0] = new Ptl::OgrePhysicalBody(mSceneMgr,
							  "frontblocker",
							  "resources/front_blocker.mesh",
							  aPos + front_blocker_dpos[i],
							  front_blocker_rot[i],
							  new Ptl::BulletImporterShapeDispatcher("resources/front_blocker.bcs", 0),
							  0.1,
							  Ogre::Vector3(0, 0, 0),
							  1.0,
							  1.0);
	
		btRigidBody *frontblockerBody = static_cast<btRigidBody*>(mFrontBlocker[0]->getCollisionObject());
		mWorld->addRigidBody(frontblockerBody);
	
		btTransform frameInA;
		btTransform frameInB;
		btGeneric6DofSpringConstraint* pGen6DOFSpring;
	
		/* Attach rearblocker */
		frameInA = btTransform::getIdentity();
		frameInA.setOrigin(rear_blocker_dpos[i]);
	
		frameInB = btTransform::getIdentity();
		frameInB.setOrigin(btVector3(0., 0., 0.));
		frameInB.setRotation(rear_blocker_rot[i]);
	
		pGen6DOFSpring = new btGeneric6DofSpringConstraint(*wheelBody, *rearblockerBody, frameInA, frameInB, true);
		pGen6DOFSpring->setLinearUpperLimit(btVector3(0., 0., 0.));
		pGen6DOFSpring->setLinearLowerLimit(btVector3(0., 0., 0.));
		
		pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
		
		mWorld->addConstraint(pGen6DOFSpring, true);
	
		/* Attach frontblocker */
		frameInA = btTransform::getIdentity();
		frameInA.setOrigin(front_blocker_dpos[i]);
	
		frameInB = btTransform::getIdentity();
		frameInB.setOrigin(btVector3(0., 0., 0.));
		frameInB.setRotation(front_blocker_rot[i]);
	
		pGen6DOFSpring = new btGeneric6DofSpringConstraint(*wheelBody, *frontblockerBody, frameInA, frameInB, true);
		pGen6DOFSpring->setLinearUpperLimit(btVector3(0., 0., 0.));
		pGen6DOFSpring->setLinearLowerLimit(btVector3(0., 0., 0.));
		
		pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
		
		mWorld->addConstraint(pGen6DOFSpring, true);
	}
}

RingwheelBodyComponent::~RingwheelBodyComponent()
{
}

btRigidBody* RingwheelBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
}

btTransform RingwheelBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0, 0, -RINGWHEEL_WIDTH/2.0));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}
		
void RingwheelBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
/*	mRingwheelHinge = new btHingeConstraint(*getRootBody(),
					    *parentComponent,
					    getRootAnchor().getOrigin(),
					    parentAnchor.getOrigin(),
					    btVector3(0, 0, 1),
					    btVector3(1, 0, 0),
					    true);*/
	mRingwheelHinge = new btHingeConstraint(*getRootBody(),
					    getRootAnchor().getOrigin(),
					    btVector3(0, 0, 1));
	mWorld->addConstraint(mRingwheelHinge);
}

btHingeConstraint* RingwheelBodyComponent::getHinge()
{
	return mRingwheelHinge;
}

void RingwheelBodyComponent::setActivationState(int actState)
{
	btRigidBody* body;

	body = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
	body->setActivationState(actState);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->setActivationState(actState);
}

void RingwheelBodyComponent::switchToKinematic()
{
	btRigidBody *body;

	body = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->setMassProps(0.0, btVector3(0,0,0));
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToKinematic();
}

void RingwheelBodyComponent::switchToDynamic()
{
	btRigidBody *body;
	btVector3 inertia(0, 0, 0);

	body = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());
	mWorld->removeRigidBody(body);
	body->getCollisionShape()->calculateLocalInertia(mRingwheel->getMass(), inertia);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	body->setMassProps(mRingwheel->getMass(), btVector3(0,0,0));
//	body->setActivationState(WANTS_DEACTIVATION);
	body->forceActivationState(ACTIVE_TAG);
	body->setDeactivationTime( 0.f );
	body->updateInertiaTensor();
	mWorld->addRigidBody(body);

	for (int i = 0; i < mChildComponents.size(); ++i)
		mChildComponents[i]->switchToDynamic();
}

};
