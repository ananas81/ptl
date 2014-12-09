#include "PtlOgrePhysicalBody.h"
#include "PtlRingwheelComponent.h"
#include "PtlSlotBlockerComponent.h"
#include <math.h>

namespace Ptl
{

const float RingwheelBodyComponent::RINGWHEEL_RADIUS;
const float RingwheelBodyComponent::RINGWHEEL_WIDTH;
int RingwheelBodyComponent::mRingwheelElementsCnt = 0;
int RingwheelBodyComponent::mRingweightElementsCnt = 0;
int RingwheelBodyComponent::mBlockerElementsCnt = 0;

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
						  "resources/Ringwheel4.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("resources/ringwheel4.bcs", 0),
						  2.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	/* Crete sphere weight */
	sprintf(bodyName, "Ringweight_%d", ++mRingweightElementsCnt);
	mRingweight = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/Ringweight.mesh",
						  Ogre::Vector3(aPos.x, aPos.y + RINGWHEEL_RADIUS - 2., aPos.z),
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::SPHERE),
						  20.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());

//	wheelBody->setFriction(1);
	wheelBody->setDamping(0.8,0.8);
	wheelBody->setFlags(0);
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	btRigidBody *ringweightBody = static_cast<btRigidBody*>(mRingweight->getCollisionObject());

	ringweightBody->setFriction(1);
	ringweightBody->setDamping(0.1f,0.1f);
//+	ringweightBody->setFlags(0);

	mWorld->addRigidBody(ringweightBody);
	mWorld->addRigidBody(wheelBody);

	Ptl::Quaternion rear_blocker_rot[] = { Ptl::Quaternion(1., 0., 0., 0.047),
					       Ptl::Quaternion(0.67, 0.044, 0.02, 0.74),
					       Ptl::Quaternion(-0.047, 0.04, 0.022, 1.),
					       Ptl::Quaternion(-0.74, 0.045, -0.01, 0.67) };
	Ptl::Quaternion front_blocker_rot[] = { Ptl::Quaternion(0.96, 0., 0., 0.26),
					        Ptl::Quaternion(0.26, 0., 0., 0.96),
					        Ptl::Quaternion(-0.74, 0.045, -0.01, 0.67) };
	Ptl::Vector3 rear_blocker_dpos[] = { Ptl::Vector3(0., 16., 0.),
					     Ptl::Vector3(16., 0., 0.),	
					     Ptl::Vector3(0., -16., 0.),
					     Ptl::Vector3(-16., 0., 0.) };
	Ptl::Vector3 front_blocker_dpos[] = { Ptl::Vector3(9.94, 17.23, -2.),
					     Ptl::Vector3(9.94, -17.22, -2.),
					     Ptl::Vector3(-19.89, 0., -2.) };

	btTransform frameInA;
	btTransform frameInB;

	for (int i = 0; i < 4; ++i) {
		/* Create rearblocker */
		sprintf(bodyName, "RingwheelBlocker_%d", ++mBlockerElementsCnt);
		mRearBlocker[i] = new Ptl::OgrePhysicalBody(mSceneMgr,
							  bodyName,
							  "resources/rear_blocker.mesh",
							  aPos + rear_blocker_dpos[i],
							  rear_blocker_rot[i],
							  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
							  0.1,
							  Ogre::Vector3(0, 0, 0),
							  1.0,
							  1.0);
	
		btRigidBody *rearblockerBody = static_cast<btRigidBody*>(mRearBlocker[i]->getCollisionObject());
		mWorld->addRigidBody(rearblockerBody);
	
		btGeneric6DofSpringConstraint* pGen6DOFSpring;
	
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
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
		pGen6DOFSpring->setStiffness(3, 1.0);
		
		mWorld->addConstraint(pGen6DOFSpring, true);
	
		/* Attach frontblocker */
		/*mSlotBlocker[i] = new Ptl::SlotBlockerComponent(aSceneMgr, aWorld, aPos + front_blocker_dpos[i], front_blocker_rot[i]);

		frameInA = btTransform::getIdentity();
		frameInA.setOrigin(front_blocker_dpos[i]);

		mSlotBlocker[i]->attachTo(wheelBody, frameInA);*/
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
	frame.setRotation(Ptl::Quaternion(mOrient));

	return frame;
}

btTransform RingwheelBodyComponent::getRootAnchor(int anchorId)
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0, 0, RINGWHEEL_WIDTH/2.0));
	frame.setRotation(Ptl::Quaternion(0.5, 0., 0., 0.866));

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

void RingwheelBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor, int anchorId)
{
	btGeneric6DofConstraint *pGen6DOF;

	btTransform frameInB;

	/* Attach rearblocker */
	frameInB = btTransform::getIdentity();
	frameInB.setOrigin(btVector3(0., 0., -RINGWHEEL_WIDTH/2.0));
	frameInB.setRotation(Ptl::Quaternion(mOrient));

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mRingwheel->getCollisionObject());

	pGen6DOF = new btGeneric6DofConstraint(*parentComponent, *wheelBody, parentAnchor, frameInB, true);
	pGen6DOF->setLinearUpperLimit(btVector3(0., 0., 0.));
	pGen6DOF->setLinearLowerLimit(btVector3(0., 0., 0.));
	
	pGen6DOF->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
	pGen6DOF->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
	
	mWorld->addConstraint(pGen6DOF, true);
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
