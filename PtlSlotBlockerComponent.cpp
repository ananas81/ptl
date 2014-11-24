#include "PtlOgrePhysicalBody.h"
#include "PtlSlotBlockerComponent.h"

namespace Ptl
{

int SlotBlockerComponent::mBlockerRackElementsCnt = 0;
int SlotBlockerComponent::mBlockerArmElementsCnt = 0;

SlotBlockerComponent::SlotBlockerComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mBlockerRack(NULL)
{
	char bodyName[15];

	sprintf(bodyName, "SlotBlocker_Racl%d", ++mBlockerRackElementsCnt);
	mBlockerRack = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/arm_cuboid.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  1.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);


	btRigidBody *rackBody = static_cast<btRigidBody*>(mBlockerRack->getCollisionObject());
	rackBody->getCollisionShape()->setMargin(0.1);
//	rackBody->setLinearFactor(btVector3(1, 1, 1));

	mWorld->addRigidBody(rackBody);

	sprintf(bodyName, "SlotBlocker_Arm%d", ++mBlockerRackElementsCnt);
	mBlockerArm = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/slot_blocker.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  1.0,
						  Ogre::Vector3(0, 0, 0),
						  1.0,
						  1.0);


	btRigidBody *armBody = static_cast<btRigidBody*>(mBlockerArm->getCollisionObject());
//	rackBody->setLinearFactor(btVector3(1, 1, 1));

	mWorld->addRigidBody(armBody);

	btTransform frameA = btTransform::getIdentity();
	btTransform frameB = btTransform::getIdentity();

	frameA.setOrigin(btVector3(0., 0., -4.));
	frameB.setOrigin(btVector3(0.125, 0., 0.));

	mBlockerArmConstr = new btGeneric6DofConstraint(*rackBody, *armBody, frameA, frameB, true);
	mBlockerArmConstr->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));
	mBlockerArmConstr->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
	mBlockerArmConstr->setAngularUpperLimit(btVector3(-1., 0., 0.));
	mBlockerArmConstr->setAngularLowerLimit(btVector3(1., 0., 0.));

	mWorld->addConstraint(mBlockerArmConstr);
}

SlotBlockerComponent::~SlotBlockerComponent()
{
}

btRigidBody* SlotBlockerComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mBlockerRack->getCollisionObject());
}

btTransform SlotBlockerComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0., -2., 0.));
	frame.setRotation(btQuaternion(mOrient.x, mOrient.y, mOrient.z, mOrient.w));

	return frame;
}

void SlotBlockerComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
	btRigidBody *rackBody = static_cast<btRigidBody*>(mBlockerRack->getCollisionObject());

	mBlockerRackConstr = new btGeneric6DofConstraint(*parentComponent, *rackBody, parentAnchor, getRootAnchor(), true);
	mBlockerRackConstr->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));
	mBlockerRackConstr->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
	mBlockerRackConstr->setAngularUpperLimit(btVector3(0., -0.001, -0.001));
	mBlockerRackConstr->setAngularLowerLimit(btVector3(0., 0.001, 0.001));

	mBlockerRackConstr->setDbgDrawSize(btScalar(5.f));

	mWorld->addConstraint(mBlockerRackConstr);
/*
	btVector3 axisA(0.f, 1.f, 0.f);
	btVector3 axisB(0.f, 1.f, 0.f);
	btVector3 pivotA(-1.f, 0.f, 0.f);
	btVector3 pivotB(1.f, 0.f, 0.f);
	btHingeConstraint *hingeConstr = new btHingeConstraint(*parentComponent, *rackBody,
								parentAnchor.getOrigin(),
								getRootAnchor().getOrigin(), axisA, axisB);
	hingeConstr->setLimit(0., 0., 0.);

	mWorld->addConstraint(hingeConstr);
*/
}

void SlotBlockerComponent::setActivationState(int actState)
{
}

void SlotBlockerComponent::switchToKinematic()
{
}

void SlotBlockerComponent::switchToDynamic()
{
}

};
