#include "PtlOgrePhysicalBody.h"
#include "PtlSlotBlockerComponent.h"

namespace Ptl
{

int SlotBlockerComponent::mSlotBlockerElementsCnt = 0;

SlotBlockerComponent::SlotBlockerComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mSlotBlocker(NULL)
{
	char bodyName[15];

	sprintf(bodyName, "SlotBlocker_%d", ++mSlotBlockerElementsCnt);
	mSlotBlocker = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/arm_cuboid.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  10.0,
						  Ogre::Vector3(0, 0, 0),
						  0.,
						  0.);


	btRigidBody *slotBlockerBody = static_cast<btRigidBody*>(mSlotBlocker->getCollisionObject());
	slotBlockerBody->setLinearFactor(btVector3(1, 1, 1));

	mWorld->addRigidBody(slotBlockerBody);
}

SlotBlockerComponent::~SlotBlockerComponent()
{
}

btRigidBody* SlotBlockerComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mSlotBlocker->getCollisionObject());
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
	btRigidBody *slotBlockerBody = static_cast<btRigidBody*>(mSlotBlocker->getCollisionObject());

	mSlotBlockerConstr = new btGeneric6DofConstraint(*parentComponent, *slotBlockerBody, parentAnchor, getRootAnchor(), true);
	mSlotBlockerConstr->setLinearUpperLimit(btVector3(0., 0.0, 0.0));
	mSlotBlockerConstr->setLinearLowerLimit(btVector3(0., 0.0, 0.0));
	mSlotBlockerConstr->setAngularUpperLimit(btVector3(0, 0, 0));
	mSlotBlockerConstr->setAngularLowerLimit(btVector3(0, 0, 0));

	mWorld->addConstraint(mSlotBlockerConstr);
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
