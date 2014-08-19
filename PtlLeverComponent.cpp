#include "PtlOgrePhysicalBody.h"
#include "PtlLeverComponent.h"

namespace Ptl
{

int LeverBodyComponent::mLeverElementsCnt = 0;
const double LeverBodyComponent::LEVER_HEIGHT;
const double LeverBodyComponent::LEVER_ARM_LENGTH;

LeverBodyComponent::LeverBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mLever(NULL)
{
	char bodyName[15];

	sprintf(bodyName, "Lever_%d", ++mLeverElementsCnt);
	mLever = new Ptl::OgrePhysicalBody(mSceneMgr,
						  bodyName,
						  "resources/lever.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  100.0,
						  Ogre::Vector3(0, 0, 0),
						  0.,
						  0.);


	btRigidBody *leverBody = static_cast<btRigidBody*>(mLever->getCollisionObject());
	leverBody->setLinearFactor(btVector3(1, 1, 1));

	mWorld->addRigidBody(leverBody);
}

LeverBodyComponent::~LeverBodyComponent()
{
}

btRigidBody* LeverBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mLever->getCollisionObject());
}

btTransform LeverBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0, LEVER_HEIGHT, -LEVER_ARM_LENGTH));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}

void LeverBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
	mLeverConstr = new btGeneric6DofConstraint(*parentComponent, parentAnchor, true);
	mLeverConstr->setLinearUpperLimit(btVector3(0., 0.0, 0.0));
	mLeverConstr->setLinearLowerLimit(btVector3(0., 0.0, 0.0));
	mLeverConstr->setAngularUpperLimit(btVector3(0, 0, 0));
	mLeverConstr->setAngularLowerLimit(btVector3(0, 0, 0));

	mWorld->addConstraint(mLeverConstr);
}

void LeverBodyComponent::setActivationState(int actState)
{
}

void LeverBodyComponent::switchToKinematic()
{
}

void LeverBodyComponent::switchToDynamic()
{
}

};
