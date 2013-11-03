#include "PtlOgrePhysicalBody.h"
#include "PtlWheelComponent.h"
#include "PtlChainComponent.h"

namespace Ptl
{

WheelBodyComponent::WheelBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mOgrePhysBody(NULL),
				       mWheelHinge(NULL)
{
	mOgrePhysBody = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "Flywheel",
						  "Flywheel.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("flywheel.bcs", 0),
						  40.0,
						  Ogre::Vector3(0, 0, 0),
						  10.0,
						  1.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mOgrePhysBody->getCollisionObject());
	mWheelHinge = new btHingeConstraint(*wheelBody, btVector3(0,0,0), btVector3(0,0,1), true);

	wheelBody->setFriction(1);
	wheelBody->setDamping(0.01f,0.01f);
	wheelBody->setFlags(0);
	wheelBody->setLinearFactor(btVector3(0, 0, 0.5));
	wheelBody->setAngularFactor(btVector3(0, 0, 0.5));
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	mWorld->addConstraint(mWheelHinge);              
	mWorld->addRigidBody(wheelBody);

	btGeneric6DofConstraint* dofConstraint;
	btTransform frameInA, frameInB;

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y - 38.5, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_DOWN));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., -38.5, 0.));
	frameInB = mChildComponents[0]->getRootAnchor();

	dofConstraint = new btGeneric6DofConstraint(*wheelBody, *mChildComponents[0]->getRootBody(), frameInA, frameInB, true);
	dofConstraint->setOverrideNumSolverIterations(100);
	dofConstraint->setLinearUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setLinearLowerLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularLowerLimit(btVector3(0., 0., 0.));
	mWorld->addConstraint(dofConstraint);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x + 38.5, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_RIGHT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(38.5, 0., 0.));
	frameInB = mChildComponents[1]->getRootAnchor();

	dofConstraint = new btGeneric6DofConstraint(*wheelBody, *mChildComponents[1]->getRootBody(), frameInA, frameInB, true);
	dofConstraint->setOverrideNumSolverIterations(100);
	dofConstraint->setLinearUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setLinearLowerLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularLowerLimit(btVector3(0., 0., 0.));
	mWorld->addConstraint(dofConstraint);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x - 38.5, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_LEFT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(-38.5, 0., 0.));
	frameInB = mChildComponents[2]->getRootAnchor();

	dofConstraint = new btGeneric6DofConstraint(*wheelBody, *mChildComponents[2]->getRootBody(), frameInA, frameInB, true);
	dofConstraint->setOverrideNumSolverIterations(100);
	dofConstraint->setLinearUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setLinearLowerLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularLowerLimit(btVector3(0., 0., 0.));
	mWorld->addConstraint(dofConstraint);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y + 38.5, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_UP));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., 38.5, 0.));
	frameInB = mChildComponents[3]->getRootAnchor();

	dofConstraint = new btGeneric6DofConstraint(*wheelBody, *mChildComponents[3]->getRootBody(), frameInA, frameInB, true);
	dofConstraint->setOverrideNumSolverIterations(100);
	dofConstraint->setLinearUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setLinearLowerLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularLowerLimit(btVector3(0., 0., 0.));
	mWorld->addConstraint(dofConstraint);
}

WheelBodyComponent::~WheelBodyComponent()
{
}

btRigidBody* WheelBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mOgrePhysBody->getCollisionObject());
}

btTransform WheelBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0., 0., 0.));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}

btHingeConstraint* WheelBodyComponent::getHinge()
{
	return mWheelHinge;
}

};
