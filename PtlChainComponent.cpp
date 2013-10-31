#include "PtlChainComponent.h"
#include "PtlOgrePhysicalBody.h"
#include <stdio.h>

namespace Ptl
{

ChainBodyComponent::ChainBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mOgrePhysBody(NULL),
				       mChainHinge(NULL)
				      
{
	double chainElementMass = 0.1;

	mChainElements.push_back(new Ptl::OgrePhysicalBody(
					mSceneMgr,
					"RopeSphere_0",
					"SmallSphere.mesh",
					mPos,
					mOrient,
					new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::SPHERE),
					chainElementMass,
					Ogre::Vector3(0, 0, 0),
					10.0,
					1.0));

	btCollisionShape* chainElementShape = mChainElements[0]->getCollisionShape();

	int i;
	for (i = 1; i < 40; ++i)
	{
		char bodyName[15];
		sprintf(bodyName, "RopeSphere_%d", i);
		mChainElements.push_back(new Ptl::OgrePhysicalBody(
					mSceneMgr,
					bodyName,
					"SmallSphere.mesh",
					Ogre::Vector3(mPos.x, mPos.y-i*2.0, mPos.z),
					mOrient,
					chainElementShape,
					chainElementMass,
					Ogre::Vector3(0, 0, 0),
					10.0,
					1.0));
	}

	mChainElements.push_back(new Ptl::OgrePhysicalBody(
					mSceneMgr,
					"WeightSphere_0",
					"Sphere.mesh",
					Ogre::Vector3(mPos.x, mPos.y-i*2.0-6.12, mPos.z),
					mOrient,
					new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::SPHERE),
					10.0,
					Ogre::Vector3(0, 0, 0),
					10.0,
					1.0));

        for (i = 0; i < mChainElements.size(); ++i)
        {
                btRigidBody* ropeElementBody = static_cast<btRigidBody*>(mChainElements[i]->getCollisionObject());
                ropeElementBody->setActivationState(DISABLE_DEACTIVATION);
                mWorld->addRigidBody(static_cast<btRigidBody*>(mChainElements[i]->getCollisionObject()));
        }

	btGeneric6DofConstraint* dofConstraint;
	btRigidBody *s1, *s2;
	btTransform frameInA, frameInB;

	for (i = 1; i < mChainElements.size() - 1; ++i)
	{
		s1 = static_cast<btRigidBody*>(mChainElements[i-1]->getCollisionObject());
		s2 = static_cast<btRigidBody*>(mChainElements[i]->getCollisionObject());
		frameInA = btTransform::getIdentity();
		frameInB = btTransform::getIdentity();
		frameInA.setOrigin(btVector3(0., -1.0, 0.));
		frameInB.setOrigin(btVector3(0., 1.0, 0.));
		dofConstraint = new btGeneric6DofConstraint(*s1, *s2,frameInA,frameInB,true);
		dofConstraint->setLinearUpperLimit(btVector3(0., 0., 0.));
		dofConstraint->setLinearLowerLimit(btVector3(0., 0., 0.));
		dofConstraint->setAngularUpperLimit(btVector3(0., 0., 0.));
		dofConstraint->setAngularLowerLimit(btVector3(0., 0., 0.));
		dofConstraint->setOverrideNumSolverIterations(100);
		mWorld->addConstraint(dofConstraint);
	}

	s1 = static_cast<btRigidBody*>(mChainElements[i-1]->getCollisionObject());
	s2 = static_cast<btRigidBody*>(mChainElements[i]->getCollisionObject());
	frameInA = btTransform::getIdentity();
	frameInB = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., -1.0, 0.));
	frameInB.setOrigin(btVector3(0., 6.12, 0.));
	dofConstraint = new btGeneric6DofConstraint(*s1, *s2,frameInA,frameInB,true);
	dofConstraint->setLinearUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setLinearLowerLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularUpperLimit(btVector3(0., 0., 0.));
	dofConstraint->setAngularLowerLimit(btVector3(0., 0., 0.));
	dofConstraint->setOverrideNumSolverIterations(100);
	mWorld->addConstraint(dofConstraint);
}

ChainBodyComponent::~ChainBodyComponent()
{
}

btRigidBody* ChainBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mChainElements[0]->getCollisionObject());
}

btVector3 ChainBodyComponent::getRootAnchor()
{
	return btVector3(mPos.x, mPos.y + 1.0, mPos.z);
}

};
