#include "PtlChainComponent.h"
#include "PtlOgrePhysicalBody.h"
#include <stdio.h>
#include <math.h>

namespace Ptl
{

const double ChainBodyComponent::CHAIN_ELEMENT_RADIUS;
const double ChainBodyComponent::WEIGHT_RADIUS;
int ChainBodyComponent::mChainElementsCnt = 0;

Ogre::Vector3 ChainBodyComponent::calculateChainElementPos(int elementId)
{
	double offsetX = mPos.x, offsetY = mPos.y, offsetZ = mPos.z;

	switch (mDirection)
	{
		case DIR_LEFT:
			offsetX += -CHAIN_ELEMENT_RADIUS - elementId*CHAIN_ELEMENT_RADIUS*2.0;
			break;
		case DIR_UP:
			offsetY += CHAIN_ELEMENT_RADIUS + elementId*CHAIN_ELEMENT_RADIUS*2.0;
			break;
		case DIR_RIGHT:
			offsetX += CHAIN_ELEMENT_RADIUS + elementId*CHAIN_ELEMENT_RADIUS*2.0;
			break;
		case DIR_DOWN:
			offsetY += -CHAIN_ELEMENT_RADIUS - elementId*CHAIN_ELEMENT_RADIUS*2.0;
			break;
		default:
			break;
	}

	return Ogre::Vector3(offsetX, offsetY, offsetZ);
}

Ogre::Vector3 ChainBodyComponent::calculateWeightPos(int elementId)
{
	double offsetX = mPos.x, offsetY = mPos.y, offsetZ = mPos.z;

	switch (mDirection)
	{
		case DIR_LEFT:
			offsetX += -CHAIN_ELEMENT_RADIUS - elementId*CHAIN_ELEMENT_RADIUS*2.0 - WEIGHT_RADIUS;
			break;
		case DIR_UP:
			offsetY += CHAIN_ELEMENT_RADIUS + elementId*CHAIN_ELEMENT_RADIUS*2.0 + WEIGHT_RADIUS;
			break;
		case DIR_RIGHT:
			offsetX += CHAIN_ELEMENT_RADIUS + elementId*CHAIN_ELEMENT_RADIUS*2.0 + WEIGHT_RADIUS;
			break;
		case DIR_DOWN:
			offsetY += -CHAIN_ELEMENT_RADIUS - elementId*CHAIN_ELEMENT_RADIUS*2.0 - WEIGHT_RADIUS;
			break;
		default:
			break;
	}

	return Ogre::Vector3(offsetX, offsetY, offsetZ);
}

ChainBodyComponent::ChainBodyComponent(Ogre::SceneManager *aSceneMgr,
				       btDiscreteDynamicsWorld *aWorld,
				       const Ogre::Vector3& aPos,
				       const Ogre::Quaternion& aOrient,
				       ChainDirection aDirection
				       ) :
				       BodyComponent(aSceneMgr, aWorld, aPos, aOrient),
				       mDirection(aDirection),
				       mOgrePhysBody(NULL),
				       mChainHinge(NULL)
				      
{
	double chainElementMass = 0.1;
	char bodyName[15];

	switch (mDirection)
	{
		case DIR_LEFT:
			mOrient = Ogre::Quaternion(sqrt(0.5), 0, 0, -sqrt(0.5));
			break;
		case DIR_UP:
			mOrient = Ogre::Quaternion(0, 0, 0, 1);
			break;
		case DIR_RIGHT:
			mOrient = Ogre::Quaternion(sqrt(0.5), 0, 0, sqrt(0.5));
			break;
		case DIR_DOWN:
			mOrient = Ogre::Quaternion(1, 0, 0, 0);
			break;
		default:
			mOrient = Ogre::Quaternion(1, 0, 0, 0);
			break;
	}

	sprintf(bodyName, "ChainElement_%d", ++mChainElementsCnt);
	mChainElements.push_back(new Ptl::OgrePhysicalBody(
					mSceneMgr,
					bodyName,
					"ChainSegment.mesh",
					calculateChainElementPos(0),
					mOrient,
					new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::SPHERE),
					chainElementMass,
					Ogre::Vector3(0, 0, 0),
					10.0,
					1.0));

	btCollisionShape* chainElementShape = mChainElements[0]->getCollisionShape();

	int i;
	for (i = 1; i < 4; ++i)
	{
		sprintf(bodyName, "ChainElement_%d", ++mChainElementsCnt);
		mChainElements.push_back(new Ptl::OgrePhysicalBody(
					mSceneMgr,
					bodyName,
					"ChainSegment.mesh",
					calculateChainElementPos(i),
					mOrient,
					chainElementShape,
					chainElementMass,
					Ogre::Vector3(0, 0, 0),
					10.0,
					1.0));
	}

	sprintf(bodyName, "ChainElement_%d", ++mChainElementsCnt);
	mChainElements.push_back(new Ptl::OgrePhysicalBody(
					mSceneMgr,
					bodyName,
					"Sphere.mesh",
					calculateWeightPos(i),
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
		frameInA.setOrigin(btVector3(0., -CHAIN_ELEMENT_RADIUS, 0.));
		frameInB.setOrigin(btVector3(0., CHAIN_ELEMENT_RADIUS, 0.));
		dofConstraint = new btGeneric6DofConstraint(*s1, *s2,frameInA,frameInB,true);
	/*	dofConstraint->setLinearUpperLimit(btVector3(0.3, 0.3, 0.3));
		dofConstraint->setLinearLowerLimit(btVector3(0.3, 0.3, 0.3));
		dofConstraint->setAngularUpperLimit(btVector3(0.3, 0.3, 0.3));
		dofConstraint->setAngularLowerLimit(btVector3(0.3, 0.3, 0.3));*/
		setRotationalMotor(dofConstraint);
		dofConstraint->setOverrideNumSolverIterations(100);
		mWorld->addConstraint(dofConstraint);
	}

	s1 = static_cast<btRigidBody*>(mChainElements[i-1]->getCollisionObject());
	s2 = static_cast<btRigidBody*>(mChainElements[i]->getCollisionObject());
	frameInA = btTransform::getIdentity();
	frameInB = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., -CHAIN_ELEMENT_RADIUS, 0.));
	frameInB.setOrigin(btVector3(0., WEIGHT_RADIUS, 0.));
	dofConstraint = new btGeneric6DofConstraint(*s1, *s2,frameInA,frameInB,true);
/*	dofConstraint->setLinearUpperLimit(btVector3(0.3, 0.3, 0.3));
	dofConstraint->setLinearLowerLimit(btVector3(0.3, 0.3, 0.3));
	dofConstraint->setAngularUpperLimit(btVector3(0.3, 0.3, 0.3));
	dofConstraint->setAngularLowerLimit(btVector3(0.3, 0.3, 0.3));*/
	setRotationalMotor(dofConstraint);
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

void ChainBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
	btGeneric6DofConstraint* dofConstraint;

	dofConstraint = new btGeneric6DofConstraint(*parentComponent, *getRootBody(), parentAnchor, getRootAnchor(), true);
/*	dofConstraint->setLinearUpperLimit(btVector3(0.3, 0.3, 0.3));
	dofConstraint->setLinearLowerLimit(btVector3(0.3, 0.3, 0.3));
	dofConstraint->setAngularUpperLimit(btVector3(0.3, 0.3, 0.3));
	dofConstraint->setAngularLowerLimit(btVector3(0.3, 0.3, 0.3));*/
	setRotationalMotor(dofConstraint);
	dofConstraint->setOverrideNumSolverIterations(100);
        mWorld->addConstraint(dofConstraint);

}

void ChainBodyComponent::setRotationalMotor(btGeneric6DofConstraint* dofConstraint)
{
	btRotationalLimitMotor *dofRotMotor;

	for (int i = 0; i < 3; ++i)
	{
		dofRotMotor = dofConstraint->getRotationalLimitMotor(i);
		dofRotMotor->m_enableMotor = true;
		dofRotMotor->m_normalCFM = 0.0;
		dofRotMotor->m_stopCFM = 0.0;
		dofRotMotor->m_stopERP = 0.0;
//		dofRotMotor->m_hiLimit = 0.2;
//		dofRotMotor->m_loLimit = 0.2;
		dofRotMotor->m_limitSoftness = 0.0;
	}
}

btTransform ChainBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();
	btVector3 anchorPoint(.0, CHAIN_ELEMENT_RADIUS, .0);

	frame.setOrigin(anchorPoint);

	switch (mDirection)
	{
		case DIR_LEFT:
			frame.setRotation(btQuaternion(0, 0, sqrt(0.5), sqrt(0.5)));
			break;
		case DIR_UP:
			frame.setRotation(btQuaternion(0, 0, 1, 0));
			break;
		case DIR_RIGHT:
			frame.setRotation(btQuaternion(0, 0, sqrt(0.5), -sqrt(0.5)));
			break;
		case DIR_DOWN:
			break;
		default:
			frame.setRotation(btQuaternion(0, 0, sqrt(0.5), -sqrt(0.5)));
			break;
	}

	return frame;
}

};
