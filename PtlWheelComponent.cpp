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
				       mWheel(NULL),
				       mWheelHinge(NULL)
{
//	mRack = new Ptl::OgrePhysicalBody(mSceneMgr,
//						  "Rack",
//						  "Rack.mesh",
//						  Ogre::Vector3(mPos.x, mPos.y+10, mPos.z - 150),
//						  Ogre::Quaternion(sqrt(0.5), 0 , -sqrt(0.5), 0),
//						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
//						  1.0,
//						  Ogre::Vector3(0, 0, 0),
//						  10.0,
//						  1.0);
	mRack = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "Rack",
						  "Sphere.mesh",
						  Ogre::Vector3(25, 10, -150),
						  Ogre::Quaternion(sqrt(0.5), 0 , -sqrt(0.5), 0),
						  new Ptl::BtOgreShapeDispatcher(NULL, Ptl::BtOgreShapeDispatcher::CONVEX_HULL),
						  1.0,
						  Ogre::Vector3(0, 0, 0),
						  10.0,
						  1.0);

	mWheel = new Ptl::OgrePhysicalBody(mSceneMgr,
						  "Flywheel",
						  "Flywheel.mesh",
						  mPos,
						  mOrient,
						  new Ptl::BulletImporterShapeDispatcher("flywheel.bcs", 0),
						  40.0,
						  Ogre::Vector3(0, 0, 0),
						  10.0,
						  1.0);

	btRigidBody *wheelBody = static_cast<btRigidBody*>(mWheel->getCollisionObject());
	btRigidBody *rackBody = static_cast<btRigidBody*>(mRack->getCollisionObject());
	rackBody->setLinearFactor(btVector3(1, 1, 1));
	//rackBody->setAngularFactor(btVector3(0, 0, 0.5));
	//rackBody->setActivationState(DISABLE_DEACTIVATION);

	btTransform rackBottom = btTransform::getIdentity();
	rackBottom.setOrigin(btVector3(25., 10., -150.));
	//rackBottom.setRotation(btQuaternion(0 , sqrt(0.5), 0, sqrt(0.5)));
	btGeneric6DofConstraint *dofConstr = new btGeneric6DofConstraint(*rackBody, rackBottom, true);

        btRotationalLimitMotor *dofRotMotor;

//	dofConstr->setLinearUpperLimit(btVector3(10.0, 0.0, 0.0));
//	dofConstr->setLinearLowerLimit(btVector3(-10.0, 0.0, 0.0));
	dofConstr->setLinearUpperLimit(btVector3(0., 0.0, 10.0));
	dofConstr->setLinearLowerLimit(btVector3(0., 0.0, -10.0));
	dofConstr->setAngularUpperLimit(btVector3(0, 0, 0));
	dofConstr->setAngularLowerLimit(btVector3(0, 0, 0));
/*        for (int i = 0; i < 3; ++i)
	{
		dofRotMotor = dofConstr->getRotationalLimitMotor(i);
		dofRotMotor->m_enableMotor = true;
		dofRotMotor->m_normalCFM = 0.0;
		dofRotMotor->m_stopCFM = 0.0;
		dofRotMotor->m_stopERP = 0.0;
		dofRotMotor->m_maxLimitForce = 100.0;
		dofRotMotor->m_maxMotorForce = 100.0;
//	      dofRotMotor->m_hiLimit = 0.2;
//	      dofRotMotor->m_loLimit = 0.2;
		dofRotMotor->m_limitSoftness = 0.0;
                dofConstr->getTranslationalLimitMotor()->m_enableMotor[i] = true;
                dofConstr->getTranslationalLimitMotor()->m_targetVelocity[i] = 5.0f;
                dofConstr->getTranslationalLimitMotor()->m_maxMotorForce[i] = 10.0f;
	}*/

/*
w	x	y	z	Description
1	0	0	0	Identity quaternion, no rotation
0	1	0	0	180° turn around X axis
0	0	1	0	180° turn around Y axis
0	0	0	1	180° turn around Z axis
sqrt(0.5)	sqrt(0.5)	0	0	90° rotation around X axis
sqrt(0.5)	0	sqrt(0.5)	0	90° rotation around Y axis
sqrt(0.5)	0	0	sqrt(0.5)	90° rotation around Z axis
sqrt(0.5)	-sqrt(0.5)	0	0	-90° rotation around X axis
sqrt(0.5)	0	-sqrt(0.5)	0	-90° rotation around Y axis
sqrt(0.5)	0	0	-sqrt(0.5)	-90° rotation around Z axis*/

//	mWheelHinge = new btHingeConstraint(*wheelBody, btVector3(0,0,0), btVector3(0,0,1), true);
//	mWheelHinge = new btHingeConstraint (btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, const btVector3 &axisInA, const btVector3 &axisInB, bool useReferenceFrameA=false)

	//mWheelHinge = new btHingeConstraint(*wheelBody, *rackBody, btVector3(0,0,0), btVector3(0,0,0), btVector3(0,0,1), btVector3(0,0,1), true);
	btTransform wheelFrame = btTransform::getIdentity();
	wheelFrame.setOrigin(btVector3(0., 0., -3.64));
	//wheelFrame.setRotation(btQuaternion(0, 0, 1, 0));
	btTransform rackFrame = btTransform::getIdentity();
//	rackFrame.setOrigin(btVector3(150.0, 0., 0.));
	rackFrame.setOrigin(btVector3(150.0, 0., 0.));
	rackFrame.setRotation(btQuaternion(0 , sqrt(0.5), 0, sqrt(0.5)));
	
	//mWheelHinge = new btHingeConstraint(*wheelBody, *rackBody, wheelFrame, rackFrame, true);
	mWheelHinge = new btHingeConstraint(*wheelBody, *rackBody, btVector3(0, 0, -3.64), btVector3(150.0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0), true);

	wheelBody->setFriction(1);
	wheelBody->setDamping(0.1f,0.1f);
	wheelBody->setFlags(0);
//	wheelBody->setLinearFactor(btVector3(0, 0, 0.5));
//	wheelBody->setAngularFactor(btVector3(0, 0, 0.5));
	wheelBody->setActivationState(DISABLE_DEACTIVATION);

	mWorld->addRigidBody(wheelBody);
	mWorld->addRigidBody(rackBody);
//	mWorld->addConstraint(mWheelHinge);              
	mWorld->addConstraint(dofConstr);

//	rackBody->setLinearVelocity(btVector3(5, 0, 0));
//	rackBody->setLinearFactor(btVector3(1, 0, 0));
//	rackBody->setCcdMotionThreshold(0.5);

	btTransform frameInA;

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y - 38.5, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_DOWN));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., -38.5, 0.));

	mChildComponents[0]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x - 38.5, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_LEFT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(-38.5, 0., 0.));

	mChildComponents[1]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x, mPos.y + 38.5, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_UP));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(0., 38.5, 0.));

	mChildComponents[2]->attachTo(wheelBody, frameInA);

	mChildComponents.push_back(new ChainBodyComponent(mSceneMgr,
							  mWorld,
							  Ogre::Vector3(mPos.x + 38.5, mPos.y, mPos.z),
							  mOrient,
							  ChainBodyComponent::DIR_RIGHT));

	frameInA = btTransform::getIdentity();
	frameInA.setOrigin(btVector3(38.5, 0., 0.));

	mChildComponents[3]->attachTo(wheelBody, frameInA);
}

WheelBodyComponent::~WheelBodyComponent()
{
}

btRigidBody* WheelBodyComponent::getRootBody()
{
	return static_cast<btRigidBody*>(mWheel->getCollisionObject());
}

btRigidBody* WheelBodyComponent::getRackBody()
{
	return static_cast<btRigidBody*>(mRack->getCollisionObject());
}

btTransform WheelBodyComponent::getRootAnchor()
{
	btTransform frame = btTransform::getIdentity();

	frame.setOrigin(btVector3(0., 0., 0.));
	frame.setRotation(btQuaternion(1, 0, 0, 0));

	return frame;
}

void WheelBodyComponent::attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor)
{
}

btHingeConstraint* WheelBodyComponent::getHinge()
{
	return mWheelHinge;
}

};
