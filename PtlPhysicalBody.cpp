#include "PhysicalBody.h"

namespace Ptl
{

PhysicalBody::PhysicalBody(Ogre::SceneManager* sceneManager,
			   const std::string& bodyName,
			   const std::string& meshName,
			   const Ogre::Vector3& pos,
			   const Ogre::Quaternion& orient,
			   btCollisionShape* shape,
			   double mass,
			   const btVector3& inertia, 
			   double friction,
			   double rollingFriction) :
			   mSceneManager(sceneManager),
			   mBodyNode(NULL),
			   mBodyName(bodyName),
			   mMeshName(meshName),
			   mPos(pos),
			   mOrient(orient),
			   mEntity(NULL),
			   mShape(shape),
			   mConstrInfo(constrInfo),
			   mPos(pos),
			   mMass(mass),
			   mInertia(inertia),
			   mCollisionObject(NULL),
			   mMotionState(NULL),
			   mFriction(friction),
			   mRollingFriction(rollingFriction)
{
	init();
}

PhysicalBody::~PhysicalBody()
{
	delete mMotionState;
	delete mEntity;
}

btCollisionObject* PhysicalBody::getCollisionObject() const
{
	return mCollisionObject;
}

Ogre::Entity* PhysicalBody::getVisualObject() const
{
	return mEntity;
}

void PhysicalBody::init()
{
        mEntity = mSceneMgr->createEntity(mBodyName.c_str(), mMeshName.c_str());
	mBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(mBodyName.c_str(),
									mPos,
									mOrient);
	mBodyNode->attachObject(mEntity);
	mMotionState = new MotionState(btTransform(btQuaternion(mOrient.x, mOrient.y, mOrient.z, mOrient.w),
						   btVector3(mPos.x, mPos.y, mPos.z)),
				       mBodyNode);
	mShape->calculateLocalInertia(mMass, mInertia);
	btRigidBody::btRigidBodyConstructionInfo
					constrInfo(mMass,
						   mMotionState,
						   mShape,
						   mInertia);
	constrInfo.m_friction = mFriction;
	constrInfo.m_rollingFriction = mRollingFriction;
	mCollisionObject = new btRigidBody(mConstrInfo); 
}

PhysicalBody::MotionState::MotionState(const btTransform &initialpos, Ogre::SceneNode *node) :
	mVisibleObj(node),
	mPos(initialpos)
{
}

PhysicalBody::MotionState::~MotionState()
{
}

void PhysicalBody::MotionState::setNode(Ogre::SceneNode *node)
{
	mVisibleObj = node;
}

void PhysicalBody::MotionState::getWorldTransform(btTransform &worldTrans) const
{
	worldTrans = mPos;
}

void PhysicalBody::MotionState::setWorldTransform(const btTransform &worldTrans)
{
	if(NULL == mVisibleObj) {
		PTL_LOG("mVisibleObj null pointer");
		return; // silently return before we set a node
	}
	btQuaternion rot = worldTrans.getRotation();
	mVisibleObj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
	btVector3 pos = worldTrans.getOrigin();
	mVisibleObj->setPosition(pos.x(), pos.y(), pos.z());
}

};
