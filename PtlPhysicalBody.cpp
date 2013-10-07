#include "PtlPhysicalBody.h"

namespace Perpetual
{

PtlPhysicalBody::PtlPhysicalBody(Ogre::Entity* visualObject,
				 btCollisionShape* shape,
				 btRigidBody::btRigidBodyConstructionInfo constrInfo,
				 const btVector3& pos,
				 const btQuaternion& quat,
				 double mass,
				 const btVector3& inertia) : 
				 mVisualObject(visualObject),
				 mShape(shape),
				 mConstrInfo(constrInfo),
				 mPos(pos),
				 mMass(mass),
				 mInertia(inertia),
				 mCollisionObject(NULL),
				 mMotionState(NULL) { 
}

PtlPhysicalBody::~PtlPhysicalBody() {
	delete mMotionState;
	delete mCollisionObject;
}

btCollisionObject* PtlPhysicalBody::getCollisionObject() const {
	return mCollisionObject;
}

Ogre::Entity* PtlPhysicalBody::getVisualObject() const {
	return mVisualObject;
}

void PtlPhysicalBody::init()
{
	mMotionState = new MotionState(btTransform(mQuat, mPos), mVisualObject->getParentSceneNode());
	mShape->calculateLocalInertia(mMass, mInertia);
	mCollisionObject = new btRigidBody(mConstrInfo); 
}

PtlPhysicalBody::MotionState::MotionState(const btTransform &initialpos, Ogre::SceneNode *node) :
	mVisibleObj(node),
	mPos(initialpos) {
}

PtlPhysicalBody::MotionState::~MotionState() {
}

void PtlPhysicalBody::MotionState::setNode(Ogre::SceneNode *node) {
	mVisibleObj = node;
}

void PtlPhysicalBody::MotionState::getWorldTransform(btTransform &worldTrans) const {
	worldTrans = mPos;
}

void PtlPhysicalBody::MotionState::setWorldTransform(const btTransform &worldTrans) {
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
