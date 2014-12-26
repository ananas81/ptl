#include "PtlOgrePhysicalBody.h"

namespace Ptl
{

OgrePhysicalBody::OgrePhysicalBody(Ogre::SceneManager* sceneMgr,
				   const std::string& bodyName,
				   const std::string& meshName,
				   const Ogre::Vector3& pos,
				   const Ogre::Quaternion& orient,
				   CollisionShapeDispatcher* shapeDispatcher,
				   double mass,
				   const Ogre::Vector3& inertia,
				   double friction,
				   double rollingFriction) :
				   PhysicalBody(NULL,
						NULL,
						btVector3(pos.x, pos.y, pos.z),
						btQuaternion(orient.x, orient.y, orient.z, orient.w),
						mass,
						btVector3(inertia.x, inertia.y, inertia.z),
						friction,
						rollingFriction),
				   mSceneManager(sceneMgr),
				   mBodyName(bodyName),
				   mMeshName(meshName),
				   mBodyNode(NULL),
				   mEntity(NULL)
{
	mEntity = mSceneManager->createEntity(mBodyName.c_str(), mMeshName.c_str());
	mBodyNode = mSceneManager->getRootSceneNode()->createChildSceneNode(mBodyName.c_str(),
									    pos,
									    orient);
	mBodyNode->attachObject(mEntity);

	shapeDispatcher->setOgreEntity(mEntity);
	mCollisionShape = shapeDispatcher->getCollisionShape();

	mMotionState = new MotionState(btTransform(btQuaternion(orient.x, orient.y, orient.z, orient.w),
						   btVector3(pos.x, pos.y, pos.z)),
				       mBodyNode);
	initPhysics();

}

OgrePhysicalBody::OgrePhysicalBody(Ogre::SceneManager* sceneMgr,
				   const std::string& bodyName,
				   const std::string& meshName,
				   const Ogre::Vector3& pos,
				   const Ogre::Quaternion& orient,
				   btCollisionShape *colShape,
				   double mass,
				   const Ogre::Vector3& inertia,
				   double friction,
				   double rollingFriction) :
				   PhysicalBody(NULL,
						colShape,
						btVector3(pos.x, pos.y, pos.z),
						btQuaternion(orient.x, orient.y, orient.z, orient.w),
						mass,
						btVector3(inertia.x, inertia.y, inertia.z),
						friction,
						rollingFriction),
				   mSceneManager(sceneMgr),
				   mBodyName(bodyName),
				   mMeshName(meshName),
				   mBodyNode(NULL),
				   mEntity(NULL)
{
	mEntity = mSceneManager->createEntity(mBodyName.c_str(), mMeshName.c_str());
	mBodyNode = mSceneManager->getRootSceneNode()->createChildSceneNode(mBodyName.c_str(),
									    pos,
									    orient);
	mBodyNode->attachObject(mEntity);

	mMotionState = new MotionState(btTransform(btQuaternion(orient.x, orient.y, orient.z, orient.w),
						   btVector3(pos.x, pos.y, pos.z)),
				       mBodyNode);
	initPhysics();
}

OgrePhysicalBody::~OgrePhysicalBody()
{
	delete mMotionState;
	delete mEntity;
}

btCollisionObject* OgrePhysicalBody::getCollisionObject() const
{
	return mCollisionObject;
}

btCollisionShape* OgrePhysicalBody::getCollisionShape() const
{
	return mCollisionShape;
}

Ogre::Vector3 OgrePhysicalBody::getPos() const
{
	btVector3 origin;
	btTransform trans;

	mMotionState->getWorldTransform(trans);
	origin = trans.getOrigin();

	return Ogre::Vector3(origin.getX(), origin.getY(), origin.getZ());
}

Ogre::Quaternion OgrePhysicalBody::getOrient() const
{
	btQuaternion orient;
	btTransform trans;

	mMotionState->getWorldTransform(trans);
	orient = trans.getRotation();

	return Ogre::Quaternion(orient.getW(), orient.getX(), orient.getY(), orient.getZ());
}

Ogre::Entity* OgrePhysicalBody::getOgreEntity() const
{
	return mEntity;
}

Ogre::SceneNode* OgrePhysicalBody::getBodyNode() const
{
	return mBodyNode;
}

OgrePhysicalBody::MotionState* OgrePhysicalBody::getMotionState() const
{
	return static_cast<MotionState*>(mMotionState);
}

void OgrePhysicalBody::setKinematicPos(btTransform &currentPos)
{
	static_cast<MotionState*>(mMotionState)->setKinematicPos(currentPos);
}

OgrePhysicalBody::MotionState::MotionState(const btTransform &initialpos, Ogre::SceneNode *node) :
	mVisualObj(node),
	mPos(initialpos)
{
}

OgrePhysicalBody::MotionState::~MotionState()
{
}

void OgrePhysicalBody::MotionState::setNode(Ogre::SceneNode *node)
{
	mVisualObj = node;
}

Ogre::SceneNode *OgrePhysicalBody::MotionState::getNode()
{
	return mVisualObj;
}

void OgrePhysicalBody::MotionState::getWorldTransform(btTransform &worldTrans) const
{
	worldTrans = mPos;
}

void OgrePhysicalBody::MotionState::setWorldTransform(const btTransform &worldTrans)
{
	if(NULL == mVisualObj) {
		PTL_LOG("mVisualObj null pointer");
		return; // silently return before we set a node
	}
	btQuaternion rot = worldTrans.getRotation();
	mVisualObj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
	btVector3 pos = worldTrans.getOrigin();
	mVisualObj->setPosition(pos.x(), pos.y(), pos.z());
	mPos.setOrigin(pos);
	mPos.setRotation(rot);
}

void OgrePhysicalBody::MotionState::setKinematicPos(btTransform &currentPos)
{
	mPos = currentPos;
}

float OgrePhysicalBody::getMass()
{
	return mMass;
}

};
