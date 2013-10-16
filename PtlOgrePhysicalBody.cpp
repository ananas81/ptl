#include "PtlOgrePhysicalBody.h"

namespace Ptl
{

OgrePhysicalBody::OgrePhysicalBody(Ogre::SceneManager* sceneMgr,
				   const std::string& bodyName,
				   const std::string& meshName,
				   const Ogre::Vector3& pos,
				   const Ogre::Quaternion& orient,
				   CollisionShapeDispatcherData* shapeDispData,
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

	mCollisionShape = dispatchCollisionShape(shapeDispData);

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

btCollisionShape* OgrePhysicalBody::dispatchCollisionShape(CollisionShapeDispatcherData *dispatcherData)
{
	PTL_LOG("Empty OgrePhysicalBody::dispatchCollisionShape called.")

	return NULL;
}

btCollisionShape* OgrePhysicalBody::dispatchCollisionShape(BtOgreDispatcherData *dispatcherData)
{
	BtOgreShapeDispatcher shapeDispatcher(mEntity, dispatcherData->getBtShape());

	return shapeDispatcher.getCollisionShape();
}

btCollisionShape* OgrePhysicalBody::dispatchCollisionShape(BulletImporterDispatcherData *dispatcherData)
{
	BulletImporterShapeDispatcher shapeDispatcher(dispatcherData->getBcsFilename(), dispatcherData->getShapeId());

	return shapeDispatcher.getCollisionShape();
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

Ogre::Entity* OgrePhysicalBody::getVisualObject() const
{
	return mEntity;
}

Ogre::SceneNode* OgrePhysicalBody::getBodyNode() const
{
	return mBodyNode;
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
}

};
