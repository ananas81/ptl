#include "PtlCollisionShapeDispatcher.h"

namespace Ptl
{
CollisionShapeDispatcher::CollisionShapeDispatcher(Ogre::Entity *entity) :
						mOgreEntity(entity)
{
}

CollisionShapeDispatcher::~CollisionShapeDispatcher()
{
}

void CollisionShapeDispatcher::setOgreEntity(Ogre::Entity *entity)
{
	mOgreEntity = entity;
}

};
