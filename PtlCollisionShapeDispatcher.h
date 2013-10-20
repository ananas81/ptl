#ifndef __COLLISION_SHAPE_DISPATCHER_H
	#define __COLLISION_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>

namespace Ogre
{
	class Entity;
}

namespace Ptl
{

class CollisionShapeDispatcher
{
	public:
		CollisionShapeDispatcher(Ogre::Entity *entity = NULL);
		virtual ~CollisionShapeDispatcher() = 0;
		virtual btCollisionShape* getCollisionShape() const = 0;
		virtual void setOgreEntity(Ogre::Entity *entity);

	protected:
		Ogre::Entity *mOgreEntity;
};

};
	
#endif //__COLLISION_SHAPE_DISPATCHER_H
