#ifndef __COLLISION_SHAPE_DISPATCHER_H
	#define __COLLISION_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>

namespace Ptl
{

class CollisionShapeDispatcher
{
	public:
		virtual ~CollisionShapeDispatcher() = 0;
		virtual btCollisionShape* getCollisionShape() const = 0;
};

};
	
#endif //__COLLISION_SHAPE_DISPATCHER_H
