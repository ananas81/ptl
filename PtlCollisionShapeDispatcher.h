#ifndef __COLLISION_SHAPE_DISPATCHER_H
	#define __COLLISION_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
#include "PtlOgrePhysicalBody.h"

namespace Ptl
{

class CollisionShapeDispatcher
{
	public:
		virtual ~CollisionShapeDispatcher() = 0;
		virtual btCollisionShape* getCollisionShape(OgrePhysicalBody *body) const = 0;
};

};
	
#endif //__COLLISION_SHAPE_DISPATCHER_H
