#ifndef __COLLISION_SHAPE_DISPATCHER_H
	#define __COLLISION_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>

namespace Perpetual
{

class PtlCollisionShapeDispatcher
{
	public:
		virtual ~PtlCollisionShapeDispatcher() = 0;
		virtual btCollisionShape* getCollisionShape() const = 0;
};

};
	
#endif //__COLLISION_SHAPE_DISPATCHER_H
