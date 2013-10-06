#ifndef __BTOGRE_SHAPE_DISPATCHER_H
	#define __BTOGRE_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
#include <OgreEntity.h>
#include <string>
#include "PtlCollisionShapeDispatcher.h"

namespace Perpetual
{

class PtlBtOgreShapeDispatcher : public PtlCollisionShapeDispatcher
{
	public:
		enum BulletShape
		{
			SPHERE = 0,
			BOX,
			BVH_TRIANGLE_MESH,
			CYLINDER,
			CONVEX_HULL,
			CAPSULE
		};
		
		PtlBtOgreShapeDispatcher(Ogre::Entity* ogreEntity, BulletShape btShape);
		virtual ~PtlBtOgreShapeDispatcher();
		virtual btCollisionShape* getCollisionShape() const;

	private:
		Ogre::Entity* mOgreEntity;
		BulletShape mBtShape;
};

};
	
#endif //__BTOGRE_SHAPE_DISPATCHER_H
