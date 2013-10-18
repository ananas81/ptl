#ifndef __BTOGRE_SHAPE_DISPATCHER_H
	#define __BTOGRE_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
#include <OgreEntity.h>
#include <string>
#include "PtlCollisionShapeDispatcher.h"

namespace Ptl
{

class BtOgreShapeDispatcher : public CollisionShapeDispatcher
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
		
		BtOgreShapeDispatcher(Ogre::Entity* ogreEntity, BulletShape btShape);
		virtual ~BtOgreShapeDispatcher();
		virtual btCollisionShape* getCollisionShape() const;
		virtual void setOgreEntity(Ogre::Entity* entity);

	private:
		Ogre::Entity* mOgreEntity;
		BulletShape mBtShape;
};

};
	
#endif //__BTOGRE_SHAPE_DISPATCHER_H
