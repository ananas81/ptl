#include <btBulletWorldImporter.h>
#include "PtlBtOgreShapeDispatcher.h"
#include "BtOgreGP.h"
#include "PtlPerpetualCommon.h"

namespace Ptl {

BtOgreShapeDispatcher::BtOgreShapeDispatcher(Ogre::Entity* ogreEntity, BulletShape btShape) :
				CollisionShapeDispatcher(ogreEntity),
				mBtShape(btShape)
{
}

BtOgreShapeDispatcher::~BtOgreShapeDispatcher()
{
}

btCollisionShape* BtOgreShapeDispatcher::getCollisionShape() const
{
        BtOgre::StaticMeshToShapeConverter converter(mOgreEntity);
        btCollisionShape *shape;

	switch (mBtShape)
	{
		case SPHERE:
        		shape = converter.createSphere();
			break;
		case BOX:
        		shape = converter.createBox();
			break;
		case BVH_TRIANGLE_MESH:
        		shape = converter.createTrimesh();
			break;
		case CYLINDER:
        		shape = converter.createCylinder();
			break;
		case CONVEX_HULL:
        		shape = converter.createConvex();
			break;
		case CAPSULE:
        		shape = converter.createCapsule();
			break;
		default:
			PTL_LOG("Unsupported bullet shape");
	}

	return shape;
}

};
