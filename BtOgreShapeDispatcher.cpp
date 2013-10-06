#include <btBulletWorldImporter.h>
#include "BtOgreShapeDispatcher.h"
#include "BtOgreGP.h"
#include "PerpetualCommon.h"

namespace Perpetual {

BtOgreShapeDispatcher::BtOgreShapeDispatcher(Ogre::Entity* ogreEntity, BulletShape btShape) :
				mOgreEntity(ogreEntity),
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
			PERPET_LOG("Unsupported bullet shape");
	}

	return shape;
}

};
