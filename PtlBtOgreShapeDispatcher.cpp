#include <btBulletWorldImporter.h>
#include "PtlBtOgreShapeDispatcher.h"
#include "BtOgreGP.h"
#include "PtlPerpetualCommon.h"

namespace Perpetual {

PtlBtOgreShapeDispatcher::PtlBtOgreShapeDispatcher(Ogre::Entity* ogreEntity, BulletShape btShape) :
				mOgreEntity(ogreEntity),
				mBtShape(btShape)
{
}

PtlBtOgreShapeDispatcher::~PtlBtOgreShapeDispatcher()
{
}

btCollisionShape* PtlBtOgreShapeDispatcher::getCollisionShape() const
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
