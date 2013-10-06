#ifndef __BULLET_IMPORTER_SHAPE_DISPATCHER_H
	#define __BULLET_IMPORTER_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
#include <string>
#include "CollisionShapeDispatcher.h"

namespace Perpetual
{

class BulletImporterShapeDispatcher : public CollisionShapeDispatcher
{
	public:
		BulletImporterShapeDispatcher(const std::string& bcsFileName, int shapeId);
		virtual ~BulletImporterShapeDispatcher();
		virtual btCollisionShape* getCollisionShape() const;

	private:
		std::string mBcsFileName;
		int mShapeId;
};

};
	
#endif //__BULLET_IMPORTER_SHAPE_DISPATCHER_H
