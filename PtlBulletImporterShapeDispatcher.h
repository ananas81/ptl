#ifndef __BULLET_IMPORTER_SHAPE_DISPATCHER_H
	#define __BULLET_IMPORTER_SHAPE_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
#include <string>
#include "PtlCollisionShapeDispatcher.h"

namespace Ptl
{

class PtlBulletImporterShapeDispatcher : public CollisionShapeDispatcher
{
	public:
		PtlBulletImporterShapeDispatcher(const std::string& bcsFileName, int shapeId);
		virtual ~PtlBulletImporterShapeDispatcher();
		virtual btCollisionShape* getCollisionShape() const;

	private:
		std::string mBcsFileName;
		int mShapeId;
};

};
	
#endif //__BULLET_IMPORTER_SHAPE_DISPATCHER_H
