#include <btBulletWorldImporter.h>
#include "BulletImporterShapeDispatcher.h"

namespace Perpetual {

BulletImporterShapeDispatcher::BulletImporterShapeDispatcher(const std::string& bcsFileName, int shapeId) :
				mBcsFileName(bcsFileName),
				mShapeId(shapeId)
{
}

BulletImporterShapeDispatcher::~BulletImporterShapeDispatcher()
{
}

btCollisionShape* BulletImporterShapeDispatcher::getCollisionShape() const
{
	btBulletWorldImporter importer;
	importer.loadFile(mBcsFileName.c_str());
	return importer.getCollisionShapeByIndex(mShapeId);
}

};
