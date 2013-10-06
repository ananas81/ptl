#include <btBulletWorldImporter.h>
#include "PtlBulletImporterShapeDispatcher.h"

namespace Perpetual {

PtlBulletImporterShapeDispatcher::PtlBulletImporterShapeDispatcher(const std::string& bcsFileName, int shapeId) :
				mBcsFileName(bcsFileName),
				mShapeId(shapeId)
{
}

PtlBulletImporterShapeDispatcher::~PtlBulletImporterShapeDispatcher()
{
}

btCollisionShape* PtlBulletImporterShapeDispatcher::getCollisionShape() const
{
	btBulletWorldImporter importer;
	importer.loadFile(mBcsFileName.c_str());
	return importer.getCollisionShapeByIndex(mShapeId);
}

};
