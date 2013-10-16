#ifndef __COLLISION_SHAPE_DISPATCHER_DATA_H
	#define __COLLISION_SHAPE_DISPATCHER_DATA_H

#include <btBulletDynamicsCommon.h>
#include <OgreEntity.h>
#include <string>
#include "PtlBtOgreShapeDispatcher.h"

namespace Ptl
{

class CollisionShapeDispatcherData
{
	public:
		virtual ~CollisionShapeDispatcherData() {}
};

class BulletImporterDispatcherData : public CollisionShapeDispatcherData
{
	public:
		BulletImporterDispatcherData(const std::string& bcsFilename, int shapeId) :
							mBcsFilename(bcsFilename),
							mShapeId(shapeId)					
		{
		}

		virtual ~BulletImporterDispatcherData() {}

		virtual std::string getBcsFilename()
		{
			return mBcsFilename;
		}

		virtual int getShapeId()
		{
			return mShapeId;
		}

	private:
		std::string mBcsFilename;
		int mShapeId;
};

class BtOgreDispatcherData : public CollisionShapeDispatcherData
{
	public:
		BtOgreDispatcherData(BtOgreShapeDispatcher::BulletShape btShape) :
							mBtShape(btShape)
		{
		}

		virtual ~BtOgreDispatcherData() {}

		virtual BtOgreShapeDispatcher::BulletShape getBtShape()
		{
			return mBtShape;
		}

		virtual void setOgreEntity(Ogre::Entity* entity)
		{
			mOgreEntity = entity;
		}

		virtual Ogre::Entity* getOgreEntity()
		{
			return mOgreEntity;
		}

	private:
		Ogre::Entity* mOgreEntity;
		BtOgreShapeDispatcher::BulletShape mBtShape;
};

};
	
#endif //__COLLISION_SHAPE_DISPATCHER_DATA_H
