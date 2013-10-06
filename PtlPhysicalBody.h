#ifndef __PHYSICAL_BODY_DISPATCHER_H
	#define __PHYSICAL_BODY_DISPATCHER_H

#include <btBulletDynamicsCommon.h>
//#include <btCollisionObject.h>
//#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <OgreEntity.h>

namespace Perpetual
{

class PtlPhysicalBody
{
	public:
		struct Vector
		{
			double x;
			double y;
			double z;
		};

		struct Quaternion
		{
			double w;
			double x;
			double y;
			double z;
		};

		PtlPhysicalBody(Ogre::Entity* visualObject,
				btCollisionShape* shape,
				btRigidBody::btRigidBodyConstructionInfo constrInfo,
				const Vector& pos,
				const Quaternion& quat,
				double mass,
				const Vector& intertia);
		virtual ~PtlPhysicalBody();
		virtual btCollisionObject* getCollisionObject() const;
		virtual Ogre::Entity* getVisualObject() const;

	private:
		Ogre::Entity* mVisualObject;
		btCollisionShape *mShape;
		btRigidBody::btRigidBodyConstructionInfo mConstrInfo;
		Vector mPos;
		Quaternion mQuat;
		double mMass;
		Vector mIntertia;
		btCollisionObject *mCollisionObject;
};

};
	
#endif //__PHYSICAL_BODY_DISPATCHER_H
