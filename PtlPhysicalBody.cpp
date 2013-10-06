#include "PtlPhysicalBody.h"

namespace Perpetual
{

PtlPhysicalBody::PtlPhysicalBody(Ogre::Entity* visualObject,
				 btCollisionShape* shape,
				 const Vector& pos,
				 const Quaternion& quat,
				 double mass,
				 const Vector& inertia) : 
				 mVisualObject(visualObject),
				 mShape(shape),
				 mPos(pos),
				 mMass(mass),
				 mIntertia(inertia),
				 mCollisionObject(NULL)
{ 
}

PtlPhysicalBody::~PtlPhysicalBody()
{
}

btCollisionObject* PtlPhysicalBody::getCollisionObject() const
{
	return mCollisionObject;
}

Ogre::Entity* PtlPhysicalBody::getVisualObject() const
{
	return mVisualObject;
}

};
