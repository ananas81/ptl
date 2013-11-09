#ifndef __WHEEL_BODY_COMPONENT_H
	#define __WHEEL_BODY_COMPONENT_H

#include "PtlBodyComponent.h"

namespace Ptl
{

class OgrePhysicalBody;

class WheelBodyComponent : public BodyComponent
{
	public:
		WheelBodyComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     );

		virtual ~WheelBodyComponent();
		virtual btRigidBody* getRootBody();
		virtual btRigidBody* getRackBody();
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual btHingeConstraint* getHinge();

	protected:
		OgrePhysicalBody *mWheel;
		OgrePhysicalBody *mRack;
		btHingeConstraint *mWheelHinge;
		std::vector<BodyComponent*> mChildComponents;
};

};
	
#endif //__WHEEL_BODY_COMPONENT_H
