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
		virtual btVector3 getRootAnchor();

	protected:
		OgrePhysicalBody *mOgrePhysBody;
		btHingeConstraint *mWheelHinge;
		std::vector<BodyComponent*> mChildComponents;
};

};
	
#endif //__WHEEL_BODY_COMPONENT_H
