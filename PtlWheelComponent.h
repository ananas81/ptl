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
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual void setActivationState(int actState);
		virtual void switchToKinematic();
		virtual void switchToDynamic();
		virtual btHingeConstraint* getHinge();

		static const float WHEEL_RADIUS = 40.0;
		static const float WHEEL_WIDTH = 8.0;

	protected:
		OgrePhysicalBody *mWheel;
		btHingeConstraint *mWheelHinge;
		std::vector<BodyComponent*> mChildComponents;
		static int mWheelElementsCnt;
};

};
	
#endif //__WHEEL_BODY_COMPONENT_H
