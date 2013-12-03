#ifndef __RACK_BODY_COMPONENT_H
	#define __RACK_BODY_COMPONENT_H

#include "PtlBodyComponent.h"

namespace Ptl
{

class OgrePhysicalBody;
class WheelBodyComponent;

class RackBodyComponent : public BodyComponent
{
	public:
		RackBodyComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     );

		virtual ~RackBodyComponent();
		virtual btRigidBody* getRootBody();
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual btGeneric6DofConstraint* getRail();
		virtual btHingeConstraint* getHinge();
		virtual void setActivationState(int actState);
		virtual void switchToKinematic();
		virtual void switchToDynamic();
		virtual void lockPosition(bool lock);

		static const double RACK_HEIGHT = 8.52;
		static const double RACK_ARM_LENGTH = 7.52;

	protected:
		OgrePhysicalBody *mRack;
		WheelBodyComponent *mFlywheel;
		btGeneric6DofConstraint *mRail;
		static int mRackElementsCnt;
};

};
	
#endif //__RACK_BODY_COMPONENT_H
