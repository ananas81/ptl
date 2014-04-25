#ifndef __RINGWHEEL_BODY_COMPONENT_H
	#define __RINGWHEEL_BODY_COMPONENT_H

#include "PtlBodyComponent.h"

namespace Ptl
{

class OgrePhysicalBody;

class RingwheelBodyComponent : public BodyComponent
{
	public:
		RingwheelBodyComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     );

		virtual ~RingwheelBodyComponent();
		virtual btRigidBody* getRootBody();
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual void setActivationState(int actState);
		virtual void switchToKinematic();
		virtual void switchToDynamic();
		virtual btHingeConstraint* getHinge();

		static const float RINGWHEEL_RADIUS = 20.0;
		static const float RINGWHEEL_WIDTH = 4.0;

	protected:
		OgrePhysicalBody *mRingwheel;
		OgrePhysicalBody *mRingweight;
		OgrePhysicalBody *mRearBlocker[3];
		OgrePhysicalBody *mFrontBlocker[3];
		btHingeConstraint *mRingwheelHinge;
		std::vector<BodyComponent*> mChildComponents;
		static int mRingwheelElementsCnt;
};

};
	
#endif //__RINGWHEEL_BODY_COMPONENT_H
