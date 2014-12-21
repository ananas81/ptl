#ifndef __RINGWHEEL_BODY_COMPONENT_H
	#define __RINGWHEEL_BODY_COMPONENT_H

#include "PtlBodyComponent.h"

namespace Ptl
{

class OgrePhysicalBody;
class SlotBlockerComponent;

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
		virtual btTransform getRootAnchor(int anchorId);
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor, int anchorId);
		virtual void setActivationState(int actState);
		virtual void switchToKinematic();
		virtual void switchToDynamic();
		virtual btHingeConstraint* getHinge();

		static const float RINGWHEEL_RADIUS = 20.0;
		static const float RINGWHEEL_WIDTH = 8.0;

	protected:
		OgrePhysicalBody *mRingwheel;
		OgrePhysicalBody *mRingweight;
		OgrePhysicalBody *mRearBlocker[4];
		OgrePhysicalBody *mFrontBlocker[4];
		OgrePhysicalBody *mLever;
		btHingeConstraint *mRingwheelHinge;
		btGeneric6DofConstraint *mLeverConstr;
		std::vector<BodyComponent*> mChildComponents;
		static int mRingwheelElementsCnt;
		static int mRingweightElementsCnt;
		static int mBlockerElementsCnt;
};

};
	
#endif //__RINGWHEEL_BODY_COMPONENT_H
