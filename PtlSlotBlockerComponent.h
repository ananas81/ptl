#ifndef __SLOT_BLOCKER_COMPONENT_H
#define __SLOT_BLOCKER_COMPONENT_H

#include "PtlBodyComponent.h"

namespace Ptl
{

class OgrePhysicalBody;

class SlotBlockerComponent : public BodyComponent
{
	public:
		SlotBlockerComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     );

		virtual ~SlotBlockerComponent();
		virtual btRigidBody* getRootBody();
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual void setActivationState(int actState);
		virtual void switchToKinematic();
		virtual void switchToDynamic();

	protected:
		OgrePhysicalBody *mBlockerRack;
		OgrePhysicalBody *mBlockerArm;
		btGeneric6DofConstraint *mBlockerRackConstr;
		btGeneric6DofConstraint *mBlockerArmConstr;
		static int mBlockerRackElementsCnt;
		static int mBlockerArmElementsCnt;
};

};
	
#endif //__SLOT_BLOCKER_COMPONENT_H
