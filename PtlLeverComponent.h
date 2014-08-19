#ifndef __LEVER_BODY_COMPONENT_H
	#define __LEVER_BODY_COMPONENT_H

#include "PtlBodyComponent.h"

namespace Ptl
{

class OgrePhysicalBody;
class WheelBodyComponent;

class LeverBodyComponent : public BodyComponent
{
	public:
		LeverBodyComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     );

		virtual ~LeverBodyComponent();
		virtual btRigidBody* getRootBody();
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual void setActivationState(int actState);
		virtual void switchToKinematic();
		virtual void switchToDynamic();

		static const double LEVER_HEIGHT = 110.0;
		static const double LEVER_ARM_LENGTH = 20.0;

	protected:
		OgrePhysicalBody *mLever;
		btGeneric6DofConstraint *mLeverConstr;
		static int mLeverElementsCnt;
};

};
	
#endif //__LEVER_BODY_COMPONENT_H
