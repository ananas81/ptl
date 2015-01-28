#ifndef __RINGWHEELGEAR_BODY_COMPONENT_H
	#define __RINGWHEELGEAR_BODY_COMPONENT_H

#include "PtlBodyComponent.h"
#include "PtlRingwheelComponent.h"

namespace Ptl
{

class OgrePhysicalBody;
class SlotBlockerComponent;

class RingwheelGearBodyComponent : public RingwheelBodyComponent
{
	public:
		RingwheelGearBodyComponent(Ogre::SceneManager *aSceneMgr,
			      btDiscreteDynamicsWorld *aWorld,
			      const Ogre::Vector3& aPos,
			      const Ogre::Quaternion& aOrient
			     );

		virtual ~RingwheelGearBodyComponent() {};
};

};
	
#endif //__RINGWHEELGEAR_BODY_COMPONENT_H
