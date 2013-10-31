#ifndef __CHAIN_BODY_COMPONENT_H
	#define __CHAIN_BODY_COMPONENT_H

#include "PtlBodyComponent.h"
#include <vector>

namespace Ptl
{

class OgrePhysicalBody;

class ChainBodyComponent : public BodyComponent
{
	public:
		ChainBodyComponent(Ogre::SceneManager *aSceneMgr,
				   btDiscreteDynamicsWorld *aWorld,
				   const Ogre::Vector3& aPos,
				   const Ogre::Quaternion& aOrient
				   );
		virtual ~ChainBodyComponent();
		virtual btRigidBody* getRootBody();
		virtual btVector3 getRootAnchor();

	protected:
		OgrePhysicalBody *mOgrePhysBody;
		btHingeConstraint *mChainHinge;
    		std::vector<Ptl::OgrePhysicalBody*> mChainElements;
};

};
	
#endif //__CHAIN_BODY_COMPONENT_H
