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
		enum ChainDirection
		{
			DIR_LEFT = 0,
			DIR_UP,
			DIR_RIGHT,
			DIR_DOWN
		};

		ChainBodyComponent(Ogre::SceneManager *aSceneMgr,
				   btDiscreteDynamicsWorld *aWorld,
				   const Ogre::Vector3& aPos,
				   const Ogre::Quaternion& aOrient,
				   ChainDirection aDirection
				   );
		virtual ~ChainBodyComponent();
		virtual btRigidBody* getRootBody();
		virtual btTransform getRootAnchor();
		virtual void attachTo(btRigidBody* parentComponent, const btTransform& parentAnchor);
		virtual Ogre::Vector3 calculateChainElementPos(int elementId);
		virtual Ogre::Vector3 calculateWeightPos(int elementId);
		virtual void setRotationalMotor(btGeneric6DofConstraint* dofConstraint);

		static const double CHAIN_ELEMENT_RADIUS = 6.6;
		static const double WEIGHT_RADIUS = 6.12;

	protected:
		ChainDirection mDirection;
		OgrePhysicalBody *mOgrePhysBody;
		btHingeConstraint *mChainHinge;
    		std::vector<Ptl::OgrePhysicalBody*> mChainElements;
		static int mChainElementsCnt;
};

};
	
#endif //__CHAIN_BODY_COMPONENT_H
