#ifndef __PTL_PERPETUAL_COMMON_H
        #define __PTL_PERPETUAL_COMMON_H

#include <stdio.h>
#include <btBulletDynamicsCommon.h>
#include <OgreQuaternion.h>

#define DEBUG 1

#ifdef DEBUG
	#define PTL_LOG(message) \
		printf("%s:%d %s\n", __func__, __LINE__, message);
#else
	#define PTL_LOG
#endif

namespace Ptl {

class Quaternion {

public:
	Quaternion(double w, double x, double y, double z) :
		mW(w), mX(x), mY(y), mZ(z) {}
	~Quaternion() {}

	operator Ogre::Quaternion()
	{
		return Ogre::Quaternion(mW, mX, mY, mZ);
	}

	operator btQuaternion()
	{
		return btQuaternion(mX, mY, mZ, mW);
	}

private:
	double mW;
	double mX;
	double mY;
	double mZ;
};

}

#endif //__PTL_PERPETUAL_COMMON_H

