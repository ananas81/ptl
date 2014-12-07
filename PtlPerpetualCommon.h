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
	Quaternion(Ogre::Quaternion &q) :
		mW(q.w), mX(q.x), mY(q.y), mZ(q.z) {}
	Quaternion(btQuaternion &q) :
		mW(q.getW()), mX(q.getX()), mY(q.getY()), mZ(q.getX()) {}
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

class Vector3 {

public:
	Vector3(double x, double y, double z) :
		mX(x), mY(y), mZ(z) {}
	~Vector3() {}

	operator Ogre::Vector3()
	{
		return Ogre::Vector3(mX, mY, mZ);
	}

	operator btVector3()
	{
		return btVector3(mX, mY, mZ);
	}

private:
	double mX;
	double mY;
	double mZ;
};

}

#endif //__PTL_PERPETUAL_COMMON_H

