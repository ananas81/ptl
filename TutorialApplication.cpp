/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
	  |___/                              
	  Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "TutorialApplication.h"
#include "PtlCollisionShapeDispatcher.h"
#include "PtlBulletImporterShapeDispatcher.h"
#include "PtlBtOgreShapeDispatcher.h"
#include "PtlCollisionShapeDispatcherData.h"


#define BULLET_TRIANGLE_COLLISION 1

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void) :
bLMouseDown(false),
bRMouseDown(false),
mCurrentObject(NULL),
mRayScnQuery(NULL),
mGUIRenderer(NULL),
mFlywheel1(NULL),
mFlywheel2(NULL),
mRopeSphere(NULL),
mDebugDrawer(NULL)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
	btCollisionShape *shape;

	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.0f, 0.0f, 0.5f));
	mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

	Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
	pointLight->setType(Ogre::Light::LT_POINT);
	pointLight->setPosition(Ogre::Vector3(0, 150, 250));
	pointLight->setDiffuseColour(1.0, 0.0, 0.0);
	pointLight->setSpecularColour(1.0, 0.0, 0.0);

	Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
	directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
	directionalLight->setDiffuseColour(Ogre::ColourValue(.25, .25, 0));
	directionalLight->setSpecularColour(Ogre::ColourValue(.25, .25, 0));
	directionalLight->setDirection(Ogre::Vector3( 0, -1, 1 ));

	Ogre::Light* spotLight = mSceneMgr->createLight("spotLight");
	spotLight->setType(Ogre::Light::LT_SPOTLIGHT);
	spotLight->setDiffuseColour(0, 0, 1.0);
	spotLight->setSpecularColour(0, 0, 1.0);
	spotLight->setDirection(-1, -1, 0);
	spotLight->setPosition(Ogre::Vector3(300, 300, 0));
	spotLight->setSpotlightRange(Ogre::Degree(35), Ogre::Degree(50));

	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
 
	Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
 
	Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
 
	entGround->setMaterialName("Examples/Rockwall");
	entGround->setCastShadows(false);

	//CEGUI setup
	mGUIRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
 
	//show the CEGUI cursor
	CEGUI::SchemeManager::getSingleton().create((CEGUI::utf8*)"TaharezLook.scheme");
	CEGUI::MouseCursor::getSingleton().setImage("TaharezLook", "MouseArrow");

	initPhysics();
}

void TutorialApplication::pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	TutorialApplication* ta = (TutorialApplication*)world->getWorldUserInfo();

	btRigidBody* rigidBody = ta->getRigidBodyByNode(ta->mCurrentObject);
	
	if (!rigidBody) {
//		printf("Corresponding rigid body not found\n");
		return;
	}
	
	btTransform transform = rigidBody->getCenterOfMassTransform();
	transform.setOrigin(btVector3(ta->mCurHitPoint.x, ta->mCurHitPoint.y, ta->mCurHitPoint.z));
	rigidBody->setCenterOfMassTransform(transform);
}

void TutorialApplication::initPhysics()
{
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	mWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

	mWorld->getDispatchInfo().m_enableSPU = true;
	mWorld->setGravity(btVector3(0,-10,0));
	mWorld->setInternalTickCallback(pickingPreTickCallback,this,true);

	//      clientResetScene();

	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	groundRigidBodyCI.m_friction = 10;
	groundRigidBodyCI.m_rollingFriction = 1;
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	mWorld->addRigidBody(groundRigidBody);

	mFlywheel1 = new Ptl::WheelBodyComponent(mSceneMgr,
						 mWorld,
						 Ogre::Vector3(25, 170, 0),
						 Ogre::Quaternion(1, 0, 0, 0));

	mDebugDrawer = new DebugDrawer(mSceneMgr, mWorld);
	mDebugDrawer->setDebugMode(0);
	mWorld->setDebugDrawer(mDebugDrawer);
}

void TutorialApplication::createFrameListener(void){
	//we still want to create the frame listener from the base app
	BaseApplication::createFrameListener();

	//but we also want to set up our raySceneQuery after everything has been initialized
	mRayScnQuery = mSceneMgr->createRayQuery(Ogre::Ray());
}

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent &evt){
//	mWorld->stepSimulation(1/60.f,10);

	mWorld->stepSimulation(evt.timeSinceLastFrame,50);
	
	if (!BaseApplication::frameRenderingQueued(evt))
		return false;

	/* 
	This next big chunk basically sends a raycast straight down from the camera's position 
	It then checks to see if it is under world geometry and if it is we move the camera back up 
	*/ 
	Ogre::Vector3 camPos = mCamera->getPosition(); 
	Ogre::Ray cameraRay(Ogre::Vector3(camPos.x, 5000.0f, camPos.z), Ogre::Vector3::NEGATIVE_UNIT_Y); 
  
	mRayScnQuery->setRay(cameraRay);

	/*
	here we tell it not to sort the raycast results world geometry would be 
	at the end of the list so sorting would be bad in this case since we are iterating through everything
	*/
	mRayScnQuery->setSortByDistance(false);
	Ogre::RaySceneQueryResult& result = mRayScnQuery->execute();
	Ogre::RaySceneQueryResult::iterator iter = result.begin();

	for(iter; iter != result.end(); iter++)
	{
	        if(iter->worldFragment)
	        {
	                //gets the results, fixes camera height and breaks the loop
	                Ogre::Real terrainHeight = iter->worldFragment->singleIntersection.y;

	                if((terrainHeight + 10.0f) > camPos.y)
	                {
	                        mCamera->setPosition(camPos.x, terrainHeight + 10.0f, camPos.z);
	                }
	                break;
	        }
	}

	mDebugDrawer->Update();

	return true;
}

btRigidBody* TutorialApplication::getRigidBodyByNode(Ogre::SceneNode *node)
{
	if (!node)
	return NULL;

	for (int i = 0; i < mWorldObjects.size(); ++i) {
	if (mWorldObjects[i]->first == node)
	    return mWorldObjects[i]->second;
	}

	return NULL;
}
 
bool TutorialApplication::mouseMoved(const OIS::MouseEvent& arg)
{
	//updates CEGUI with mouse movement
	CEGUI::System::getSingleton().injectMouseMove(arg.state.X.rel, arg.state.Y.rel);

	//if the left mouse button is held down
	if(bLMouseDown)
	{
	        //find the current mouse position
	        CEGUI::Point mousePos = CEGUI::MouseCursor::getSingleton().getPosition();

	        //create a raycast straight out from the camera at the mouse's location
	        Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(mousePos.d_x/float(arg.state.width), mousePos.d_y/float(arg.state.height));
	        mRayScnQuery->setRay(mouseRay);
	        mRayScnQuery->setSortByDistance(false); //world geometry is at the end of the list if we sort it, so lets not do that

	        Ogre::RaySceneQueryResult& result = mRayScnQuery->execute();
	        Ogre::RaySceneQueryResult::iterator iter = result.begin();

	        //check to see if the mouse is pointing at the world and put our current object at that location
		printf("moving with pressed bl\n");
	        for(iter; iter != result.end(); iter++)
	        {
			printf("iterating\n");
	                //if(iter->worldFragment)
	                if(iter->movable && iter->movable->getName().substr(0, 5) != "tile[")
	                {
				printf("setting cur obj pos\n");
	                        //mCurrentObject->setPosition(iter->worldFragment->singleIntersection);
				mCurHitPoint = mouseRay.getOrigin() + mouseRay.getDirection() * iter->distance;
				
	                        //mCurrentObject->setPosition(hitPoint);
	                        break;
	                }
	        }
	}
	else if(bRMouseDown)    //if the right mouse button is held down, be rotate the camera with the mouse
	{
	        mCamera->yaw(Ogre::Degree(-arg.state.X.rel * 0.1));
	        mCamera->pitch(Ogre::Degree(-arg.state.Y.rel * 0.1));
	}

	return true;
}

bool TutorialApplication::mousePressed(const OIS::MouseEvent& arg, OIS::MouseButtonID id)
{
	if(id == OIS::MB_Left)
	{
	        //show that the current object has been deselected by removing the bounding box visual
	        if(mCurrentObject)
	        {
	                mCurrentObject->showBoundingBox(false);
	        }

	        //find the current mouse position
	        CEGUI::Point mousePos = CEGUI::MouseCursor::getSingleton().getPosition();

	        //then send a raycast straight out from the camera at the mouse's position
	        Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(mousePos.d_x/float(arg.state.width), mousePos.d_y/float(arg.state.height));
	        mRayScnQuery->setRay(mouseRay);
	        mRayScnQuery->setSortByDistance(true);
	        mRayScnQuery->setQueryMask(ROBOT_MASK);       //will return objects with the query mask in the results

	        /*
	        This next chunk finds the results of the raycast
	        If the mouse is pointing at world geometry we spawn a robot at that position
	        */
	        Ogre::RaySceneQueryResult& result = mRayScnQuery->execute();
	        Ogre::RaySceneQueryResult::iterator iter = result.begin();

	        for(iter; iter != result.end(); iter++)
	        {
	                //if you clicked on a robot or ninja it becomes selected
	                if(iter->movable && iter->movable->getName().substr(0, 5) != "tile[")
	                {
	                        mCurrentObject = iter->movable->getParentSceneNode();
				printf(">>>> hit movable object !!!\n");
	                        break;
	                }
	                //otherwise we spawn a new one at the mouse location
	                else if(iter->worldFragment)
			{
				printf(">>>> hit world fragment !!!\n");
	                }
			else
			{
				printf(">>>> missed !!!\n");
			}
	        }

	        //now we show the bounding box so the user can see that this object is selected
	        if(mCurrentObject)
	        {
	                mCurrentObject->showBoundingBox(true);
	        }

	        bLMouseDown = true;
	}
	else if(id == OIS::MB_Right)    // if the right mouse button is held we hide the mouse cursor for view mode
	{
	        CEGUI::MouseCursor::getSingleton().hide();
	        bRMouseDown = true;
	}

	return true;
}

bool TutorialApplication::mouseReleased(const OIS::MouseEvent& arg, OIS::MouseButtonID id)
{
	if(id  == OIS::MB_Left)
	{
	        bLMouseDown = false;
	}
	else if(id == OIS::MB_Right)    //when the right mouse is released we then unhide the cursor
	{
	        CEGUI::MouseCursor::getSingleton().show();
	        bRMouseDown = false;
	}
	return true;
}

bool TutorialApplication::keyPressed(const OIS::KeyEvent& evt)
{
	printf(">>> key pressed\n");
	static bool motorOn = false;

	switch (evt.key)
	{
		case OIS::KC_1:
		{
			motorOn = !motorOn;
			if (motorOn)
				//flywheel1->setAngularVelocity(btVector3(0, 0, 5));
				mFlywheel1->getHinge()->enableAngularMotor(true, 4000, 2000);
			else
				//flywheel1->setAngularVelocity(btVector3(0, 0, 0));
				mFlywheel1->getHinge()->enableMotor(false);
	
			break;
		}
		case OIS::KC_2:
		{
			btRigidBody *rackBody = mFlywheel1->getRackBody();
			rackBody->setActivationState(DISABLE_DEACTIVATION);
			rackBody->applyImpulse(btVector3(-50, 0, 0), btVector3(0., -170., 0.));
			break;
		}
		case OIS::KC_3:
		{
			btRigidBody *rackBody = mFlywheel1->getRackBody();
			rackBody->setActivationState(DISABLE_DEACTIVATION);
			rackBody->applyImpulse(btVector3(50, 0, 0), btVector3(0., -170., 0.));
			break;
		}
		default:
			break;
	}

	//then we return the base app keyPressed function so that we get all of the functionality
	//and the return value in one line
	return BaseApplication::keyPressed(evt);
}




#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
	INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
	int main(int argc, char *argv[])
#endif
	{
	// Create application object
	TutorialApplication app;

	try {
	    app.go();
	} catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
	    MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
	    std::cerr << "An exception has occured: " <<
	        e.getFullDescription().c_str() << std::endl;
#endif
	}

	return 0;
	}

#ifdef __cplusplus
}
#endif
