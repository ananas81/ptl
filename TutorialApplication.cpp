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
#include <OgreSimpleSpline.h>


#define BULLET_TRIANGLE_COLLISION 1

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void) :
bLMouseDown(false),
bRMouseDown(false),
mCurrentObject(NULL),
mRayScnQuery(NULL),
mGUIRenderer(NULL),
mFallRigidBody(NULL),
mStaticRigidBody(NULL),
mWheelHinge(NULL)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.0f, 0.0f, 0.5f));
    mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

    mWalkList.push_back(Ogre::Vector3(550.0f,  20.0f,  50.0f ));
    mWalkList.push_back(Ogre::Vector3(-100.0f,  150.0f, -200.0f));
 
    // Create an Entity
    mEntity = mSceneMgr->createEntity("Head", "Flywheel.mesh");
    mEntity->setCastShadows(true);
//    Ogre::Entity* ogreHead = mSceneMgr->createEntity("Head", "ogrehead.mesh");
 
    // Create a SceneNode and attach the Entity to it
    mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("HeadNode");
    mNode->attachObject(mEntity);
    mNode->translate( Ogre::Vector3( 25, 150, 0 ) );
//    mNode->roll(Ogre::Degree(80));
//    mNode->scale( .3, .3, .3 );

    mEntity2 = mSceneMgr->createEntity( "Head2", "Cylinder.mesh" );
    mEntity2->setCastShadows(true);
    //Ogre::SceneNode* headNode2 = headNode->createChildSceneNode( "HeadNode2", Ogre::Vector3( 100, 0, 0 ) );
    mNode2 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "HeadNode2", Ogre::Vector3( 25, 0, 0 ) );
    mNode2->attachObject(mEntity2);
//    headNode2->translate( Ogre::Vector3( 10, 0, 10 ) );

//    mNode2->yaw(Ogre::Degree(45));
//    mNode2->pitch(Ogre::Degree(45));
//    mNode2->roll(Ogre::Degree(45));
 
    // Create a Light and set its position
 //   Ogre::Light* light = mSceneMgr->createLight("MainLight");
 //   light->setPosition(20.0f, 80.0f, 50.0f);
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

//------------rope-------------
    mRopeObject =  mSceneMgr->createManualObject("manual1"); 
    mRopeObjectNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("manual1_node");

    Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create("manual1Material","General"); 
    myManualObjectMaterial->setReceiveShadows(false); 
    myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0); 
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1); 
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1); 

    mRopeObjectNode->attachObject(mRopeObject);
//------------rope-------------

    //CEGUI setup
    mGUIRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
 
    //show the CEGUI cursor
    CEGUI::SchemeManager::getSingleton().create((CEGUI::utf8*)"TaharezLook.scheme");
    CEGUI::MouseCursor::getSingleton().setImage("TaharezLook", "MouseArrow");

    preparePhysics(mEntity, mNode, mEntity2, mNode2);
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

void TutorialApplication::preparePhysics(Ogre::Entity* entity,
					 Ogre::SceneNode* node,
					 Ogre::Entity* entity2,
					 Ogre::SceneNode* node2)
{
	const int maxProxies = 32766;
	btVector3 worldAabbMin(-1000,-1000,-1000);
        btVector3 worldAabbMax(1000,1000,1000);
        
        btBroadphaseInterface* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	m_softBodyWorldInfo.m_broadphase = broadphase;

        btDefaultCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
        m_softBodyWorldInfo.m_dispatcher = dispatcher;

        btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

        //m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback,this,true);

        mWorld = new btSoftRigidDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration,NULL);

//        mWorld->setGravity(btVector3(0,-10,0));
        mWorld->getDispatchInfo().m_enableSPU = true;
        mWorld->setGravity(btVector3(0,-10,0));
	mWorld->setInternalTickCallback(pickingPreTickCallback,this,true);
        m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

        //      clientResetScene();

        m_softBodyWorldInfo.m_sparsesdf.Initialize();

        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

	//Create shape.
	btBulletWorldImporter importer;
	importer.loadFile("flywheel.bcs");
	btCollisionShape * fallShape = importer.getCollisionShapeByIndex(0);
//	BtOgre::StaticMeshToShapeConverter converter(entity);
//	btCollisionShape* fallShape = converter.createConvex(); //You can also just use btSphereShape(1.2) or something.

//	fallShape->setMargin(1.1f);
	
	//Create BtOgre MotionState (connects Ogre and Bullet).
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	groundRigidBodyCI.m_friction = 10;
	groundRigidBodyCI.m_rollingFriction = 1;
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
//	groundRigidBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
//	groundRigidBody->setCollisionFlags(groundRigidBody->getFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);
        mWorld->addRigidBody(groundRigidBody);

        mFallMotionState =
                new MyMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(25,150,0)), node);
        btScalar mass = 4.0;
        btVector3 fallInertia(0,0,0);
        fallShape->calculateLocalInertia(mass,fallInertia);
        btRigidBody::btRigidBodyConstructionInfo mFallRigidBodyCI(mass,mFallMotionState,fallShape,fallInertia);
	mFallRigidBodyCI.m_friction = 10;
	mFallRigidBodyCI.m_rollingFriction = 1;
        mFallRigidBody = new btRigidBody(mFallRigidBodyCI);
	mFallRigidBody->setLinearFactor(btVector3(0,0,0));
//	mFallRigidBody->setAngularFactor(btVector3(1,0,0));
        mWorld->addRigidBody(mFallRigidBody);
	mWorldObjects.push_back(new WorldObject(mNode, mFallRigidBody));

	//Create shape.
	btBulletWorldImporter importer2;
	importer2.loadFile("Cylinder.bcs");
	btCollisionShape * staticShape = importer2.getCollisionShapeByIndex(0);
//	BtOgre::StaticMeshToShapeConverter converter2(entity2);
//	btCollisionShape* staticShape = converter2.createConvex(); //You can also just use btSphereShape(1.2) or something.
//	staticShape->setMargin(1.1f);

        mStaticMotionState =
                new MyMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(25,0,0)), node2);
        btScalar mass2 = 10.0;
        btVector3 staticInertia(0,0,0);
        staticShape->calculateLocalInertia(mass2,staticInertia);
        btRigidBody::btRigidBodyConstructionInfo mStaticRigidBodyCI(mass2,mStaticMotionState,staticShape,staticInertia);
	mStaticRigidBodyCI.m_friction = 10;
	mStaticRigidBodyCI.m_rollingFriction = 1;
        mStaticRigidBody = new btRigidBody(mStaticRigidBodyCI);
//	mStaticRigidBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
        mWorld->addRigidBody(mStaticRigidBody);
	mWorldObjects.push_back(new WorldObject(mNode2, mStaticRigidBody));

        mRope = btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, btVector3(25, 111, 0), btVector3(25,0,0), 15, 0);
        mRope->setTotalMass(2);
        mRope->m_cfg.kCHR = 1;
//        mRope->m_cfg.kKHR = 0;
//        mRope->m_cfg.kLF = 0;
//        mRope->m_cfg.kDG = 0;
//        mRope->m_cfg.kDP = 0;
//        mRope->m_cfg.kAHR = 0;
//        mRope->m_cfg.kSRHR_CL = 1;
        mRope->m_cfg.piterations = 10;
//        mRope->m_cfg.citerations = 10;
//	mRope->m_cfg.collisions	= btSoftBody::fCollision::CL_SS | btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::SDF_RS;
        getSoftDynamicsWorld()->addSoftBody(mRope);
        mRope->appendAnchor(0, mFallRigidBody);
        mRope->appendAnchor(mRope->m_nodes.size()-1, mStaticRigidBody);

	mWheelHinge = new btHingeConstraint(*mFallRigidBody,btVector3(0,0,0),btVector3(0,0,1),true);
//	mFallRigidBody->setAngularVelocity(btVector3(0,0,5));
	mFallRigidBody->setFriction(1);
	mFallRigidBody->setDamping(0.01f,0.01f);
	mFallRigidBody->setFlags(0);
	//mFallRigidBody->setLinearFactor(btVector3(0.5, 0.5, 0.5));
	//mFallRigidBody->setAngularFactor(btVector3(0.5, 0.5, 0.5));
	//mFallRigidBody->applyTorqueImpulse(btVector3(1,1,3));
	//mFallRigidBody->applyCentralImpulse(btVector3(1,1,3));
	//mFallRigidBody->applyForce(btVector3(25,111,100), btVector3(20, 111, 0));
	//mFallRigidBody->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);
	//mFallRigidBody->applyTorque(btVector3(20, 111, 10));

	mWorld->addConstraint(mWheelHinge);

}

void TutorialApplication::createFrameListener(void){
    //we still want to create the frame listener from the base app
    BaseApplication::createFrameListener();

    //but we also want to set up our raySceneQuery after everything has been initialized
    mRayScnQuery = mSceneMgr->createRayQuery(Ogre::Ray());

    // Set default values for variables
    mWalkSpeed = 150.0f;
    mDirection = Ogre::Vector3::ZERO;
}

bool TutorialApplication::nextLocation(void){
    static int i = 0;
    i = !i;
    mDestination = mWalkList[i];  // this gets the front of the deque
    mDirection = mDestination - mNode->getPosition();
    mDistance = mDirection.normalise();
    return true;
}


bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent &evt){
//	mWorld->stepSimulation(1/60.f,10);

	btSoftBody::tLinkArray& links(mRope->m_links);
	Ogre::SimpleSpline spline;

	spline.setAutoCalculate(false);

	for(int j=0;j<links.size();++j)
	{
		btSoftBody::Node*   node_0=links[j].m_n[0];
		btSoftBody::Node*   node_1=links[j].m_n[1];

		spline.addPoint(Ogre::Vector3(node_0->m_x.getX(), node_0->m_x.getY(), node_0->m_x.getZ()));
		if (j == links.size() - 1)
			spline.addPoint(Ogre::Vector3(node_1->m_x.getX(), node_1->m_x.getY(), node_1->m_x.getZ()));

	}
	spline.recalcTangents();

	mRopeObject->clear();
        //mRopeObject->begin("manual1Material", Ogre::RenderOperation::OT_LINE_LIST); 
        mRopeObject->begin("manual1Material", Ogre::RenderOperation::OT_LINE_STRIP); 
//	printf("rope segments:\n");
	for(int s=0;s<spline.getNumPoints();++s)
	{
		for(float i = 0.0; i <= 1.0; i += 0.1) {
			Ogre::Vector3 point = spline.interpolate(s, i);
			mRopeObject->position(point.x, point.y, point.z); 
		//	printf("curIntPoint: x: %2.2f, y: %2.2f, z: %2.2f\n", point.x, point.y, point.z);
		}
	}

	mRopeObject->end();

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

	switch (evt.key) {
	case OIS::KC_1: 
	        motorOn = !motorOn;
		if (motorOn)
			mWheelHinge->enableAngularMotor(true, 20, 5);
		else
			mWheelHinge->enableMotor(false);

        	break;
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
