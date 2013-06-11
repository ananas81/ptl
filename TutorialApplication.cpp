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

#define BULLET_TRIANGLE_COLLISION 1

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
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
    mEntity = mSceneMgr->createEntity("Head", "Cylinder.mesh");
    mEntity->setCastShadows(true);
//    Ogre::Entity* ogreHead = mSceneMgr->createEntity("Head", "ogrehead.mesh");
 
    // Create a SceneNode and attach the Entity to it
    mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("HeadNode");
    mNode->attachObject(mEntity);
    mNode->translate( Ogre::Vector3( 25, 350, 0 ) );
    mNode->roll(Ogre::Degree(80));
    mNode->scale( .3, .3, .3 );

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

    preparePhysics(mEntity, mNode, mEntity2, mNode2);
}

void TutorialApplication::preparePhysics(Ogre::Entity* entity,
					 Ogre::SceneNode* node,
					 Ogre::Entity* entity2,
					 Ogre::SceneNode* node2)
{
        btBroadphaseInterface* broadphase = new btDbvtBroadphase();

        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

        mWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

        mWorld->setGravity(btVector3(0,-10,0));

        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

	//Create shape.
	btBulletWorldImporter importer;
	importer.loadFile("mugCollisionShape.bcs");
	btCollisionShape * fallShape = importer.getCollisionShapeByIndex(0);
//	BtOgre::StaticMeshToShapeConverter converter(entity);
//	btCollisionShape* fallShape = converter.createTrimesh(); //You can also just use btSphereShape(1.2) or something.

	fallShape->setMargin(1.1f);
	
	//Create BtOgre MotionState (connects Ogre and Bullet).
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	groundRigidBodyCI.m_friction = 10;
	groundRigidBodyCI.m_rollingFriction = 1;
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	groundRigidBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
	groundRigidBody->setCollisionFlags(groundRigidBody->getFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);
        mWorld->addRigidBody(groundRigidBody);

        mFallMotionState =
                new MyMotionState(btTransform(btQuaternion(70,100,150,1),btVector3(25,350,0)), node);
        btScalar mass = 10;
        btVector3 fallInertia(0,0,0);
        fallShape->calculateLocalInertia(mass,fallInertia);
        btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,mFallMotionState,fallShape,fallInertia);
	fallRigidBodyCI.m_friction = 10;
	fallRigidBodyCI.m_rollingFriction = 1;
        btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
        mWorld->addRigidBody(fallRigidBody);

	//Create shape.
	btBulletWorldImporter importer2;
	importer2.loadFile("mugCollisionShape.bcs");
	btCollisionShape * staticShape = importer2.getCollisionShapeByIndex(0);
	//BtOgre::StaticMeshToShapeConverter converter2(entity2);
	//btCollisionShape* staticShape = converter2.createTrimesh(); //You can also just use btSphereShape(1.2) or something.
	staticShape->setMargin(1.1f);

        mStaticMotionState =
                new MyMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(25,0,0)), node2);
        btScalar mass2 = 0.1;
        btVector3 staticInertia(0,0,0);
        staticShape->calculateLocalInertia(mass2,staticInertia);
        btRigidBody::btRigidBodyConstructionInfo staticRigidBodyCI(mass2,mStaticMotionState,staticShape,staticInertia);
	staticRigidBodyCI.m_friction = 10;
	staticRigidBodyCI.m_rollingFriction = 1;
        btRigidBody* staticRigidBody = new btRigidBody(staticRigidBodyCI);
	staticRigidBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
        mWorld->addRigidBody(staticRigidBody);
}

void TutorialApplication::createFrameListener(void){
    BaseApplication::createFrameListener();
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
	mWorld->stepSimulation(evt.timeSinceLastFrame,50);
	
        return BaseApplication::frameRenderingQueued(evt);
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
