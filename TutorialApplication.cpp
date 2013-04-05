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

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//void TutorialApplication::createScene(void)
//{
//    mSceneMgr->setAmbientLight(Ogre::ColourValue(0, 0, 0));
//    mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
// 
//    Ogre::Entity* entNinja = mSceneMgr->createEntity("Ninja", "ninja.mesh");
//    entNinja->setCastShadows(true);
//    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entNinja);
// 
//    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
// 
//    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
//        plane, 1500, 1500, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
// 
//    Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
//    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);
// 
//    entGround->setMaterialName("Examples/Rockwall");
//    entGround->setCastShadows(false);
//
//    Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
//    pointLight->setType(Ogre::Light::LT_POINT);
//    pointLight->setPosition(Ogre::Vector3(0, 150, 250));
//
//    pointLight->setDiffuseColour(1.0, 0.0, 0.0);
//    pointLight->setSpecularColour(1.0, 0.0, 0.0);
// 
//}

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
    mNode->translate( Ogre::Vector3( 25, 150, 0 ) );
    mNode->roll(Ogre::Degree(80));
//    headNode->scale( .5, 1, 2 );

    Ogre::Entity* ogreHead2 = mSceneMgr->createEntity( "Head2", "Cylinder.mesh" );
    ogreHead2->setCastShadows(true);
    //Ogre::SceneNode* headNode2 = headNode->createChildSceneNode( "HeadNode2", Ogre::Vector3( 100, 0, 0 ) );
    Ogre::SceneNode* headNode2 = mSceneMgr->getRootSceneNode()->createChildSceneNode( "HeadNode2", Ogre::Vector3( 100, 0, 0 ) );
    headNode2->attachObject( ogreHead2 );
//    headNode2->translate( Ogre::Vector3( 10, 0, 10 ) );

    headNode2->yaw(Ogre::Degree(45));
    headNode2->pitch(Ogre::Degree(45));
    headNode2->roll(Ogre::Degree(45));
 
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
        if (mDirection == Ogre::Vector3::ZERO) {
                if (nextLocation()) {
                }//if
        }else{
                Ogre::Real move = mWalkSpeed * evt.timeSinceLastFrame;
                mDistance -= move;
                if (mDistance <= 0.0f){
                        mNode->setPosition(mDestination);
                        mDirection = Ogre::Vector3::ZERO;
                        // Set animation based on if the robot has another point to walk to. 
                        if (!nextLocation()){
                        }else{
                                // Rotation Code will go here later
                                Ogre::Vector3 src = mNode->getOrientation() * Ogre::Vector3::UNIT_X;
                        }//else
                }else{
                        mNode->translate(mDirection * move);
                } // else
        } // if
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
