cmake_minimum_required (VERSION 2.7) 
project (ogre_tutorial) 

SET(ENV{PKG_CONFIG_PATH} "usr/local/lib/pkgconfig")
SET(BULLET_PHYSICS_SOURCE_DIR "/home/anjack/opengl/bullet-2.82-r2704")

message("PKG_CONFIG_PATH: "$ENV{PKG_CONFIG_PATH})

SET(OPENGL_gl_LIBRARY "/usr/lib/i386-linux-gnu/libGL.so")
SET(OPENGL_glu_LIBRARY "/usr/lib/i386-linux-gnu/libGLU.so")
SET(GLUT_glut_LIBRARY "/usr/lib/i386-linux-gnu/libglut.so.3")

SET(SRC
    "TutorialApplication.cpp"
    "BaseApplication.cpp"
    "DebugDraw.cpp"
    "DynamicLineDrawer.cpp"
    "PtlBtOgreShapeDispatcher.cpp"
    "PtlBulletImporterShapeDispatcher.cpp"
    "PtlCollisionShapeDispatcher.cpp"
    "PtlPhysicalBody.cpp"
    "PtlOgrePhysicalBody.cpp"
    "PtlWheelComponent.cpp"
    "PtlChainComponent.cpp"
    "PtlRackComponent.cpp"
    "PtlFlywheelChainWeights.cpp"
    "PtlRingwheelComponent.cpp"
    "PtlRingwheelGearComponent.cpp"
    "PtlParallelRingwheels.cpp"
    "PtlLeverComponent.cpp"
    "PtlSlotBlockerComponent.cpp"
)

INCLUDE(FindPkgConfig)

pkg_check_modules(ogre_deps REQUIRED OGRE OIS OGRE-Terrain bullet CEGUI-0 CEGUI-0-OGRE)

FOREACH(CFLAG ${ogre_deps_CFLAGS})
    SET(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} ${CFLAG}")
ENDFOREACH(CFLAG)

SET(CMAKE_C_FLAGS "${EXTRA_C_FLAGS} -g -Wall")
SET(CMAKE_CXX_FLAGS "${EXTRA_C_FLAGS} -g -Wall")

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${BULLET_PHYSICS_SOURCE_DIR}/src 
    ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL
)

add_executable(ogrePhysics ${SRC})
target_link_libraries(ogrePhysics ${ogre_deps_LDFLAGS}
		      ${PROJECT_SOURCE_DIR}/libBtOgre.a
		      ${PROJECT_SOURCE_DIR}/libBulletWorldImporter.a
		      ${PROJECT_SOURCE_DIR}/libOpenGLSupport.a
		      ${PROJECT_SOURCE_DIR}/libBulletFileLoader.a
		      ${GLUT_glut_LIBRARY}
		      ${OPENGL_gl_LIBRARY}
		      ${OPENGL_glu_LIBRARY})
