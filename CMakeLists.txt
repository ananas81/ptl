cmake_minimum_required (VERSION 2.7) 
project (ogre_tutorial) 

SET(ENV{PKG_CONFIG_PATH} "usr/local/lib/pkgconfig")

message("PKG_CONFIG_PATH: "$ENV{PKG_CONFIG_PATH})

SET(SRC
    "TutorialApplication.cpp"
    "BaseApplication.cpp"
)

INCLUDE(FindPkgConfig)

pkg_check_modules(ogre_deps REQUIRED OGRE OIS OGRE-Terrain bullet CEGUI)

FOREACH(CFLAG ${ogre_deps_CFLAGS})
    SET(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} ${CFLAG}")
ENDFOREACH(CFLAG)

SET(CMAKE_C_FLAGS "${EXTRA_C_FLAGS} -g -Wall")
SET(CMAKE_CXX_FLAGS "${EXTRA_C_FLAGS} -g -Wall")

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
)

add_executable(ogrePhysics ${SRC})
target_link_libraries(ogrePhysics ${ogre_deps_LDFLAGS} ${PROJECT_SOURCE_DIR}/libBtOgre.a)
