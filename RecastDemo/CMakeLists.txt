# Find dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)

# SDL2 setup - platform specific
if(WIN32)
	if(NOT SDL2_ROOT_DIR)
		set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
	endif()
	find_package(SDL2 REQUIRED)
elseif(APPLE)
	if(NOT SDL2_ROOT_DIR)
		set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Bin")
	endif()
	set(SDL2_FRAMEWORK_PATH "${SDL2_ROOT_DIR}/SDL2.framework")
	# Use the framework's Headers directory for <SDL.h> style includes
	set(SDL2_INCLUDE_DIR "${SDL2_FRAMEWORK_PATH}/Headers")
	# Also add the parent so <SDL2/SDL.h> style includes work (used internally by SDL headers)
	set(SDL2_INCLUDE_DIR_PARENT "${SDL2_ROOT_DIR}")
else()
	# Linux - use system SDL2
	find_package(SDL2 REQUIRED)
endif()

# Build the third-party Contrib library (must come after SDL2 setup since it uses SDL2 variables)
add_subdirectory(Contrib)

# RecastDemo executable
if(WIN32)
	add_executable(RecastDemo WIN32)
elseif(APPLE)
	add_executable(RecastDemo MACOSX_BUNDLE)
else()
	add_executable(RecastDemo)
endif()

target_sources(RecastDemo PRIVATE
	Source/AppState.cpp
	Source/InputGeom.cpp
	Source/main.cpp
	Source/PartitionedMesh.cpp
	Source/PerfTimer.cpp
	Source/Sample.cpp
	Source/Sample_SoloMesh.cpp
	Source/Sample_TempObstacles.cpp
	Source/Sample_TileMesh.cpp
	Source/SampleInterfaces.cpp
	Source/TestCase.cpp
	Source/Tool_ConvexVolume.cpp
	Source/Tool_Crowd.cpp
	Source/Tool_NavMeshPrune.cpp
	Source/Tool_NavMeshTester.cpp
	Source/Tool_OffMeshConnection.cpp
	Source/ValueHistory.cpp
)

target_include_directories(RecastDemo PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/Include
)

set_target_properties(RecastDemo PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)

# Platform-specific compiler options
if(APPLE)
	# Suppress deprecated OpenGL warnings (gluPerspective, gluProject, etc.)
	target_compile_options(RecastDemo PRIVATE -Wno-deprecated-declarations)
endif()

# Link libraries
target_link_libraries(RecastDemo PRIVATE
	OpenGL::GL
	OpenGL::GLU
	Contrib
	RecastNavigation::DebugUtils
	RecastNavigation::Detour
	RecastNavigation::DetourCrowd
	RecastNavigation::DetourTileCache
	RecastNavigation::Recast
)

# SDL2 linking - platform specific
if(APPLE)
	target_link_libraries(RecastDemo PRIVATE "-F${SDL2_ROOT_DIR} -framework SDL2")
	target_include_directories(RecastDemo PRIVATE ${SDL2_INCLUDE_DIR} ${SDL2_INCLUDE_DIR_PARENT})
else()
	target_link_libraries(RecastDemo PRIVATE SDL2::SDL2 SDL2::SDL2main)
endif()

# Copy runtime files to build directory
if(WIN32)
	add_custom_command(TARGET RecastDemo POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SDL2_RUNTIME_LIBRARY}" $<TARGET_FILE_DIR:RecastDemo>/
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes $<TARGET_FILE_DIR:RecastDemo>/Meshes
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases $<TARGET_FILE_DIR:RecastDemo>/TestCases
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf $<TARGET_FILE_DIR:RecastDemo>/
	)
else()
	file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
	file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
	file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

# Installation
install(TARGETS RecastDemo
	RUNTIME DESTINATION bin
	BUNDLE DESTINATION bin
)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

if(WIN32)
	install(FILES "${SDL2_RUNTIME_LIBRARY}" DESTINATION bin)
endif()

# ----------------------------------------------------------------------------
# Optional Python binding target (PyNavmesh)
# Builds a Python extension module (.pyd on Windows, .so on Linux/macOS) that
# exposes the GUI-free Navmesh wrapper. The module has NO dependency on
# SDL2 / OpenGL / imgui / DebugDraw GL backend — only Recast / Detour core
# plus a handful of demo-shared source files (geometry loader, BuildContext).
# ----------------------------------------------------------------------------
option(RECASTNAVIGATION_PYTHON_BINDING "Build PyNavmesh Python extension module" OFF)

if(RECASTNAVIGATION_PYTHON_BINDING)
	# pybind11 (and STL types passed through it) require C++ exceptions to be
	# enabled. The root CMakeLists.txt globally disables them via /EHs-c- on
	# MSVC and -fno-exceptions on GCC/Clang, which we have to undo here.
	# Note: this also re-enables exceptions for RecastDemo when this option is
	# on, which is harmless.
	if(MSVC)
		string(REPLACE "/EHs-c-" "/EHsc" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	else()
		string(REPLACE "-fno-exceptions" "-fexceptions" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	endif()

	find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
	find_package(pybind11 CONFIG REQUIRED)

	# Sources kept to the minimum required by Navmesh.cpp:
	#   * PyNavmesh.cpp / Navmesh.cpp -- the binding itself
	#   * InputGeom.cpp / PartitionedMesh.cpp -- OBJ loader + BVH for raycastMesh
	#   * SampleInterfaces.cpp -- BuildContext / FileIO (DebugDrawGL is stubbed
	#     to no-ops via RECASTNAVIGATION_HEADLESS, see file)
	#   * PerfTimer.cpp -- backing implementation for BuildContext timers
	pybind11_add_module(PyNavmesh MODULE
		Source/PyNavmesh.cpp
		Source/Navmesh.cpp
		Source/InputGeom.cpp
		Source/PartitionedMesh.cpp
		Source/SampleInterfaces.cpp
		Source/PerfTimer.cpp
	)

	target_include_directories(PyNavmesh PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/Include
	)

	target_compile_definitions(PyNavmesh PRIVATE RECASTNAVIGATION_HEADLESS)

	set_target_properties(PyNavmesh PROPERTIES
		CXX_STANDARD 20
		CXX_STANDARD_REQUIRED ON
		CXX_EXTENSIONS OFF
	)

	target_link_libraries(PyNavmesh PRIVATE
		RecastNavigation::DebugUtils
		RecastNavigation::Detour
		RecastNavigation::DetourCrowd
		RecastNavigation::DetourTileCache
		RecastNavigation::Recast
	)

	install(TARGETS PyNavmesh
		LIBRARY DESTINATION ${Python3_SITEARCH}
		RUNTIME DESTINATION ${Python3_SITEARCH}
	)
endif()
