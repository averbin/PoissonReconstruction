cmake_minimum_required(VERSION 3.0)

project(PoissonReconstruction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

if( ${VTK_BUILD_SHARED_LIBS} MATCHES ON )
  set( SHARED_LIBS "SHARED" )
  add_definitions( "-DvtkPoissonReconstruction_EXPORTS" )
  add_definitions( "-DBUILD_SHARED_LIBS" )
else(${VTK_BUILD_SHARED_LIBS} MATCHES ON )
  set( SHARED_LIBS "STATIC" )
endif(${VTK_BUILD_SHARED_LIBS} MATCHES ON )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY
  ${PoissonReconstruction_BINARY_DIR}/bin
  CACHE PATH "path for all executables"
)

set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY
  ${PoissonReconstruction_BINARY_DIR}/lib
  CACHE PATH "path for all libraries"
)

SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY
  ${PoissonReconstruction_BINARY_DIR}/lib
  CACHE PATH "path for all libraries"
)

include_directories(
  ${PoissonReconstruction_SOURCE_DIR}/source
  ${PoissonReconstruction_BINARY_DIR}
)

add_subdirectory(source)

include( CTest )
add_subdirectory(example)

set(BUILD_PARAVIEW_PLUGIN OFF CACHE BOOL "Build Paraview plugin?")

if(BUILD_PARAVIEW_PLUGIN)
  add_subdirectory(plugin)
endif(BUILD_PARAVIEW_PLUGIN)

option( BUILD_DOCUMENTATION "Build Doxygen Documentation" OFF )

if( BUILD_DOCUMENTATION )
  add_subdirectory( documentation )
endif( BUILD_DOCUMENTATION )

target_compile_features(vtkPoissonReconstruction PUBLIC cxx_std_14)
set_target_properties(vtkPoissonReconstruction PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14 /Zc:__cplusplus")
endif()

if (MSVC)
    target_compile_options(vtkPoissonReconstruction INTERFACE "/Zc:__cplusplus")
endif()
message("Supported features = ${CMAKE_CXX_COMPILE_FEATURES}")