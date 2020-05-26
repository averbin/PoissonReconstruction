set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

configure_file(
  ${PoissonReconstruction_SOURCE_DIR}/source/vtkPoissonReconstructionConfigure.h.in
  ${PoissonReconstruction_BINARY_DIR}/vtkPoissonReconstructionConfigure.h
  @ONLY IMMEDIATE
)

set(INCLSOURCE
	Allocator.h
	BinaryNode.h
	Factor.h
	FunctionData.h
	Geometry.h
	Hash.h
	MarchingCubes.h
	MultiGridOctest.h
	MultiGridOctreeData.h
	Octree.h
	Polynomial.h
	PPolynomial.h
	SparseMatrix.h
	Vector.h
	vtkPoissonReconstruction.h
)

add_library( vtkPoissonReconstruction
  ${SHARED_LIBS}
  ${INCLSOURCE}
  vtkPoissonReconstruction.cxx
  Factor.cpp
  Geometry.cpp
  MarchingCubes.cpp
  MultiGridOctest.cpp
  MultiGridOctreeData.cpp
)

target_link_libraries( vtkPoissonReconstruction
  ${VTK_LIBRARIES}
)

target_compile_features(vtkPoissonReconstruction PUBLIC cxx_std_17)
set_target_properties(vtkPoissonReconstruction PROPERTIES
    CXX_STANDARD 17
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
