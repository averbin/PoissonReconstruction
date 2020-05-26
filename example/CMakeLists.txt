add_executable(PoissonReconstruction Example.cxx )
target_link_libraries( PoissonReconstruction vtkPoissonReconstruction )

add_test( PoissonReconstructionTest
  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/PoissonReconstruction
  ${PoissonReconstruction_SOURCE_DIR}/data/horsePoints.vtp
  5
  temp.vtp
)

message(STATUS "vtk libs : ${VTK_LIBRARIES} : ${VTK_BINARY_DIR}")
add_custom_command(TARGET PoissonReconstruction POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "D:/programming/Lib/VTK-9.0.0/build/bin/Debug/"
        $<TARGET_FILE_DIR:PoissonReconstruction>)