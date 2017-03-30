include_directories( ${CONAN_INCLUDE_DIRS}
                      ${GTEST_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/lib )

file( GLOB UNIT_TEST_SRCS test_*.cpp )

add_executable( lsdslam_test
                ${UNIT_TEST_SRCS} )

link_directories( ${CONAN_LINK_DIRECTORIES} )
target_link_libraries( lsdslam_test
          ${CONAN_LIBS}
          ${GTEST_BOTH_LIBRARIES}
          lsdslam )


# # Don't use ctest, instead just execture the Google test binaries
add_custom_target( unit_test
                		WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                		COMMAND lsdslam_test
                    DEPENDS lsdslam_test )
