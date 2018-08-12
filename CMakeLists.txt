cmake_minimum_required( VERSION 2.8 )

project( FastPhOdo )

find_package( OpenCV 3.0 REQUIRED )
find_package( Boost COMPONENTS program_options filesystem REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIR} )

add_executable( phodo "src/phodo.cpp" )
target_link_libraries( phodo ${OpenCV_LIBS} ${Boost_LIBRARIES} )
