cmake_minimum_required(VERSION 3.7)

set(CALIB_PROJ_NAME mono_calibrator)




######################################
# The external reference directory (searching) for cmake
set(EXT_DIR ${CMAKE_SOURCE_DIR}/cmake )
list( APPEND CMAKE_MODULE_PATH
  ${EXT_DIR}
)

# find opencv
find_package(OpenCV REQUIRED)


list( APPEND
	EXT_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS}
)
list( APPEND
	EXT_LIBS
	${OpenCV_LIBS}
)



set(CALIB_PROJ_SOURCE
        ${CMAKE_SOURCE_DIR}/main.cpp
        
)





#
# declaring external library include directories
include_directories(${EXT_INCLUDE_DIRS})

# The gcalibrator executable     
add_executable(${CALIB_PROJ_NAME}
	       ${CALIB_PROJ_SOURCE}
	          )
        

              
target_link_libraries(${CALIB_PROJ_NAME}
		      ${EXT_LIBS}
		      )

add_executable(${CALIB_PROJ_NAME}_images
	          main_image_based.cpp
	          )
target_link_libraries(${CALIB_PROJ_NAME}_images
		      ${EXT_LIBS}
		      )
