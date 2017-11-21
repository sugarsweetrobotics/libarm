set(_srcs arm.cpp)

set(test_srcs test.cpp ${_srcs})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)

find_package(Threads)

add_library(${PROJECT_NAME} SHARED ${_srcs})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

add_executable(${PROJECT_NAME}_test ${test_srcs})
target_link_libraries(${PROJECT_NAME}_test ${CMAKE_THREAD_LIBS_INIT})

if(WIN32)
 set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:\"LIBCMT.LIB\"")
endif()

add_library(${PROJECT_NAME}Static STATIC ${_srcs})
target_link_libraries(${PROJECT_NAME}Static ${CMAKE_THREAD_LIBS_INIT})

set_property(TARGET ${PROJECT_NAME}Static APPEND PROPERTY COMPILE_DEFINITIONS EXPORT_${PROJECT_NAME}_STATIC)
