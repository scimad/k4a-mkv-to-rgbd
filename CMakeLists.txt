project(k4a-mkv-to-rgbd)
cmake_minimum_required(VERSION 2.9)
find_package(k4a)
find_package(OpenCV)
find_package(PkgConfig REQUIRED)

pkg_search_module(TURBOJPEG REQUIRED libturbojpeg)
link_directories(${TURBOJPEG_LIBDIR})


add_executable(mkv-to-rgbd
    k4a_rgbd.cpp)
target_link_libraries(mkv-to-rgbd PRIVATE 
    ${OpenCV_LIBS}
    k4a::k4a
    k4arecord
    ${TURBOJPEG_LIBRARIES})