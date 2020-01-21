# Install script for directory: C:/Code/MAT 240B/allolib_playground/al_ext

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/allolib_playground")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Code/MAT 240B/allolib_playground/Darthnerda/assignment/build/reverse-words/Release/al_ext/assets3d/cmake_install.cmake")
  include("C:/Code/MAT 240B/allolib_playground/Darthnerda/assignment/build/reverse-words/Release/al_ext/openvr/cmake_install.cmake")
  include("C:/Code/MAT 240B/allolib_playground/Darthnerda/assignment/build/reverse-words/Release/al_ext/soundfile/cmake_install.cmake")
  include("C:/Code/MAT 240B/allolib_playground/Darthnerda/assignment/build/reverse-words/Release/al_ext/spatialaudio/cmake_install.cmake")
  include("C:/Code/MAT 240B/allolib_playground/Darthnerda/assignment/build/reverse-words/Release/al_ext/statedistribution/cmake_install.cmake")

endif()

