# CMakeList.txt : CMake project for scale_interface, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("scale_interface")


# --- BOOST CONFIG ---
# Tell CMake where Boost is
set(BOOST_ROOT "C:/local/boost_1_83_0")
set(Boost_USE_STATIC_LIBS ON)  # use static .lib files (optional, but common on Windows)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)  # Set ON only if you want static CRT linkage

# Find Boost libraries you need
find_package(Boost REQUIRED COMPONENTS system)





# Add source to this project's executable.
add_executable (scale_interface "main.cpp" "scale_interface.h" "scale_interface.cpp")


# Tell CMake where to find headers
include_directories(
    "C:/local/DynamixelSDK/DynamixelSDK-main/c++/include/dynamixel_sdk"
)

# Define the debug and release library paths
set(DYNAMIXELSDK_DEBUG_LIB
    "C:/local/DynamixelSDK/DynamixelSDK-main/c++/build/win64/x64/Debug/dxl_x64_cpp.lib"
)
set(DYNAMIXELSDK_RELEASE_LIB
    "C:/local/DynamixelSDK/DynamixelSDK-main/c++/build/win64/output/dxl_x64_cpp.lib"
)


if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET scale_interface PROPERTY CXX_STANDARD 20)
endif()


# --- Include and link Boost ---
target_include_directories(scale_interface PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(scale_interface PRIVATE ${Boost_LIBRARIES} debug ${DYNAMIXELSDK_DEBUG_LIB} optimized ${DYNAMIXELSDK_RELEASE_LIB})
# TODO: Add tests and install targets if needed.
