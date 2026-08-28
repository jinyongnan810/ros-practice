cmake_minimum_required(VERSION 3.8)
project(simple_car_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install description, mesh, launch, rviz, config, and world files
install(DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
