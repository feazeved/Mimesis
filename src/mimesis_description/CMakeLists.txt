cmake_minimum_required(VERSION 3.22)
project(mimesis_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # The description must expand and parse, with fake and with real hardware.
  set(CHECK ${CMAKE_CURRENT_SOURCE_DIR}/test/check_urdf.sh)
  set(TOP ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mimesis.urdf.xacro)
  add_test(NAME urdf_mock_hardware COMMAND bash ${CHECK} ${TOP} use_mock_hardware:=true)
  add_test(NAME urdf_real_hardware COMMAND bash ${CHECK} ${TOP} use_mock_hardware:=false)
  add_test(NAME urdf_with_prefix COMMAND bash ${CHECK} ${TOP} prefix:=left_)
endif()

ament_package()
