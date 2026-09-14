cmake_minimum_required(VERSION 3.8)
project(tfm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias de compilacion e instalacion del paquete.
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Linters de copyright/estilo Python desactivados: colcon test valida
  # CMake/XML sin bloquear por formato.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Instala launch, configuracion, mapas y URDF en share/${PROJECT_NAME}.
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# install(PROGRAMS
#   scripts/mission_manager.py
#   scripts/odom_tf_broadcaster.py
#   scripts/optitrack_republisher.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
