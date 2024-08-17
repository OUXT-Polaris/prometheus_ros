cmake_minimum_required(VERSION 3.8)
project(prometheus_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_vendor(prometheus-cpp
  VCS_URL https://github.com/jupp0r/prometheus-cpp.git
  VCS_VERSION v1.2.4
  CMAKE_ARGS -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
