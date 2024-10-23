cmake_minimum_required(VERSION 3.8)
project(rws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0135 NEW)

### Fetch JSON at configure time
include(FetchContent)
fetchcontent_declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
fetchcontent_makeavailable(json)
# /JSON

### Fetch ASIO at configure time
add_definitions(
  # Complile ASIO without Boost
  -DBOOST_DATE_TIME_NO_LIB
  -DBOOST_REGEX_NO_LIB
  -DASIO_STANDALONE
)
fetchcontent_declare(asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
  GIT_TAG asio-1-24-0)

fetchcontent_getproperties(asio)
if(NOT asio_POPULATED)
  fetchcontent_populate(asio)
endif()

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
# /ASIO

### Fetch ASIO at configure time
add_definitions(
  # Compile websocketpp with C++11
  -D_WEBSOCKETPP_CPP11_STL_
  -D_WEBSOCKETPP_CPP11_FUNCTIONAL_
)
fetchcontent_declare(websocketpp
GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
  GIT_TAG 0.8.2)
fetchcontent_getproperties(websocketpp)
if(NOT websocketpp_POPULATED)
  fetchcontent_populate(websocketpp)
  add_subdirectory(${websocketpp_SOURCE_DIR} ${websocketpp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

add_library(websocketpp INTERFACE)
target_include_directories(websocketpp INTERFACE ${websocketpp_SOURCE_DIR})
# /WEBSOCKETPP

set(DEPENDENCIES
  rclcpp
  ament_index_cpp
  CycloneDDS
)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

file(GLOB SRCFILES src/*.*pp)
add_executable(rws_server ${SRCFILES})
ament_target_dependencies(rws_server ${DEPENDENCIES})
target_include_directories(rws_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rws_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(rws_server CycloneDDS::ddsc asio websocketpp nlohmann_json::nlohmann_json)

install(TARGETS rws_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcl_interfaces REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # rws_translate_test
  ament_add_gtest(rws_translate_test
    src/typesupport_helpers.cpp
    src/translate.cpp
    src/serdes.cpp
    test/translate_test.cpp
  )
  ament_target_dependencies(rws_translate_test ${DEPENDENCIES})
  target_include_directories(rws_translate_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(rws_translate_test CycloneDDS::ddsc nlohmann_json::nlohmann_json)

  # rws_typesupport_helpers_test
  ament_add_gtest(rws_typesupport_helpers_test
    src/typesupport_helpers.cpp
    test/typesupport_helpers_test.cpp
  )
  ament_target_dependencies(rws_typesupport_helpers_test ${DEPENDENCIES})
  target_include_directories(rws_typesupport_helpers_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>)

  # rws_connector_test
  ament_add_gmock(rws_connector_test
    test/connector_test.cpp
    src/typesupport_helpers.cpp
  )
  ament_target_dependencies(rws_connector_test ${DEPENDENCIES})
  target_include_directories(rws_connector_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/mocks>
    $<INSTALL_INTERFACE:include>)

  # client_handler_test
  ament_add_gmock(client_handler_test
    test/client_handler_test.cpp
    src/client_handler.cpp
    src/typesupport_helpers.cpp
    src/translate.cpp
    src/serdes.cpp
  )
  ament_target_dependencies(client_handler_test ${DEPENDENCIES})
  target_include_directories(client_handler_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/mocks>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(client_handler_test CycloneDDS::ddsc nlohmann_json::nlohmann_json)
endif()

ament_package()
