cmake_minimum_required(VERSION 2.8.3)
project(pcu_sl_interface)
find_package (Threads REQUIRED) # Notwendig, damit die executable kompiliert

# Code für Library ros_sl_interface.so
add_library(ros_sl_interface SHARED src/node.cpp)
target_link_libraries(ros_sl_interface  ${CMAKE_THREAD_LIBS_INIT})

# Code für Minimalbeispiel test_interface
add_executable(test_interface src/test_interface.cpp)
target_link_libraries(test_interface ros_sl_interface)

# Zum Kopieren der Kompilate in die entsprechenden Ordner bei `make install`
# Standardmäßig Systemordner (/usr/lib).
install(TARGETS ros_sl_interface EXPORT pcu_sl_interface DESTINATION lib)
install(TARGETS test_interface EXPORT pcu_sl_interface DESTINATION bin)
