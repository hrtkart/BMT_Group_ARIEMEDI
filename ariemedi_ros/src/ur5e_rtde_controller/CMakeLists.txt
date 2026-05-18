cmake_minimum_required(VERSION 3.8)
project(ur5e_rtde_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/ur5e_rtde_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ur5e_rtde_node
)

install(PROGRAMS
  scripts/eef_marker_relative_plot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME eef_marker_relative_plot
)

install(PROGRAMS
  scripts/dual_ur5e_rtde_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dual_ur5e_rtde_node
)

install(PROGRAMS
  scripts/dual_eef_marker_relative_plot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dual_eef_marker_relative_plot
)

install(PROGRAMS
  scripts/dual_eef_marker_data_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dual_eef_marker_data_recorder
)

install(PROGRAMS
  scripts/dual_eef_marker_time_aligned_plot_from_numpy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dual_eef_marker_time_aligned_plot_from_numpy
)

install(PROGRAMS
  scripts/test_dual_base_transform.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_dual_base_transform
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
