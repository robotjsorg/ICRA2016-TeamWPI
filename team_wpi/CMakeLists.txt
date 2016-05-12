cmake_minimum_required (VERSION 2.8.3)
project (team_wpi)

find_package (catkin REQUIRED COMPONENTS rospy dynamaestro)

catkin_package ()

#              #
#              # 
# LAUNCH FILES #
#              #
#              #
install (FILES launch/wpi_solution_01.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install (FILES launch/wpi_solution_02.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#         #
#         #
# SCRIPTS #
#         #
#         #
install (PROGRAMS scripts/wpi_solution_01.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install (PROGRAMS scripts/wpi_solution_02.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#           #
#           #
# TOOLBOXES #
#           #
#           #
install (PROGRAMS scripts/toolboxes/other_toolbox.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install (PROGRAMS scripts/toolboxes/planner_toolbox.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install (PROGRAMS scripts/toolboxes/tsp_toolbox.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install (PROGRAMS scripts/toolboxes/prm_toolbox.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)