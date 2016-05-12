cmake_minimum_required (VERSION 2.8.3)
project (team_wpi_integrator_chains)

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

#                               #
#                               #
# TRIAL GENERATION CONFIG FILES #
#                               #
#                               #
install (FILES trialconf/mc-small-out1-order2.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
)
install (FILES trialconf/mc-small-out2-order3.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
)
install (FILES trialconf/mc-small-out2only-order3.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
)
install (FILES trialconf/mc-small-out3-order3.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
)
install (FILES trialconf/mc-small-out3only-order3.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
)
install (FILES trialconf/mc-small-out4only-order3.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
)
install (FILES trialconf/mc-small-out10-order10.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trialconf
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