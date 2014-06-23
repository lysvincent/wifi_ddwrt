# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(wifi_ddwrt)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  Network.msg
  SiteSurvey.msg
  AccessPoint.msg
  CellSig.msg
  CellAp.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)

install(PROGRAMS 
   nodes/ddwrt.py
   scripts/analysis.py
   scripts/ddwrt_sim.py
   scripts/monitor
   scripts/ddwrt_config.py
   scripts/wanip.py
   scripts/fingerprinting
   scripts/approximate
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY   
   launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY   
   config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
