cmake_minimum_required(VERSION 3.5)
project(smb_mission_planner)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs message_generation)


add_service_files(
  DIRECTORY srv
  FILES BaseGoal.srv DetectObject.srv RecordBasePose.srv RecordMission.srv RemoveMission.srv RemoveWaypoint.srv ToggleFileDump.srv)


catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs )

catkin_package(
  CATKIN_DEPENDS message_runtime
)


