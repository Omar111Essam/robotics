cmake_minimum_required(VERSION 3.0.2)

project(dobot_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS
  scripts/wave_demo.py
  scripts/pick_place_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
