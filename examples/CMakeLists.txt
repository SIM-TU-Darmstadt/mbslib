subdirlist(directories ${CMAKE_CURRENT_SOURCE_DIR})
list(REMOVE_ITEM directories ".svn")
list(REMOVE_ITEM directories ".git")
list(REMOVE_ITEM directories "CMakeFiles")
list(REMOVE_ITEM directories "fortranExample") # fortranExample is a standalone project 
list(REMOVE_ITEM directories "OptControlAppDraft") # OptControlAppDraft are standalone fortran exmaples

if(NOT USE_DIRCOL)
  list(REMOVE_ITEM directories "1DOF_Pendel_simple") # add 1DOF_Pendel_simple just if USE_DIRCOL
  list(REMOVE_ITEM directories "2DOF_Pendel_simple") # add 2DOF_Pendel_simple just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslib") # add dircolMbslib just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibMuscle") # add dircolMbslibMuscle just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i1") # add dircolMbslibRobot_i1 just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i1_grav") # add dircolMbslibRobot_i1_grav just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i1_grav_time") # add dircolMbslibRobot_i1_grav_time just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i1_lin") # add dircolMbslibRobot_i1_lin just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i1_time") # add dircolMbslibRobot_i1_time just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i2") # add dircolMbslibRobot_i2 just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolMbslibRobot_i2_time") # add dircolMbslibRobot_i2_time just if USE_DIRCOL
  list(REMOVE_ITEM directories "dircolSimpleSample") # add dircolSimpleSample just if USE_DIRCOL
  list(REMOVE_ITEM directories "DIRCOLexamples") # add DIRCOLexamples just if USE_DIRCOL
endif(NOT USE_DIRCOL)

list(SORT directories)
foreach(item ${directories})
  message("  adding example: " ${item})
  add_subdirectory(${item})
  if(TARGET ${item})
  project_folder("examples" ${item})
  endif(TARGET ${item})
endforeach(item ${directories})
