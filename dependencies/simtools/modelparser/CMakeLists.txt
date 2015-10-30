SET(sim_tools_parser_SRC
  parseTools.cpp
  parseTools.h
  GenericRobotModelReader.cpp
  GenericRobotModelReader.h
)
add_library(sim_tools_parser ${sim_tools_parser_SRC}) 
project_folder("dependencies" sim_tools_parser)