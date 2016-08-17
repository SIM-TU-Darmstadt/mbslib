subdirlist(directories ${CMAKE_CURRENT_SOURCE_DIR})
list(REMOVE_ITEM directories ".svn")
list(REMOVE_ITEM directories ".git")

# setup boost
SET(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.40.0 COMPONENTS date_time filesystem system signals thread unit_test_framework)

if(${Boost_FOUND})
  link_directories(${Boost_LIBRARY_DIRS})
  include_directories(${Boost_INCLUDE_DIR})
endif()

foreach(item ${directories})
 message("  adding test: " ${item})
 add_subdirectory(${item})  
 if(TARGET ${item})
 project_folder("tests" ${item})  
 add_test(NAME ${item} COMMAND ${item})
 endif(TARGET ${item})
endforeach(item ${directories})

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
 
