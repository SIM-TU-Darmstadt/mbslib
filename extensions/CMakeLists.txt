# get all subdirectories but remove .svn if it exists
SET(MBSLIB_INCLUDE_DIR
    ${MBSLIB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
        CACHE INTERNAL "mbslib include directories" FORCE)

subdirlist(directories ${CMAKE_CURRENT_SOURCE_DIR})
list(REMOVE_ITEM directories ".svn")
list(REMOVE_ITEM directories ".git")
list(REMOVE_ITEM directories "include")

# include current source dir
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# sort directories list
list(SORT directories)

# add every subdirectory found
foreach(item ${directories})
  STRING(REPLACE "." "_" ITEM_OPTION_NAME ${item})
  option(USE_EXTENSION_${ITEM_OPTION_NAME} "Use extension ${item}" ON)
  IF(USE_EXTENSION_${ITEM_OPTION_NAME})
    message("  adding extension: " ${item})
    add_subdirectory(${item})
  ENDIF()
endforeach(item ${directories})
