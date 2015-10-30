##add_subdirectory()
IF(USE_ADOLC)
#subdirs(AmsterdamModel arm_1DOF_muscle KistemakerModel ModelTest StelzerJumpingModel)

subdirlist(directories ${CMAKE_CURRENT_SOURCE_DIR})
list(REMOVE_ITEM directories ".svn")
list(REMOVE_ITEM directories ".git")
list(REMOVE_ITEM directories "CMakeFiles")
# list(REMOVE_ITEM directories "fortranExample") # fortranExample is a standalone project
# list(REMOVE_ITEM directories "OptControlAppDraft") # OptControlAppDraft are standalone fortran exmaples
foreach(item ${directories})
 message("  adding URDFexample: " ${item})
 add_subdirectory(${item})
 if(TARGET ${item})
 project_folder("examples" ${item})
 endif(TARGET ${item})
endforeach(item ${directories})

ENDIF(USE_ADOLC)
