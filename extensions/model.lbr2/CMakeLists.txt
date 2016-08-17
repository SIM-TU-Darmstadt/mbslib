file(GLOB_RECURSE model *.cpp *.hpp *.h *.c)
add_library(model.lbr2 ${model})
target_link_libraries(model.lbr2 mbslib)
