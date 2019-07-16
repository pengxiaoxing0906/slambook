#add_executable(程序名 源代码文件)
add_executable( run_vo  run_vo.cpp)
#add_executable()将run_vo.cpp源文件编译成run_vo的可执行文件
target_link_libraries( run_vo myslam )
#target_link_libraries指令的作用为将目标文件（run_vo可执行文件)与库文件进行链接