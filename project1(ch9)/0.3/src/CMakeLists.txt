add_library(myslam SHARED
        camera.cpp
        config.cpp
        frame.cpp
        map.cpp
        mappoint.cpp
        visual_odometry.cpp g2o_types.cpp ../include/myslam/g2o_types.h)
#myslam为要生成的库的名字 后面为库里的各个类的源文件列表
#add_library指令的主要作用就是将指定的源文件生成链接文件，然后添加到工程中去


target_link_libraries( myslam
        ${THIRD_PARTY_LIBS}
        )
#不理解这里的写法
#target_link_libraries指令的作用为将目标文件与库文件进行链接
