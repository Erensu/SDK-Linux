运行方式：
1.安装Boost,gtest,g3log，opencv3.4，protobuf3.4
2.将libindem.a libDBoW2.so libDLib.so libhandle_fuse.so libslam_imp.so放到/use/local/lib 下或者通过环境变量制定链接路径
3.修改Indem.cpp，获得图像，IMU数据