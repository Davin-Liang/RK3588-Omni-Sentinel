本压缩包只包含 SentinelQT 中需要替换的文件，不包含整个工程和二进制文件。

替换方式：
1. 在 APP/SentinelQT 下备份原文件：
   cp widget.cpp widget.cpp.bak
   cp widget.h widget.h.bak
   cp CMakeLists.txt CMakeLists.txt.bak
   cp config.ini config.ini.bak
   cp install/config.ini install/config.ini.bak
   cp install/web/index.html install/web/index.html.bak

2. 将本压缩包中的 SentinelQT/ 对应文件覆盖到 APP/SentinelQT/。

3. 重新编译：
   cd APP/SentinelQT
   rm -rf build install
   ./build.sh

4. 板端测试前确认：
   ls -l /dev/icm45686

本版本只保留 IMU-only EIS：Icm45686Reader -> EisStabilizer -> set_eis_offset_callback -> RGA crop。
已删除 SentinelQT 中视觉 EIS / IMU assist 相关调用和配置入口。
