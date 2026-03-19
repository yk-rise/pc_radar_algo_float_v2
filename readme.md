
# 编译
cmake -S pc_radar_algo_float_v2 -B pc_radar_algo_float_v2/build
cmake --build pc_radar_algo_float_v2/build --config Release

# 运行
pc_radar_algo_float_v2\build\Release\pc_radar_algo_float_v2.exe 2dfft AT_sy_r10_1.txt


# txt_to_sradarframe.py

运行：
python txt_to_sradarframe.py AT_sy_r10_1.txt
增加输出名：
python txt_to_sradarframe.py AT_sy_r10_1.txt -o my_frames.c
更换数组名：
python txt_to_sradarframe.py AT_sy_r10_1.txt --prefix myRadarFrame
