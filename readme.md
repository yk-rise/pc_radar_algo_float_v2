
# 编译
cmake -S pc_radar_algo_float_v2 -B pc_radar_algo_float_v2/build
cmake --build pc_radar_algo_float_v2/build --config Release

# 运行
pc_radar_algo_float_v2\build\Release\pc_radar_algo_float_v2.exe 2dfft AT_sy_r10_1.txt
<!-- 固定帧 -->
pc_radar_algo_float_v2\build\Release\pc_radar_algo_float_v2.exe 2dfft_array AT_sy_r10_1.txt
<!-- 生成单帧数据结构体 -->
python txt_to_sradarframe.py AT_sy_r10_1.txt --select-frame 0 -o frame_0000.c
<!-- 串口接收数据并保存 -->
python tools\golden_serial_capture.py --port COM5 --baudrate 115200 --output golden_batch.json


# 串口转发程序
<!-- 下载完com0com之后，改一下虚拟端口号 -->
PS E:\destop\project\AT24_20260311_Checkdefault> & "C:\Program Files (x86)\com0com\setupc.exe" change CNCA0 PortName=COM50
PS E:\destop\project\AT24_20260311_Checkdefault> & "C:\Program Files (x86)\com0com\setupc.exe" change CNCB0 PortName=COM51

<!-- 在端口10，波特率115200上发送 -->
python pc_radar_algo_float_v2\serial_stdout_forwarder.py --port COM10 --baudrate 115200 --log forward.log -- pc_radar_algo_float_v2\build\Release\pc_radar_algo_float_v2.exe 2dfft_array AT_sy_r10_1.txt

python pc_radar_algo_float_v2\serial_stdout_forwarder.py --port COM50 --baudrate 3250000 --log forward.log -- pc_radar_algo_float_v2\build\Release\pc_radar_algo_float_v2.exe 2dfft_array AT_sy_r10_1.txt


# txt_to_sradarframe.py

运行：
python txt_to_sradarframe.py AT_sy_r10_1.txt
增加输出名：
python txt_to_sradarframe.py AT_sy_r10_1.txt -o my_frames.c
更换数组名：
python txt_to_sradarframe.py AT_sy_r10_1.txt --prefix myRadarFrame
