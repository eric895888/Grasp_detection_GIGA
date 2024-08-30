# Demo 步驟


# 查看tsdf可視化（前置）:
開一個終端機輸入
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30 filters:=pointcloud
```
會設定解析度及深度解析度為640x480並且顯示點雲
再開另外一個終端機輸入
```
rosrun rviz rviz -d /home/robotic/Grasp_detection_GIGA/Grasp.rviz
```
可以看到畫面-d 後面是config的路徑

# 執行（選一種）：

1.開啟vscode 打開mian.py並執行,預設Block,可以修改880行更換
parser.add_argument("--model", type=str, default="Block")

2.或是終端機中打下方指令
```
python scripts/main.py --model Block
```

```
共有下面四種可以替換類別
Block
TT_cube
Bolt
Sundries
```

# GIGA模擬夾取
```
積木
python sim_grasp_multiple.py --num-view 1 --object-set blocks2 --scene pile --num-rounds 100 --sideview --add-noise dex --force --best --model data/models/Block_giga.pt --type giga --result-path /path/to/result --sim-gui
```
```
金屬方塊
python sim_grasp_multiple.py --num-view 1 --object-set TT_cube --scene pile --num-rounds 100 --sideview --add-noise dex --force --best --model data/models/TT_cube_giga.pt --type giga --result-path /path/to/result --sim-gui
```