# Grasp_detection_GIGA
> numpy注意要能使用np.int只能使用1.20以下或是1.23.5 (已在requirement.txt指定版本)  
目前使用1.22.4	
而 scikit-image==0.19.0 (已在requirement.txt指定版本)

# 安裝ros: 
```
sudo sh bash ros1_noetic_install.sh 就會自動安裝
```

# 安裝Anaconda
推薦安裝在/home/你的使用者名稱/anaconda3 裡面
conda未啟動解決方式：https://developer.huawei.com/consumer/cn/blog/topic/03940616429410292

或手動啟動source /home/eric/anaconda3/bin/activate
```
sudo sh Anaconda3-2024.02-1-Linux-x86_64.sh 
```

# (推薦)透過.yaml產生已安裝好的環境
```
conda env create --file environment.yaml --name 自行定義名字
```

(不推薦)或是使用作者提供requirements下列指令,但因為open3d版本問題跟skylearn套件改名問題要自行處理
```
pip install -r requirements.txt
```

# 套件安裝
```
pip install opencv-contrib-python==4.6.0.66
pip install numpy==1.22.4
pip install pytorch-ignite==0.5.0.post2
pip install pykdtree
pip install rospkg
pip install networkx==2.2
```
#目前open3d要是0.12.0!!!!!!!!!不然可能有未知錯誤像是產生core dump 或是 segmentation fault 這東西非常容易版本問題跟其他套件產成錯誤根據gdb追蹤可能是底層cuda調用c++那邊容易出問題
只靠requirements.txt也許torch少了些其他東西下方為安裝指定torch版本(當下是2.2.2)
參考來源:https://pytorch.org/get-started/locally/

# torch安裝
其他版本torch參考來源:https://pytorch.org/get-started/previous-versions/ 不要用stable

其他版本torch範例 pip install torch==1.7.0 torchvision==0.8.0 torchaudio==0.7.0
```
pip install torch==2.2.2 torchvision==0.17.2 torchaudio==2.2.2 --index-url https://download.pytorch.org/whl/cu118
```

# 測試：
在python中查看自己對應的版本理論上torch應該是2.2.2 cuda應該是11.8也就是cu118
終端機裡打上python啟動環境或是vscode中新增一份.py檔案

如果沒顯示出任何東西表示安裝不成功!!!!
```
import torch
print(torch.__version__)
顯示 2.2.2+cu118
print(torch.version.cuda)
顯示  11.8
print(torch.backends.cudnn.version())
顯示  8700
```

# 安裝torch-scatter額外函式庫在下方對應的版本如下：
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.2.2+cu118.html

# 安裝vgn
打開終端機輸入: 
```
cd Grasp_detection_GIGA
pip install -e . //安裝vgn
```
# 編譯C++原始碼
編譯src裡的vgn資料夾c++給python,編譯完後會自動把build/lib.linux-x86_64-3.8/src/內的資料夾複製進scripts中的資料夾中並覆蓋

因為這步驟在windows底下會出錯只能在linux中使用推測因為windows會調用Microsoft Visual Studio裡的編譯器來做會導致出錯
打開終端機輸入: 
```
python scripts/convonet_setup.py build_ext --inplace 
```

# 產生資料集
進入scripts資料夾
在終端機打上指令產生資料 範例是產生積木資料


```
cd scripts
python generate_data_parallel.py --scene pile --object-set blocks2 --num-grasps 4000000 --num-proc 1 --save-scene ./data/pile/blocks
```
參數說明:

範例是產生blocks積木資料綠色部分的字可以換成M24_60mm_bolt(螺絲)，TT_cube(金屬方塊)，sundries(生活雜物)

--scene 設定pile表示散堆擺放模式

--object-set 後面表示urdf物件模型要載入哪一個資料夾 

--num-grasps 代表夾取次數 

--num-proc 代表幾個process在跑，如果40會跑不動就設定小一點或是1

--save-scene 代表產生的data儲存的位置

--single-view 表示單一視角

--add-noise 針對dataset添加雜訊


# 平衡正負樣本的數量
python scripts/clean_balance_data.py ./data/pile/blocks

# 產生Occupancy probility
產生occupancy probility的資料
```
python save_occ_data_parallel.py ./data/pile/blocks 100000 2 --num-proc 40
```
# 轉成訓練用格式資料集
```
python construct_dataset_parallel.py --num-proc 1 --single-view --add-noise dex data/pile/blocks data/dataset/blocks
```

# 訓練模型
```
python scripts/train_giga.py --dataset dataset/pile/blocks --dataset_raw data/pile/blocks
```
參數說明:

範例是產生積木資料

--dataset 路徑是使用construct_dataset_parallel.py 產生的訓練用格式的資料集

--dataset_raw 路徑是使用generate_data_parallel.py 產生的原始夾取資料

# GIGA模擬夾取(範例)
```
# 積木
python sim_grasp_multiple.py --num-view 1 --object-set blocks2 --scene pile --num-rounds 100 --sideview --add-noise dex --force --best --model data/models/Block_giga.pt --type giga --result-path /path/to/result --sim-gui

# 金屬方塊
python sim_grasp_multiple.py --num-view 1 --object-set TT_cube --scene pile --num-rounds 100 --sideview --add-noise dex --force --best --model data/models/TT_cube_giga.pt --type giga --result-path /path/to/result --sim-gui
```

參數說明:

範例是產生積木資料

--num-view 視角數量

--object-set 後面表示urdf物件模型要載入哪一個資料夾 

--scene 設定pile表示散堆擺放模式

--num-rounds 夾取次數

--sideview 表示單一視角

--add-noise 添加雜訊的方式

--force 當全部候選點都低於threshold，強迫選出一個抓取位置

--best 例如有好幾個候選抓取位置，選擇最好的候選夾取位置

--model 模型的路徑位置

--type 使用的模型格式

--result-path 實驗的結果存放位置



# 實際環境夾取:

連接相機及啟動Rviz:

開一個終端機輸入下方指令，會設定解析度及深度解析度為640x480並且透過ROS取像
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30 filters:=pointcloud
```

再開另外一個終端機輸入，啟動Rviz,可以看到夾取位置預測畫面-d 後面是rviz config的路徑
```
rosrun rviz rviz -d /home/robotic/Grasp_detection_GIGA/Grasp.rviz
```

更改main.py 44行:模型名稱可選四種:
```
Block
TT_cube
Bolt
Sundries
```

執行夾取：
```
python scripts/main.py
```
# 提醒事項
1.常使用的套件如果沒有包含，請再自行安裝

2.cal4marker_V1.py 已沒在使用，請改用 V2 版本

