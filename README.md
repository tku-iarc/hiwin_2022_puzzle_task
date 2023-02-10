## Puzzle Task in HIWIN Competition

### Getting start

#### Clone this Repository
```
cd <your_workspace>/src
git clone --recurse-submodules https://github.com/tku-iarc/hiwin_puzzle_task.git
```

### Description

主程式		main.py
角度檢測	Angle_Detect.py
YOLO檢測	YOLO_Detect.py
座標校正	Correction.py
可視化工具	matlab.py

YOLO相關檔案包	cfg
手臂控制相關包	modbus_file


RoboticArm_Puzzle
	- main
		- main.py
		- Angle_Detect.py
		- matlab.py
		- YOLO_Detect.py
		- darknet.py
			- cfg
				- .data
				- .name
				- .cfg
				- weights
					- yolov4-tiny-obj_best.weights
			- modbus_file
				- Hiwin_API.c
				- Hiwin_API.so
