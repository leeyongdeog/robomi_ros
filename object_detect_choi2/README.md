pip install ultralytics


git clone https://github.com/ultralytics/yolov5  # clone


cd yolov5




pip install -r requirements.txt  # install



unzip archive.zip





python train.py --img 640 --epochs 3 --data coco128.yaml --weights yolov5s.pt



python detect.py --weights /home/user/Desktop/exp7/weights/best.pt --img 416 --conf 0.25 --source 0
