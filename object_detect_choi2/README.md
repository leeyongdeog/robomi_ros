pip install ultralytics


git clone https://github.com/ultralytics/yolov5  # clone


cd yolov5




pip install -r requirements.txt  # install


wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt



unzip archive.zip





python train.py --img 640 --epochs 3 --data data.yaml --weights yolov5m.pt



python detect.py --weights /home/user/Desktop/exp7/weights/best.pt --img 416 --conf 0.25 --source 0
