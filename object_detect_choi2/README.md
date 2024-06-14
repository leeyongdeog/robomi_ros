

pip install ultralytics


git clone https://github.com/ultralytics/yolov5  # clone


cd yolov5


pip install testresources



pip install -r requirements.txt  # install



git clone https://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@github.com/leeyongdeog/robomi_ros.git


cd robomi_ros


find . -maxdepth 1 ! -name 'object_detect_choi2' ! -name '.' -exec rm -rf {} +

cd object_detect_choi2


sudo apt install unzip


unzip this.zip



mv train ~/yolov5/
mv data.yaml ~/yolov5/

cd..



cd..


wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt



#gpu로 학습







python train.py --img 640 --epochs 50 --batch 16 --data data.yaml --cfg /home/ubuntu/yolov5/models/yolov5s.yaml --weights yolov5s.pt




















python train.py --img 640 --epochs 50 --batch 16 --data data.yaml --cfg /home/ubuntu/yolov5/models/yolov5s.yaml --weights yolov5s.pt --device cpu









python detect.py --weights /home/yolov5/runs/train/exp/weights/best.pt --img 416 --conf 0.25 --source 0


