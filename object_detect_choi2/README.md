

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

## 기존 m에서 s로 변경#
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt



#gpu로 학습







python train.py --img 640 --epochs 50 --data data.yaml --cfg 본인주소/yolov5/models/yolov5s.yaml --weights yolov5s.pt













#cpu로 학습







python train.py --img 640 --epochs 50 --data data.yaml --cfg 본인주소/yolov5/modeslyolov5m.yaml --weights yolov5m.pt --device cpu






실행하기 


python detect.py --weights 본인주소/yolov5/runs/train/exp/weights/best.pt --img 416 --conf 0.25 --source 0

exp1, exp2 이와 같은 exp 숫자는 학습 완료 시 출력됨
