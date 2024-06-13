pip install ultralytics


git clone https://github.com/ultralytics/yolov5  # clone


cd yolov5




pip install -r requirements.txt  # install

unzip this.zip압축을 해제하고 yolov5폴더로 이동시킨다


data.yaml 파일을 yolov5/data에로 옮긴다

data.yaml에 train,val주소는 동일하게 설정

train: 본인주소/yolov5/train/images
val: 본인주소/yolov5/train/images



폴더 yolov5에서 아래명령 실행하기
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5m.pt




python train.py --img 640 --epochs 50 --data data.yaml --weights yolov5m.pt



python detect.py --weights 본인주소/yolov5/runs/train/exp/weights/best.pt --img 416 --conf 0.25 --source 0
