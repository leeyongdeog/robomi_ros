

pip install ultralytics


git clone https://github.com/ultralytics/yolov5  # clone


cd yolov5


pip install testresources



pip install -r requirements.txt  # install





unzip this.zip





#압축을 해제하고 yolov5폴더로 이동시킨다


#data.yaml 파일을 yolov5/data에로 옮긴다

#data.yaml에 train,val주소는 동일하게 설정

#train: 본인주소/yolov5/train/images
#val: 본인주소/yolov5/train/images



#폴더 yolov5에서 실행하기
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5m.pt



#gpu로 학습







python train.py --img 640 --epochs 50 --data data.yaml --cfg 본인주소/yolov5/models/yolov5m.yaml --weights yolov5m.pt













#cpu로 학습







python train.py --img 640 --epochs 50 --data data.yaml --cfg 본인주소/yolov5/modeslyolov5m.yaml --weights yolov5m.pt --device cpu






실행하기 


python detect.py --weights 본인주소/yolov5/runs/train/exp/weights/best.pt --img 416 --conf 0.25 --source 0

exp1, exp2 이와 같은 exp 숫자는 학습 완료 시 출력됨
