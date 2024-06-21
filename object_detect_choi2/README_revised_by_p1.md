### 욜로 패키지 설치

```
pip install ultralytics
pip install testresources
cd /robomi_work
git clone https://github.com/ultralytics/yolov5   # clone
cd yolov5
pip install -r requirements.txt   # install
```

### 로보미 패키지 설치

```
cd /tmp   # 임시 디렉토리 활용
git clone https://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@github.com/leeyongdeog/robomi_ros.git
cd robomi_ros/object_detect_choi2
unzip this.zip   # 필요 시, sudo apt install unzip
cat data.yaml   # 다음 진행하는 디렉토리 경로로의 지정이 맞는지 확인
mv train /robomi_work/yolov5/
mv data.yaml /robomi_work/yolov5/
cd /robomi_work/yolov5/
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt
```

### 커스텀 데이터셋으로, 객체 인식 모델 학습

* CPU 이용 시
```
cd /robomi_work/yolov5/
python train.py --img 640 --epochs 50 --batch 16 --data  data.yaml --cfg ./models/yolov5s.yaml --weights  yolov5s.pt --device cpu   
```
* GPU 이용 시
```
cd cd /robomi_work/yolov5/
python train.py --img 640 --epochs 50 --batch 16 --data  data.yaml --cfg ./models/yolov5s.yaml --weights yolov5s.pt
```

※ 학습 진행 내용 중 `Validating runs/train/exp7/weights/best.pt...` 메시지 확인 필요

### 객체 탐지 기능 작동

```
find . -name best.pt   # ./exp2/weights/best.pt 확인 가능
```

* CPU 이용 시
```
cd /robomi_work/yolov5/
python detect.py --weights ./runs/train/exp2/weights/best.pt --img 416 --conf 0.25 --source 0 --device cpu
```
* GPU 이용 시
```
cd /robomi_work/yolov5/
python detect.py --weights ./runs/train/exp2/weights/best.pt --img 416 --conf 0.25 --source 0
```
