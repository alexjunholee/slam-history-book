# Ch.17 — LiDAR 평행 우주: LOAM에서 FAST-LIO까지

앞 장들에서 다룬 사진측량부터 Foundation 3D까지의 연구는 모두 카메라로 찍은 영상을 입력으로 받는다. MonoSLAM, PTAM, ORB-SLAM, DSO, DUSt3R가 다 그렇다. 같은 시기에 LiDAR를 쓰던 연구자들은 [ICP (Besl·McKay, 1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)와 Lu–Milios의 pose network를 바탕으로 따로 방법을 쌓아 왔다. 두 쪽은 서로의 논문을 거의 인용하지 않았고, 벤치마크와 학회도 달랐다.

Ji Zhang이 RSS에서 LOAM을 발표한 2014년에 카메라 쪽에서는 LSD-SLAM이 나왔고, 이듬해에는 ElasticFusion이 나왔다. LOAM은 이 방법들과 거의 관계없이 만들어졌고, 그 뒤 10년 동안에도 두 쪽이 서로 주고받은 것은 많지 않았다.

---

## 17.1 LOAM: edge와 plane으로 실시간 3D 정합

LOAM 전에도 레이저 스캔으로 지도를 만드는 방법은 있었다. [Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping" (Autonomous Robots)](https://doi.org/10.1023/A:1008854305733)은 2D 레인지 스캔 하나하나를 노드로, 스캔 사이의 상대 제약을 edge로 두고 궤적 전체를 한꺼번에 최적화했다. 이 "network of poses"는 뒤에 pose-graph SLAM의 원형이 된다(Ch.6 참조). 스캔끼리 맞추는 방법으로는 ICP 말고도 [Biber·Straßer 2003. "The Normal Distributions Transform" (IROS)](https://doi.org/10.1109/IROS.2003.1249285)의 NDT가 있었다. NDT는 공간을 셀로 나누고 셀마다 포인트를 가우시안 분포 하나로 요약한 다음, 새 스캔을 그 분포에 맞춘다. 뒤에 Magnusson이 이를 3D로 확장했다.

움직이면서 3D 스캔을 모으는 일은 더 까다로웠다. 포인트마다 측정 시각이 달라서, 그동안 센서가 움직인 양을 잘못 추정하면 포인트 클라우드가 어긋나게 쌓인다. LOAM 논문에 따르면 그때까지 일관된 3D 지도는 오프라인 일괄 처리로 만들었고, drift는 흔히 loop closure로 바로잡았다. LOAM은 같은 수준의 정확도를 실시간으로 내겠다고 나섰다.

당시 3D LiDAR는 비쌌다(가격은 §17.5). LOAM 저자들도 이 센서를 쓰지 않았다. 논문의 자체 실험에는 Hokuyo UTM-30LX 2D 레이저 스캐너를 모터로 돌려 만든 2축 LiDAR를 썼고, Velodyne 데이터는 KITTI 벤치마크로만 다뤘다.

Ji Zhang은 Carnegie Mellon 대학 Robotics Institute에서 Sanjiv Singh의 지도를 받아 [Zhang & Singh 2014. "LOAM: Lidar Odometry and Mapping in Real-time" (RSS)](https://www.roboticsproceedings.org/rss10/p07.pdf)를 냈다. LOAM은 포인트마다 smoothness $c$를 계산한다. $c$는 같은 scan line에서 양옆 포인트들과의 위치 차이를 더해 구한 값으로, 그 자리의 표면이 얼마나 매끄러운지를 나타낸다. $c$가 가장 큰 포인트들은 **edge point**로, 가장 작은 포인트들은 **planar point**로 고르고, 매칭에는 이 두 집합만 쓴다. edge point는 직전 sweep에서 찾은 edge line까지의 거리로, planar point는 직전 sweep에서 찾은 평면까지의 거리로 제약을 건다. 전체 포인트 대신 적은 수의 feature만 맞추므로 실시간으로 돌릴 만큼 계산량이 줄어든다.

LOAM은 알고리즘 두 개를 함께 돌린다. Lidar Odometry는 약 10 Hz로 스캔 사이의 6-DoF 움직임을 거칠게 추정하고, Lidar Mapping은 1 Hz로 왜곡을 보정한 스캔을 지도에 정밀하게 맞춘다. 두 결과를 합친 위치는 약 10 Hz로 나온다. 이렇게 빠르고 거친 계산과 느리고 정밀한 계산을 나눠 실시간으로 돌면서도 drift(누적 위치 오차)를 줄였다. 이 구조는 뒤에 나온 LeGO-LOAM이 그대로 이어받았다.

2026년 10월 현재 KITTI odometry 공개 순위표에 올라 있는 LOAM의 평균 translation error는 0.55%다.

> 🔗 **차용.** LOAM의 포인트 정합은 Besl·McKay(1992)의 ICP에서 출발한다.

---

## 17.2 LeGO-LOAM: 지면을 먼저 분리하다

LOAM은 지면 포인트를 다른 포인트와 섞어서 처리했다.

Stevens Institute of Technology의 Robust Field Autonomy Lab에서 Tixiao Shan과 지도교수 Brendan Englot은 [Shan & Englot 2018. LeGO-LOAM](https://doi.org/10.1109/IROS.2018.8594299)에서 지면 분리를 파이프라인 맨 앞에 두었다. 포인트 클라우드를 range image(픽셀마다 거리를 담은 2D 영상)로 펼쳐 지면 포인트를 먼저 떼어 내고, 남은 포인트를 묶음으로 나눈다(segmentation). 최적화도 두 단계로 나눴다. 먼저 지면의 planar feature로 높이($t_z$)와 roll, pitch를 구하고, 이어서 edge feature로 나머지 $t_x$, $t_y$, yaw를 구한다. 두 단계 모두 Levenberg-Marquardt로 푼다.

저자들은 지상 차량으로 울퉁불퉁한 지형에서 모은 데이터로 실험해, LeGO-LOAM이 LOAM보다 적은 계산으로 비슷하거나 더 나은 정확도를 낸다고 보고했다. VLP-16 데이터는 저전력 임베디드 보드인 NVIDIA Jetson TX2에서 실시간으로 처리했다. 공개 구현은 수평으로 단 VLP-16을 기준으로 맞춰져 있고, [README](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)는 스캔에 지면이 늘 있다고 가정한다고 밝힌다.

같은 해 Bonn 대학의 Jens Behley와 Cyrill Stachniss는 실외 LiDAR에 **surfel**(surface element, 작은 원반 모양의 표면 조각)을 썼다. [Behley & Stachniss 2018. "Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments" (RSS)](http://www.roboticsproceedings.org/rss14/p16.pdf)의 SuMa는 지도를 surfel로 만들고, 새 스캔을 그 지도에 맞춘다(scan-to-model). Ch.9에서 본 ElasticFusion이 실내 RGB-D(Kinect) 데이터에 쓰던 표현을 실외 Velodyne 데이터에 가져온 것이다. 후속작 [Chen et al. 2019. "SuMa++" (IROS)](https://doi.org/10.1109/IROS40897.2019.8967704)는 semantic segmentation 결과를 surfel에 붙여 움직이는 물체를 걸러 냈다. 2018년 무렵에는 LOAM처럼 feature를 골라 쓰는 방식, LeGO-LOAM처럼 지면을 먼저 나누는 방식, SuMa처럼 surfel 지도를 쓰는 방식이 함께 쓰였다.

---

## 17.3 FAST-LIO: tightly coupled LiDAR-IMU

회전식 LiDAR는 보통 1초에 10번 한 바퀴를 돈다. 한 바퀴를 도는 동안 센서가 빠르게 움직이면 처음과 끝의 센서 위치가 달라져 포인트 클라우드가 일그러진다. 이것을 motion distortion이라고 한다. LOAM은 한 sweep 동안 속도가 일정하다고 보고 이 왜곡을 보정했는데, 움직임이 급하면 이 가정이 맞지 않는다.

IMU는 이보다 훨씬 자주 측정하므로 스캔과 스캔 사이를 채울 수 있다. 두 센서를 합치는 방식에는 loosely coupled와 tightly coupled가 있다. loosely coupled 방식은 센서마다 따로 추정한 결과를 나중에 합친다. tightly coupled 방식은 추정기 하나에 두 센서의 관측을 함께 넣는다. 그만큼 두 센서 사이의 관계를 더 많이 쓰지만, 실제 성능은 모델과 보정, 구현에 따라 달라진다.

University of Hong Kong(HKU) MaRS Lab의 Wei Xu와 지도교수 Fu Zhang은 2021년 RA-L에 [FAST-LIO](https://arxiv.org/abs/2010.08196)를 발표했다. FAST-LIO는 iterated Extended Kalman Filter(iEKF, 반복 확장 칼만 필터)로 LiDAR feature point와 IMU 측정을 합친다. 측정값이 많아도 계산이 불어나지 않도록 Kalman gain을 새로 유도해, 계산량이 측정 개수가 아닌 상태 차원에 따라 정해지게 했다. 저자들은 쿼드로터에 실은 컴퓨터에서 한 스캔의 유효 feature point를 1,200개 넘게 합치면서 iEKF 한 단계의 반복을 모두 25 ms 안에 마쳤다고 보고했다. iEKF는 측정 업데이트를 할 때 새로 얻은 추정치에서 다시 선형화해 계산을 되풀이하므로, 한 번만 선형화하는 EKF보다 선형화 오차를 줄일 수 있다.

이듬해 TRO에 실린 FAST-LIO2([Xu et al. 2022](https://doi.org/10.1109/TRO.2022.3141876))는 feature를 따로 뽑지 않고 원시 포인트를 바로 지도에 맞췄다. 손으로 설계한 feature 추출 단계가 없어지자 스캔 방식이 다른 LiDAR에도 그대로 쓸 수 있게 됐다. 지도는 ikd-Tree(incremental kd-Tree)에 담았다. ikd-Tree는 포인트를 넣고 지우는 일을 트리 전체를 다시 만들지 않고 처리하며, 필요할 때 균형을 다시 맞춘다. 저자들은 공개 데이터셋 19개 시퀀스로 비교했고, 넓은 실외에서 odometry와 mapping을 최대 100 Hz로 돌렸다고 보고했다. 실험 플랫폼은 UAV, 핸드헬드 장치, 자동차였다.

같은 연구실의 [He et al. 2023. "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry" (Advanced Intelligent Systems)](https://doi.org/10.1002/aisy.202200459)는 포인트를 스캔 하나로 모으지 않았다. Point-LIO는 LiDAR 포인트가 하나 들어올 때마다 그 포인트의 측정 시각에서 상태를 갱신하므로, 스캔 안에서 생기는 motion distortion이 처음부터 생기지 않는다. 실험에서 odometry 출력은 4~8 kHz였다. IMU 측정값을 운동 모델의 출력으로 두어서, IMU가 측정 범위를 넘어 포화된 상태에서도 초당 75 rad로 도는 움직임을 추정했다. 공개 데이터셋 비교에서는 정확도와 계산 시간 모두 다른 방법과 비슷했다고 저자들은 보고했다.

> 🔗 **구분.** FAST-LIO는 raw IMU 측정으로 상태를 앞으로 전파(forward propagation)하고, 스캔 안의 각 포인트를 거꾸로 전파(backward propagation)해 보정한 다음 iEKF로 갱신한다. [Forster et al. 2017. "On-Manifold Preintegration" (TRO)](https://doi.org/10.1109/TRO.2016.2597321)의 IMU preintegration factor는 factor graph에서 쓰는 방식이고, 이 장에서는 LIO-SAM이 이 방식을 쓴다.

---

## 17.4 LIO-SAM: LiDAR와 IMU를 factor graph로 묶다

그 무렵 visual-inertial 시스템에서는 factor graph가 널리 쓰였다. LiDAR 쪽에도 Lu–Milios의 pose network로 거슬러 올라가는 graph SLAM 연구가 있었다. [GTSAM](https://gtsam.org/)은 Georgia Tech의 Frank Dellaert 연구실이 만든 factor graph 라이브러리로, 여러 센서의 제약을 factor로 엮어 최적화한다.

Tixiao Shan은 LeGO-LOAM의 다음 작업으로 [Shan et al. 2020. LIO-SAM](https://doi.org/10.1109/IROS45743.2020.9341176)을 IROS에 발표했다. LIO-SAM은 GTSAM의 factor graph로 LiDAR-IMU odometry를 푼다. 상대 측정과 절대 측정을 출처에 상관없이 factor로 넣을 수 있어서, IMU preintegration, LiDAR odometry, GPS, loop closure를 한 그래프에 함께 올렸다. IMU preintegration으로 추정한 움직임은 포인트 클라우드의 왜곡을 펴고 LiDAR odometry 최적화의 초기값이 된다. 실시간 성능을 위해 새 스캔을 전체 지도가 아닌 최근 keyframe 몇 개로 만든 국소 지도에 맞춘다.

[공개 구현](https://github.com/TixiaoShan/LIO-SAM)은 실행 내내 유지되는 factor graph에서 LiDAR odometry factor와 GPS factor를 최적화한다. 또 다른 factor graph는 IMU와 LiDAR odometry factor로 IMU bias를 추정하고, IMU 주기로 위치를 내기 위해 주기적으로 초기화된다. README는 loop closure 기능을 LeGO-LOAM에서 그대로 가져온 ICP 기반의 예제라고 설명하고, 더 나은 구현으로 Scan Context를 권한다.

2023년 Bonn 대학의 [Vizzo et al. 2023. "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L)](https://doi.org/10.1109/LRA.2023.3236571)은 이름(Keep It Small and Simple)대로 구성 요소를 대부분 덜어 냈다. 남은 것은 point-to-point ICP, 대응점(correspondence)을 고르는 거리 기준을 상황에 맞춰 바꾸는 adaptive threshold, robust kernel, 단순한 motion compensation, point cloud subsampling이다. IMU 없이 3D point cloud만 입력으로 받는다. 저자들은 파라미터를 바꾸지 않고 자동차, UAV, segway, 핸드헬드 LiDAR 데이터에서 최신 방법과 비슷한 성능을 냈다고 보고했다.

---

## 17.5 센서 가격 하락과 보급: 2007–2024

2007년 DARPA Urban Challenge에서 주요 팀들이 단 Velodyne HDL-64E는 한 대에 75,000달러였다. 이 센서는 주로 자율주행 연구팀과 국방 과제에서 썼다. 2011년 IEEE Spectrum이 소개한 Google 자율주행 Prius도 지붕에 Velodyne 64빔 LiDAR를 달았다([IEEE Spectrum 2011](https://spectrum.ieee.org/how-google-self-driving-car-works)). Velodyne은 2014년 9월 16채널 VLP-16을 [7,999달러에 내놓았지만](https://www.prweb.com/releases/breaking_through_the_price_barrier_for_lidar_sensors_velodyne_introduces_3d_real_time_lidar_puck_at_7_999/prweb12139477.htm), 이 값도 연구실 예산에서 작은 돈은 아니었다.

2019년에는 값이 훨씬 다양해졌다. DJI에서 나온 중국 업체 Livox는 그해 [Mid-40의 미국 소매가를 599달러로 발표했다](https://www.livoxtech.com/news/1). 같은 해 1월 Ouster는 128채널 OS1-128을 [18,000달러에 내놓았다](https://ouster.com/insights/blog/introducing-the-os-1-128-lidar-sensor). Ouster는 2020년 10월에 solid-state 센서 ES2를 발표하면서 [2024년 차량 양산용 예상 가격을 600달러로 제시했다](https://investors.ouster.com/news-releases/news-release-details/ouster-announces-first-high-performance-true-solid-state-digital). 이 제품들은 채널 수, 시야각, 측정 거리가 다르고, 소매가와 양산 예상가는 매기는 기준도 다르다.

센서가 싸지고 종류가 늘자 알고리즘이 풀어야 할 문제도 생겼다. Solid-state LiDAR는 회전식보다 시야각(FoV)이 좁고, Livox 일부 제품은 같은 경로를 되풀이하지 않는 스캔 패턴을 쓴다. 360° 회전 스캔의 scan line 구조를 전제로 만든 원래 LOAM은 이런 센서에 그대로 쓰기 어렵다. FAST-LIO와 FAST-LIO2는 회전식과 solid-state LiDAR를 모두 지원하고, 저자들은 시야각이 좁은 solid-state LiDAR로 한 실험 결과도 보고했다.

---

## 17.6 Visual SLAM과 LiDAR SLAM이 따로 발전한 이유

두 센서는 재는 것이 다르다. 카메라는 밝기와 색을 기록하고, LiDAR는 거리를 재서 형상을 얻는다. 그래서 카메라 기반 방법은 keypoint, descriptor, photometric consistency(같은 점은 영상마다 밝기가 같다는 가정)를 중심으로 발전했고, LiDAR 방법은 edge, plane, range image를 주로 썼다. place recognition(이미 와 본 곳인지 알아보는 일)에서도 카메라 쪽은 DBoW2나 NetVLAD처럼 이미지의 겉모습(appearance)을 쓰고, LiDAR 쪽은 [Scan Context(Kim·Kim, IROS 2018)](https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf)나 [PointNetVLAD](https://arxiv.org/abs/1804.03492)처럼 point cloud의 3D 구조를 쓴다.

논문을 내는 곳도 달랐다. 이 장에서 다룬 LiDAR 방법은 LOAM이 RSS, LeGO-LOAM과 LIO-SAM이 IROS, FAST-LIO가 RA-L, FAST-LIO2가 TRO에 실렸다. 앞 장들의 카메라 기반 방법은 로봇공학 학회지 말고도 컴퓨터 비전 학회에 자주 실렸다. PTAM은 ISMAR, DTAM은 ICCV, LSD-SLAM은 ECCV, DUSt3R는 CVPR에 나왔다.

2020년대 초부터 두 분야를 잇는 연구가 나왔다. Tixiao Shan은 [LVI-SAM (ICRA 2021)](https://arxiv.org/abs/2104.10831)에서 LIO-SAM에 visual-inertial 시스템을 붙였다. LVI-SAM은 factor graph 하나로 visual-inertial system(VIS)과 lidar-inertial system(LIS)을 묶는다. VIS는 LIS의 추정으로 초기화하고 LiDAR로 시각 feature의 깊이를 얻는다. LIS는 VIS의 추정을 scan matching의 초기값으로 쓴다. 한쪽이 실패해도 다른 쪽만으로 동작한다.

---

## 17.7 Visual-LiDAR 수렴 시도: 2024-2025

센서 종류에 상관없이 feature를 뽑는 foundation model이 나오면서, 카메라와 LiDAR를 한 프레임워크에서 다루려는 연구도 늘었다.

한쪽은 LiDAR와 카메라 데이터를 같은 embedding space(특징 벡터 공간)에 놓는 multi-modal pretrained feature를 만든다. [CLIP(Radford et al., 2021)](https://arxiv.org/abs/2103.00020)이 이미지와 텍스트를 contrastive learning(같은 대상의 짝은 가깝게, 다른 짝은 멀게 학습하는 방법)으로 맞춘 것처럼, LiDAR와 이미지 사이에 같은 학습을 적용한다.

다른 쪽은 센서 출력을 geometric primitive나 neural field 같은 공통 표현으로 바꾼 다음 backend 하나에서 처리하려 한다.

그래도 이 장에서 다룬 LiDAR 방법과 앞 장들의 카메라 방법은 대부분 한 종류의 센서를 중심으로 설계됐다.

---

## 17.8 Radar는 본 책의 scope 밖이다

Radar SLAM은 독립된 하위 분야로 자랐다. 하드웨어는 Navtech 계열의 회전식 radar와, 칩 하나에 송수신부를 넣은 4D mmWave radar로 나뉜다. 4D mmWave radar는 Doppler 효과로 각 반사점이 센서 쪽으로 다가오는 속도(radial velocity)를 직접 재므로, 스캔 사이의 대응점을 찾지 않고도 자기 움직임을 구할 수 있다. 어느 쪽이든 speckle, multipath, receiver saturation처럼 전파에서 생기는 노이즈를 따로 모델링해야 한다. 연구는 Oxford의 [Cen & Newman 2018](https://doi.org/10.1109/ICRA.2018.8460687)에서 시작해 Adolfsson·Magnusson의 CFEAR와 후속 TBV-SLAM, Burnett·Barfoot의 continuous-time 방법으로 이어졌다. 벤치마크로는 Oxford Radar RobotCar, Boreas, MulRan 같은 radar 전용 데이터셋을 쓴다. radar는 비, 안개, 연기 속에서도 측정할 수 있다. 다만 이 책이 따라온 photogrammetry → SfM → Visual SLAM → learning → 3D foundation의 흐름과 겹치는 데가 적어 여기서는 다루지 않는다. 자세한 내용은 *SLAM Handbook*(Cambridge University Press, 2026년 4월 21일자 원고) 9장 "Radar SLAM"을 보라.

---

## 📜 예언 vs 실제

> Zhang·Singh는 2014년 LOAM 논문의 결론에서 loop closure로 drift를 바로잡는 일과, 자신들의 출력을 IMU와 함께 Kalman filter로 합쳐 drift를 더 줄이는 일을 future work로 적었다. 이 일들은 10년 안에 이루어졌다. loop closure는 LIO-SAM(2020)이 factor graph에 넣었고, IMU 결합은 FAST-LIO(2021)와 FAST-LIO2(2022)가 iEKF 기반 tightly coupled 방식으로 풀었다.

---

## 🧭 아직 열린 것

**Visual+LiDAR 완전 융합.** LVI-SAM 뒤에도 두 센서를 상태 추정기 하나에서 tightly coupled로 다루는 방식은 연구마다 설계가 다르다. 두 센서를 함께 쓰려면 센서 사이 캘리브레이션이 필요하다. 한 추정기에서 두 센서를 함께 다루는 표준 설계를 정하는 일은 남은 과제다.

**Solid-state LiDAR에 맞춘 알고리즘.** 원래 LOAM은 360° 회전식 LiDAR의 scan line 구조를 전제로 했다. Livox 일부 제품의 비반복 스캔과 solid-state 센서의 좁은 FoV에서는 어떤 방향의 움직임을 관측할 수 있는지(observability)와 motion distortion이 생기는 양상이 달라진다. FAST-LIO2처럼 원시 포인트를 지도에 바로 맞추는 방식이나 Livox용 LOAM 구현([loam_livox](https://github.com/hku-mars/loam_livox))처럼 이런 센서를 다루는 시스템은 이미 있다. 서로 다른 FoV와 스캔 패턴을 설정 하나로 모두 처리하는 일은 아직 남아 있다.

**동적 물체 처리.** LOAM 논문이 future work로 꼽지 않은 문제다. SLAM은 오래전부터 환경이 움직이지 않는다고 가정했고, LiDAR SLAM도 이 가정을 따른다. 움직이는 점을 떼어 내는 방법으로는 §17.2의 SuMa++처럼 semantic segmentation 결과를 쓰는 방식과, Ch.15b의 ERASOR와 Dynablox처럼 기하 정보로 동적 점을 고르는 방식이 있다.
