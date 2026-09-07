# Ch.17 — LiDAR 평행 우주: LOAM에서 FAST-LIO까지

사진측량에서 Foundation 3D에 이르는 계보는 센서가 카메라라는 공통 전제 위에 서 있다. MonoSLAM·PTAM·ORB-SLAM·DSO·DUSt3R는 모두 픽셀로 세계를 읽는 전통 안에 있다. 같은 시간, 같은 로봇공학 커뮤니티 안에서 전혀 다른 계보가 자라고 있었다. LiDAR 계보는 카메라 진영의 keypoint·photometric consistency·feature descriptor와 무관하게, ICP의 뼈대 위에서 자체적인 문법을 만들어냈다. 두 계보는 인용 관계가 드물었고, 주로 쓰는 벤치마크와 발표 무대도 달랐다.

Ji Zhang은 2014년 RSS에서 LOAM을 발표했다. 그해 Visual 진영에서는 LSD-SLAM 계열이 전개되고 있었고, ElasticFusion은 이듬해인 2015년에 발표됐다. LOAM은 카메라 기반 방법론이나 코드와 직접 연결되지 않았고, 연구 커뮤니티의 겹침도 적었다. 두 계보는 같은 로보틱스라는 이름 아래에서 접점이 드문 채 10년을 달렸다. LOAM은 [ICP (Besl·McKay, 1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)의 오래된 뼈대 위에 섰다. 다만 graph SLAM에는 Lu–Milios의 laser pose network처럼 LiDAR 쪽의 오래된 계보도 있었다. 평행 우주는 제한된 교류 속에서 성숙했다.

---

## 17.1 LOAM: edge와 plane, 그리고 KITTI의 점령

2014년, Google의 Waymo 전신 프로그램이 도로 위를 달리고 있었고, DARPA Urban Challenge의 여파가 채 가시지 않은 때였다. Velodyne HDL-64E는 한 대에 75,000달러였다. 당시 LiDAR 연구는 이 장비를 감당할 수 있는 대형·고예산 연구 그룹에 주로 한정됐다. CMU Robotics Institute의 Autonomous Mobile Robot Lab(Sanjiv Singh 교수 연구실)이 그중 하나였다.

LOAM 이전에도 LiDAR로 지도를 쌓는 시도는 있었다. [Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping" (Autonomous Robots)](https://doi.org/10.1023/A:1008854305733)은 2D 레인지 스캔을 노드로 두고 스캔 간 상대 제약을 edge로 묶어 전체 궤적을 동시 최적화하는 방식을 제안했고, 이 "network of poses" 발상은 훗날 pose-graph SLAM의 원형이 된다(Ch.6 참조). 매칭 자체는 Besl·McKay의 ICP 외에도 [Biber·Straßer 2003. "The Normal Distributions Transform" (IROS)](https://doi.org/10.1109/IROS.2003.1249285)가 제안한 NDT(셀 단위 가우시안 분포에 정렬하는 분포 기반 매칭)와 이후 Magnusson의 3D 확장이 ICP 대안으로 공존했다. 이 모두가 2D 또는 오프라인 3D였다. LOAM의 몫은 실시간 3D였다.

Ji Zhang은 Singh의 지도 아래 [Zhang & Singh 2014. "LOAM: Lidar Odometry and Mapping in Real-time" (RSS)](https://www.roboticsproceedings.org/rss10/p07.pdf)를 냈다. LiDAR 포인트를 두 종류의 feature로 분류했다. **edge point**는 smoothness $c$가 높은 지점(곡률 높음), **planar point**는 $c$가 낮은 지점(곡률 낮음). ICP처럼 포인트 전체를 등록하지 않고 이 두 feature 집합만 매칭한다. edge point는 이웃 scan의 edge line에, planar point는 이웃 scan의 local plane에 point-to-line·point-to-plane 거리로 제약을 건다. 계산 비용이 낮아진다. 실시간 가능성이 열린다.

알고리즘 구조는 두 단계로 나뉜다. Lidar Odometry는 스캔 간 6-DoF 변환을 10Hz에서 추정한다. Lidar Mapping은 더 낮은 주파수(1Hz)에서 전체 맵과 정합해 오차를 보정한다. 고주파 odometry와 저주파 mapping을 분리해 drift를 억제하면서도 실시간성을 유지한다. 이 two-tier 구조는 이후 LiDAR SLAM의 기본 문법이 된다.

KITTI 공개 비교표에서 LOAM은 발표 뒤 선두권에 들었다. 시퀀스 00의 relative translation error로 널리 인용되는 값은 0.78%이고, 전체 시퀀스 평균은 0.84%다. 이는 당시의 경쟁력을 보여주지만, 센서와 평가 조건이 다른 visual odometry에 대한 보편적 우위를 뜻하지는 않는다.

> 🔗 **차용.** LOAM의 feature-based 포인트 등록은 Besl·McKay(1992)의 ICP에서 출발한다. 차이는 edge와 planar feature만 선택적으로 매칭한다는 점이다. 고전 등록을 선별적으로 재사용함으로써 속도와 정밀도 모두를 얻었다.

---

## 17.2 LeGO-LOAM: 땅을 먼저 잘라낸다

LOAM의 문제는 지면(ground plane)을 명시적으로 다루지 않는다는 점이었다. 실외 자율주행 환경에서 포인트 클라우드의 상당 비율은 도로면이 차지한다. 이를 edge/planar feature로 한데 처리하면 매칭 노이즈가 생긴다.

Stevens Institute of Technology의 Robust Field Autonomy Lab에서 Tixiao Shan과 지도교수 Brendan Englot은 [Shan & Englot 2018. LeGO-LOAM](https://doi.org/10.1109/IROS.2018.8594299)에서 ground segmentation을 첫 단계로 분리했다. 포인트 클라우드를 range image로 투영한 뒤, 지면 포인트를 먼저 분리하고 비지면 포인트를 다시 클러스터링한다. Ground는 roll·pitch 추정에, 클러스터는 yaw·translation 추정에 각각 사용된다. 두 단계 최적화다.

결과는 LOAM 대비 연산 절감이었다. 원래 LOAM이 Velodyne VLP-16에서 실시간 동작이 버거웠다면, LeGO-LOAM은 동일 센서에서 임베디드 플랫폼(NVIDIA Jetson)에서도 돌아간다. 경량화의 대가는 있다. 포인트 희소 환경이나 지면 구조가 불규칙한 환경(레이저가 가리는 구간, 울퉁불퉁한 야지, 건물 내부)에서는 segmentation이 실패하고 odometry가 흔들린다.

LeGO-LOAM은 "센서 입력을 구조화된 모듈로 전처리한 뒤 odometry를 돌린다"는 설계 원칙을 남겼다. FAST-LIO와 LIO-SAM이 뒤에 이 원칙을 받아들인다.

LeGO-LOAM과 같은 시기, Bonn 대학의 Jens Behley와 Cyrill Stachniss는 edge/plane feature가 아니라 **surfel**(surface element)을 outdoor LiDAR에 가져왔다. [Behley & Stachniss 2018. "Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments" (RSS)](http://www.roboticsproceedings.org/rss14/p16.pdf)의 **SuMa**는 각 포인트 이웃을 원반 모양 surfel로 요약해 scan-to-model 등록을 수행했고, 후속 [Chen et al. 2019. "SuMa++" (IROS)](https://doi.org/10.1109/IROS40897.2019.8967704)는 semantic segmentation을 결합해 움직이는 물체를 surfel 수준에서 걸러냈다. Kinect 실내 RGB-D 계보(Ch.9)에서 ElasticFusion이 쓰던 surfel representation이 outdoor Velodyne으로 건너온 순간이다. feature 선택(LOAM), segmentation 선행(LeGO-LOAM), surfel 누적(SuMa)의 세 갈래가 2018년 전후로 동시에 경쟁하고 있었다.

---

## 17.3 FAST-LIO — tightly coupled LiDAR-IMU

LiDAR의 스캔 주파수는 10-20Hz다. 그 사이사이에서 빠른 움직임이 있으면 포인트 클라우드에 motion distortion이 생긴다. 스캔이 끝나는 순간의 센서 위치와 시작 순간의 위치가 다르기 때문이다. 고속 이동체에서 LOAM 계열이 흔들리는 주된 이유가 여기에 있다.

IMU는 100-400Hz로 동작한다. LiDAR의 틈을 채우기에 충분하다. 그런데 LiDAR와 IMU를 어떻게 결합하느냐에 따라 성능이 갈린다. loosely coupled는 각각 독립적으로 추정한 뒤 fusion하고, tightly coupled는 한 추정기 안에서 원시 관측이나 잔차 수준의 제약을 함께 사용한다. tight coupling은 센서 사이의 제약을 더 많이 보존할 수 있지만, 모델·보정·구현 조건과 무관하게 항상 우월한 것은 아니다.

Hong Kong University(HKU) MaRS Lab의 Wei Xu와 지도교수 Fu Zhang은 2021년 RA-L에 [**FAST-LIO**](https://arxiv.org/abs/2010.08196)를 발표했다. 드론 제어 연구실에서 나온 논문으로, 로터 진동이 심하고 기동이 빠른 UAV에서도 LiDAR odometry가 버텨야 했다. 이들이 사용한 도구는 **iterated Extended Kalman Filter(iEKF)**였다. iEKF는 측정 업데이트 단계에서 선형화 점을 현재 추정치로 반복 갱신한다. 한 번의 linearization으로 끝내는 기본 EKF보다 측정 모델의 선형화 오차를 줄일 수 있지만, 개선 폭은 초기 추정과 운동·관측 조건에 달려 있다.

이듬해 TRO에 발표한 **FAST-LIO2**([Xu et al. 2022](https://doi.org/10.1109/TRO.2022.3141876))는 ikd-Tree를 추가했다. 기존 kd-Tree는 포인트가 추가될 때마다 재구성 비용이 크다. ikd-Tree는 부분 재구성만 수행하는 incremental 방식이다. 맵 포인트가 수백만 개에 달해도 실시간 nearest-neighbor 탐색이 가능하다. 실험에서는 UAV·핸드헬드·자율주행차에서 일관된 성능이 나왔다. 드론 환경에서도 drift가 낮게 유지됐다.

FAST-LIO 계보의 다음 수는 motion distortion을 아예 없애는 쪽이었다. 같은 MaRS Lab에서 나온 [He et al. 2023. "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry" (Advanced Intelligent Systems)](https://doi.org/10.1002/aisy.202200459)는 LiDAR 포인트가 들어올 때마다 state를 갱신한다. point-by-point 관측 업데이트다. 각 포인트를 자기 시각에서 바로 fusion해 왜곡이 발생할 틈을 지운다. 고기동 플랫폼에서 FAST-LIO2보다 drift가 줄어든 것이 보고됐다.

> 🔗 **구분.** FAST-LIO는 raw IMU를 이용한 상태 전파와 LiDAR 측정의 iterated Kalman update를 결합한다. [Forster et al. 2016. "On-Manifold Preintegration" (TRO)](https://doi.org/10.1109/TRO.2016.2597321)의 preintegrated factor를 iEKF로 재구현한 구조가 아니다. 두 방법은 IMU를 결합하지만 추정기 형식이 다르다.

---

## 17.4 LIO-SAM: LiDAR와 IMU를 factor graph로 묶다

같은 시기, visual-inertial 시스템에서는 factor graph가 널리 쓰였고, LiDAR 쪽에도 Lu–Milios의 pose network처럼 더 오래된 graph-SLAM 계보가 있었다. [GTSAM (Dellaert·Kaess, 2012)](https://gtsam.org/)은 여러 센서의 제약을 factor로 함께 구성하는 도구를 제공했다.

Tixiao Shan이 LeGO-LOAM 이후 낸 [Shan et al. 2020. LIO-SAM](https://doi.org/10.1109/IROS45743.2020.9341176)은 GTSAM의 factor graph를 LiDAR-IMU 시스템의 backend로 명시적으로 채택했다. [공개 구현](https://github.com/TixiaoShan/LIO-SAM)은 두 그래프를 유지한다. 장기 mapping 그래프는 LiDAR odometry, GPS, loop closure 제약을 keyframe 사이에 쌓는다. 별도의 IMU preintegration 그래프는 관성 측정과 LiDAR odometry로 상태·bias를 추정하며, 계산량을 제한하려고 주기적으로 초기화된다.

> 🔗 **차용.** LIO-SAM은 GTSAM의 factor graph에 IMU preintegration·LiDAR odometry·GPS·loop closure factor를 함께 구성했다. 다만 graph SLAM 자체는 Lu–Milios의 laser pose network를 포함한 LiDAR 계보에도 이미 있었으므로, 이를 Visual SLAM에서 LiDAR로 일방적으로 건너온 구조라고 볼 수는 없다.

LIO-SAM은 FAST-LIO2와 달리 loop closure를 포함해 장기 궤적의 누적 drift를 보정할 수 있다. 반면 계산 비용이 높고 GPS나 추가 sensor input이 없으면 factor graph의 강점이 줄어든다. 두 시스템은 설계 목표가 다르다. FAST-LIO2는 실시간 LiDAR-IMU 구성의 속도와 정밀도를, LIO-SAM은 다중 센서 long-term mapping의 일관성을 우선한다.

역설적인 회귀도 있었다. LOAM 이후 10년 가까이 LiDAR odometry는 feature 선택·surfel·neural descriptor로 점점 복잡해지는 쪽을 달렸는데, 2023년 Bonn 대학의 [Vizzo et al. 2023. "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L)](https://doi.org/10.1109/LRA.2023.3236571)은 반대 방향을 냈다. feature 추출도, 학습된 descriptor도 없이, 적응형 threshold로 튜닝이 거의 필요 없는 point-to-point ICP 하나로 KITTI에서 경쟁력 있는 odometry를 보였다. 이름 그대로 Keep It Small and Simple이다. 이 결과는 motion compensation, sampling, robust correspondence 처리 같은 구현 선택을 잘 결합하면 고전 등록법도 경쟁력이 있음을 보였다. 이를 LOAM이 등장한 역사적 원인에 대한 저자들의 평가로 읽을 필요는 없다.

---

## 17.5 센서 가격 하락과 보급: 2007–2024

LiDAR SLAM의 역사에서 기술 논문 못지않게 중요한 것이 센서 가격이다.

2007년 DARPA Urban Challenge에서 주요 팀들이 장착한 Velodyne HDL-64E는 대당 75,000달러였다. 자율주행 연구팀이나 국방 프로젝트가 아니면 접근하기 어려운 장비였다. 2012년에도 HDL-32E가 30,000달러 수준. LOAM이 발표된 2014년에는 VLP-16이 7,999달러로 내려왔지만 여전히 연구 예산의 상당 부분이었다.

그 뒤 LiDAR의 가격대가 크게 넓어졌다. 중국 업체 Livox(DJI 계열)는 2019년 [Mid-40의 미국 소매가를 599달러로 발표했다](https://www.livoxtech.com/news/1). 같은 해 Ouster의 128채널 OS1-128은 18,000달러였고, Ouster는 2020년 [양산 프로그램용 solid-state ES2의 2024년 목표 가격을 600달러로 제시했다](https://investors.ouster.com/news-releases/news-release-details/ouster-announces-first-high-performance-true-solid-state-digital). 수백 달러대 제품과 양산 목표가가 등장한 것은 분명한 변화였지만, 이를 모든 LiDAR의 일률적인 100배 인하로 읽을 수는 없다. 채널 수·시야각·거리 성능이 다르고 소매가와 대량 양산 목표가도 서로 다른 기준이기 때문이다.

센서가 보급되어도 알고리즘 문제가 함께 해결된 것은 아니었다. Solid-state LiDAR는 spinning 타입보다 시야각(FoV)이 제한적이고, 제품에 따라 비반복 스캔 패턴을 쓴다. 360° 회전 스캔을 전제로 설계된 원래 LOAM은 그대로 적용하기 어렵다. 반면 FAST-LIO와 FAST-LIO2는 기계식·solid-state LiDAR를 함께 다루는 방향으로 설계됐고, 작은 FoV와 불규칙한 sampling에서도 작동하는 사례를 보고했다. 저가 센서의 확산은 알고리즘을 없앤 것이 아니라 관측 범위와 scan pattern에 맞춘 새 과제를 만들었다.

---

## 17.6 Visual-LiDAR 계보 분리의 원인

Visual SLAM과 LiDAR SLAM은 동시대에 발전했지만 두 커뮤니티의 교류는 오랫동안 제한적이었다. 이유는 여러 층에 걸쳐 있었다.

첫째는 센서 자체다. 카메라는 texture와 color를 보고, LiDAR는 range와 geometry를 본다. 카메라 기반 방법이 keypoint·descriptor·photometric consistency를 중심으로 발전할 때, LiDAR는 edge·plane·range image로 분화했다. 문제 공식 자체가 달랐다.

학회도 달랐다. CVPR·ICCV는 카메라 기반 방법의 주 발표 무대였고, ICRA·IROS·RSS는 LiDAR SLAM이 주로 나왔다. 연구자 집단의 겹침도 적었다. Velodyne이 구글과 자율주행 업계에 공급되던 2010년대 초중반에 LiDAR SLAM 연구자 집단은 자율주행 로봇공학 쪽에 밀집했다.

Place recognition 방법도 달랐다. 카메라는 DBoW2·NetVLAD처럼 visual appearance를 사용한다. LiDAR는 [Scan Context(Kim·Kim, 2018)](https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf)나 [PointNetVLAD](https://arxiv.org/abs/1804.03492) 같이 3D point cloud의 구조적 특징을 활용한다. 동일 장소라도 인식하는 신호 자체가 다르다.

수렴의 첫 신호는 2020년대 초에 나타났다. LiDAR-Camera 융합을 다루는 논문이 CVPR에 올라오기 시작했고, Tixiao Shan이 낸 [LVI-SAM (2021)](https://arxiv.org/abs/2104.10831)은 LIO-SAM에 visual-inertial 서브시스템을 붙인 시도였다. 저자들은 이를 tightly coupled factor-graph 시스템으로 규정했다. 다만 구현은 서로 정보를 주고받고 필요하면 독립 동작도 하는 visual-inertial subsystem(VIS)과 lidar-inertial subsystem(LIS)으로 나뉜다.

---

## 17.7 Visual-LiDAR 수렴 시도: 2024-2025

2024년을 기점으로 융합 시도가 늘었다. Foundation model이 센서와 무관하게 feature를 뽑는 방향으로 발전하면서, 카메라와 LiDAR를 하나의 프레임에서 처리하는 연구가 이어졌다. 갈래는 둘이다.

하나는 multi-modal pretrained feature다. LiDAR와 카메라를 같은 embedding space로 align한다. [CLIP(Radford et al., 2021)](https://arxiv.org/abs/2103.00020)이 image-text alignment를 해낸 것처럼, LiDAR-image contrastive learning을 사용하는 접근이다. 2023-2024년 여러 그룹에서 실험 단계다.

다른 하나는 unified sensor abstraction이다. 센서 출력을 geometric primitive나 neural field로 통합한 뒤 단일 backend에서 처리한다. 이쪽은 아직 연구 논문 단계이고 실시간 동작을 보인 시스템은 드물다.

어느 방향도 아직 LiDAR SLAM과 Visual SLAM을 실질적으로 통합한 단일 계보를 만들지 못했다. FAST-LIO2와 ORB-SLAM3는 여전히 독립적으로 쓰인다.

---

## 17.8 Radar는 본 책의 scope 밖이다

LiDAR 평행 우주 바로 옆에는 또 하나의 평행 우주가 있다. Radar SLAM은 spinning radar(Navtech CIR 계열)와 SoC 기반 4D mmWave radar라는 두 하드웨어 분기 위에, Doppler radial velocity를 직접 측정해 correspondence-free odometry가 가능하다는 점, 그리고 speckle·multipath·receiver saturation 같은 전파 고유의 noise 모델 위에서 독립 subfield로 성숙했다. [Cen & Newman 2018](https://doi.org/10.1109/ICRA.2018.8460687)의 Oxford 계열 radar localisation에서 출발해 Adolfsson·Magnusson의 **CFEAR**, 그 후속 **TBV-SLAM**, Burnett·Barfoot의 continuous-time ICP까지 계보가 이어졌고, Oxford Radar RobotCar·Boreas·MulRan 같은 전용 데이터셋이 이 영역의 벤치마크 기반을 이룬다. 악천후와 연기 관통성이라는 실용 동기는 분명하지만, 본 책이 추적해 온 photogrammetry → SfM → Visual SLAM → learning → 3D foundation의 계보와는 접점이 얇다. radar는 "앞으로 합류할 이웃"으로 남겨 두고, 이 책은 별도 역사를 쓰지 않는다. 상세는 *The SLAM Handbook*(2026) Ch.9 참조.

---

## 📜 예언 vs 실제

> Zhang·Singh는 2014년 LOAM 논문 Conclusion에서 다음 두 가지를 명시적 future work로 꼽았다. 첫째, loop closure를 도입해 drift를 보정하는 것. 둘째, IMU 출력을 Kalman filter로 자신들의 방법과 결합하는 것. 두 방향 모두 이후 10년 안에 실현됐다. IMU 결합은 FAST-LIO(2021)·FAST-LIO2(2022)가 iEKF로 tightly coupled 방식으로 정리했고, loop closure는 LIO-SAM(2020)이 factor graph backend로 통합했다. 저자들이 스케치한 경로는 꽤 정확히 구현됐다. 그러나 이 두 축 너머에는, 논문 Conclusion에는 등장하지 않았지만 실무 현장에서 꾸준히 부각된 과제가 있었다. dynamic object 처리다. LiDAR 포인트에서 움직이는 보행자·차량을 실시간 분리하는 작업은 2026년 현재도 주로 deep learning segmentation에 의존하고, SLAM 내부의 geometry 기반 방법도 있지만, 다양한 동적 환경을 포괄하는 일반 해법은 정착되지 않았다.

---

## 🧭 아직 열린 것

**Visual+LiDAR 완전 융합.** LVI-SAM 이후에도 두 센서를 하나의 상태 추정기에서 다루는 tightly coupled 설계 가운데 널리 받아들여진 공통형은 없다. 안개·강우에서 카메라가 약해질 때 LiDAR가 빈자리를 채워야 하는 시나리오는 자율주행에서 명확한 요구다. 알고리즘과 센서 캘리브레이션의 난이도가 장벽이고, 2024-2025년의 transformer 기반 융합도 아직 연구 시제품 단계다.

**Solid-state LiDAR에 최적화된 알고리즘.** 원래 LOAM은 360° spinning LiDAR의 scan line과 특징 구조를 전제로 했다. 일부 Livox 제품의 비반복 스캔과 solid-state 센서의 제한된 FoV는 관측 가능성과 motion distortion의 양상을 바꾼다. FAST-LIO2의 direct point-to-map 방식과 Livox LOAM처럼 이를 다루는 시스템은 이미 있지만, 서로 다른 FoV와 scan pattern을 한 설정으로 포괄하는 방법은 아직 확립되지 않았다.

**동적 물체 처리.** 이 문제는 LOAM의 두 가지 명시적 future work와는 별도로 남은 과제다. 정적 환경 가정은 SLAM의 오래된 전제이고, LiDAR도 예외가 없다. 움직이는 물체를 포인트 클라우드에서 실시간 분리하는 작업은 흔히 segmentation network에 맡긴다. SLAM 내부의 geometry 기반 방법은 연산 비용과 안정성 문제를 함께 안고 있다. 실제 제품의 처리 계통은 대체로 비공개이며, 공개 연구에서도 하나의 일반 해법이 널리 받아들여지지는 않았다.

---

LiDAR 계보는 Visual 주축과 접점이 적은 채로 성숙했다. 두 계보는 각자의 언어를 갖추었고, 그 언어들 사이의 번역은 아직 진행 중이다.
