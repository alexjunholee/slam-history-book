# 21장 LiDAR SLAM: LOAM에서 FAST-LIO까지

사진측량에서 출발해 Foundation 3D에 이르기까지, 앞선 장들에서 살펴본 계보는 하나같이 카메라 영상을 입력으로 받았다. MonoSLAM, PTAM, ORB-SLAM, DSO, DUSt3R가 모두 그 줄기에 서 있다. 그런데 바로 같은 시기, 레이저를 쏘아 거리를 직접 재는 LiDAR 연구자들은 전혀 다른 뿌리 위에서 독자적인 방법론을 쌓아 올리고 있었다. [ICP (Besl·McKay, 1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)와 Lu–Milios의 pose network가 그 뿌리였다. 두 세계는 서로의 논문을 거의 인용하지 않았고, 쓰는 벤치마크도, 모이는 학회도 달랐다.

2014년 Ji Zhang이 RSS에서 LOAM을 발표하던 바로 그해, 카메라 진영에서는 LSD-SLAM이 나왔고 이듬해에는 ElasticFusion이 등장했다. 그러나 LOAM은 그 방법들과 거의 교차하지 않은 채 태어났으며, 이후 10년 동안에도 두 분야가 주고받은 기술적 유산은 의외로 많지 않았다.

심지어 레이저 너머에서는 또 하나의 독립 분야인 **radar SLAM**이 따로 자라났다. 하드웨어는 크게 회전식 기계 스캐너인 Navtech 계열과, 칩 하나에 송수신부를 집적한 4D mmWave radar로 갈린다. 특히 4D mmWave radar는 Doppler 효과를 이용해 각 반사점이 센서를 향해 다가오거나 멀어지는 시선 방향 속도(radial velocity)를 직접 재기 때문에, 스캔 사이의 대응점을 찾지 않고도 센서의 자기 움직임을 바로 뽑아낼 수 있다. 물론 어떤 radar를 쓰든 speckle, multipath, receiver saturation처럼 전파 고유의 노이즈를 따로 다루어야 한다. 이 계보는 Oxford의 [Cen & Newman 2018](https://doi.org/10.1109/ICRA.2018.8460687)에서 물꼬를 튼 뒤 Adolfsson·Magnusson의 CFEAR와 후속 TBV-SLAM, Burnett·Barfoot의 연속시간 기법으로 이어졌고, Oxford Radar RobotCar, Boreas, MulRan 같은 전용 데이터셋에서 비·안개·연기를 뚫는 전천후 내비게이션을 입증해 왔다(상세한 역사는 *SLAM Handbook*, Cambridge University Press, 2026년 4월 21일자 원고 9장 "Radar SLAM" 참조). 이 장에서는 그 가운데 주류 거리 센서로 자리 잡은 LiDAR SLAM의 계보를 LOAM에서 FAST-LIO까지 따라가 본다.

---

## 21.1 LOAM: edge와 plane으로 실시간 3D 정합

LOAM 이전에도 레이저 스캔을 엮어 지도를 그리는 고전은 이미 있었다. [Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping" (Autonomous Robots)](https://doi.org/10.1023/A:1008854305733)는 궤적 전체를 한꺼번에 최적화하는 "network of poses"를 세웠다. 2D 레인지 스캔 하나하나를 노드로 삼고 스캔 사이의 상대 변환 제약을 edge로 묶은 이 구조가 훗날 포즈 그래프 SLAM의 원형이 됐다(→ 6장). 스캔을 맞대는 방법 역시 ICP 외에 [Biber·Straßer 2003. "The Normal Distributions Transform" (IROS)](https://doi.org/10.1109/IROS.2003.1249285)의 NDT가 있었다. NDT는 공간을 격자 셀로 쪼개 각 셀의 점군을 하나의 가우시안 분포로 요약한 뒤 새 스캔을 그 확률 분포에 맞추는 기법으로, 뒤에 Magnusson이 이를 3D로 확장했다.

하지만 2D에서 3D로 차원을 높이자 난제가 한꺼번에 몰려왔다. 회전하는 3D 레이저 스캐너는 한 바퀴를 도는 동안 포인트마다 찍힌 시각이 다르다. 센서가 움직이는 중에 그 이동량을 잘못 추정하면 포인트 클라우드가 뒤틀린 채 쌓이고 만다. LOAM 논문에서는 당시까지 일관된 3D 지도를 얻으려면 오프라인 일괄 처리에 의존해야 했고, 누적 오차(drift) 역시 대개 loop closure에 기대어 사후에 바로잡았다고 적는다. LOAM이 내건 목표는 loop closure 없이도 그에 버금가는 정확도를 실시간으로 뽑아내는 것이었다.

당시 3D LiDAR는 연구실에 두기엔 너무 비싼 장비였다(가격 변천은 → §21.5). 정작 LOAM 저자들조차 자체 실험에는 3D LiDAR를 쓰지 못하고, Hokuyo UTM-30LX 2D 레이저 스캐너를 모터로 끄덕이게 만든 자작 2축 LiDAR를 동원했으며 Velodyne 데이터는 KITTI 벤치마크로만 검증했다.

Carnegie Mellon 대학 Robotics Institute에서 Sanjiv Singh의 지도를 받은 Ji Zhang은 [Zhang & Singh 2014. "LOAM: Lidar Odometry and Mapping in Real-time" (RSS)](https://www.roboticsproceedings.org/rss10/p07.pdf)에서 계산량을 획기적으로 덜어 낼 기준을 찾았다. 모든 포인트를 다 맞추는 대신 포인트마다 곡률(smoothness) $c$를 계산해 두 종류의 기하 특징만 추려 낸 것이다. $c$는 같은 scan line 위에서 양옆 이웃 점들과의 위치 차이를 더해 구하는데, 이 값이 가장 큰 뾰족한 점은 **edge point**로, 가장 작은 매끈한 점은 **planar point**로 분류한다. 매칭에서는 edge point에 직전 sweep에서 찾은 edge 선분까지의 거리 제약을, planar point에 직전 sweep의 평면 패치까지의 거리 제약을 각각 걸었다. 점군 전체 대신 소수의 특징점만 건드리니 실시간 연산의 문이 열렸다.

LOAM의 또 다른 승부수는 주파수가 다른 두 루프의 분업이었다. **Lidar Odometry** 스레드는 약 10 Hz로 돌며 연속 스캔 사이의 6-DoF 움직임을 빠르게 거칠게 추정하고 그 결과로 스캔의 왜곡을 편다. 그 뒤를 **Lidar Mapping** 스레드가 1 Hz의 느린 호흡으로 따라붙어, 왜곡이 보정된 스캔을 누적 지도에 정밀하게 정합한다. 두 결과를 합치면 실시간 10 Hz 위치 출력이 나오면서도 오차가 천천히 쌓인다. 이 2단 주파수 분할 아키텍처는 훗날 LeGO-LOAM이 그대로 물려받았다.

2026년 10월 현재까지도 KITTI odometry 공개 순위표에 올라 있는 LOAM의 평균 translation error는 단 0.55%다.

> 🔗 **차용.** LOAM의 점군 정합은 Besl·McKay(1992)의 ICP 계보에서 출발하지만, 점 대 점 거리 대신 기하 특징(edge 선분과 planar 평면)까지의 거리를 최소화하는 변형으로 실시간성과 정확도를 동시에 챙겼다.

---

## 21.2 LeGO-LOAM: 지면을 먼저 분리하다

원판 LOAM은 도로 바닥을 찍은 지면 포인트와 건물·나무 포인트를 가리지 않고 한데 섞어 최적화했다. Stevens Institute of Technology의 Robust Field Autonomy Lab에서 Tixiao Shan과 지도교수 Brendan Englot은 [Shan & Englot 2018. LeGO-LOAM](https://doi.org/10.1109/IROS.2018.8594299)을 내놓으며 지면 분리를 파이프라인 맨 앞자리에 세웠다.

LeGO-LOAM은 먼저 3D 포인트 클라우드를 픽셀마다 거리를 담은 2D 영상인 range image로 펼친 뒤 지면 포인트를 떼어 내고, 남은 점들을 의미 있는 덩어리로 묶어(segmentation) 잡음을 걸러 낸다. 최적화도 두 단계로 쪼갰다. 먼저 지면에서 뽑은 planar feature만으로 높이($t_z$)와 roll·pitch 자세를 풀고, 이어서 수직 물체의 edge feature로 남은 평면 이동($t_x$, $t_y$)과 yaw 각도를 구한다. 두 단계 모두 Levenberg-Marquardt 최적화를 쓴다.

지상 차량으로 울퉁불퉁한 야지를 달린 실험에서 LeGO-LOAM은 LOAM보다 훨씬 적은 연산으로 비슷하거나 더 나은 정확도를 냈고, VLP-16 데이터를 저전력 임베디드 보드인 NVIDIA Jetson TX2 위에서 실시간으로 소화했다. 다만 [공개 구현 README](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)에서 밝히듯 수평 장착된 VLP-16을 기준으로 조율되어 있으며, 매 스캔에 지면이 늘 보인다는 전제로 작동한다.

바로 같은 해 Bonn 대학의 Jens Behley와 Cyrill Stachniss는 실외 LiDAR 지도의 그릇을 달리하는 실험을 했다. [Behley & Stachniss 2018. "Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments" (RSS)](http://www.roboticsproceedings.org/rss14/p16.pdf)에서 제안된 SuMa는 3D 지도를 **surfel**(작은 원반 모양의 표면 조각) 집합으로 쌓고, 새 스캔을 그 surfel 모델에 직접 맞췄다(scan-to-model). 실내 RGB-D 센서에서 ElasticFusion이 쓰던 surfel 표현을 도심 Velodyne 데이터로 옮겨 심은 것이다. 이듬해 나온 후속작 [Chen et al. 2019. "SuMa++" (IROS)](https://doi.org/10.1109/IROS40897.2019.8967704)는 surfel 위에 semantic segmentation 라벨을 붙여 달리는 자동차 같은 동적 점들을 걸러 냈다. 이로써 2018년 무렵 LiDAR SLAM은 LOAM식 특징점 방식, LeGO-LOAM식 지면 분리 방식, SuMa식 surfel 지도 방식이 각축을 벌이는 구도를 이뤘다.

---

## 21.3 FAST-LIO: tightly coupled LiDAR-IMU

회전식 LiDAR는 보통 1초에 10바퀴쯤 돈다. 한 바퀴를 도는 0.1초 동안에도 로봇이나 드론이 격렬하게 방향을 틀면 스캔 시작점과 끝점에서 센서 위치가 크게 달라져 점군이 일그러지는데, 이를 **motion distortion**이라 부른다. LOAM은 0.1초 동안 등속 운동을 한다고 가정해 왜곡을 폈지만, 급격한 기동 앞에서는 그 가정이 곧장 무너졌다.

이 틈새를 메우는 센서가 수백 Hz로 가속도와 각속도를 뱉어 내는 IMU다. 두 센서를 엮는 방식은 센서마다 따로 푼 결과를 나중에 합치는 **loosely coupled**와, 단일 추정기 안에서 두 센서의 원시 관측을 한꺼번에 다루는 **tightly coupled**로 나뉜다. tightly coupled 구조가 두 센서 사이의 물리적 상관관계를 더 촘촘하게 활용하지만, 실제 필드 성능은 센서 모델링과 캘리브레이션, 수치 구현의 완성도에 좌우된다.

홍콩대(HKU) MaRS Lab의 Wei Xu와 지도교수 Fu Zhang은 2021년 RA-L에 발표한 [FAST-LIO](https://arxiv.org/abs/2010.08196)에서 tightly coupled 방식을 극도로 가볍게 만드는 수학적 트릭을 선보였다. FAST-LIO는 iterated Extended Kalman Filter(iEKF)로 LiDAR 특징점과 IMU 측정을 하나로 묶었다. 이때 측정 점 개수가 수천 개로 불어나도 연산이 폭증하지 않도록 Kalman gain 공식을 새롭게 유도해, 계산 복잡도가 측정 개수가 아닌 **상태 차원**에만 비례하도록 뒤집었다. 덕분에 쿼드로터 탑재 컴퓨터에서도 한 스캔의 유효 특징점 1,200여 개를 통합하면서 iEKF 반복 전체를 25 ms 안에 마쳤다. iEKF는 매 측정 갱신마다 새로운 추정치 주변에서 선형화를 반복하는 구조여서, 한 번만 선형화하고 끝내는 일반 EKF의 선형화 오차도 크게 줄었다.

이듬해 TRO에 실린 후속작 FAST-LIO2([Xu et al. 2022](https://doi.org/10.1109/TRO.2022.3141876))는 한 걸음 더 나아가 LOAM 이래의 관습이던 특징점 추출 단계마저 없애 버리고 원시 포인트를 곧바로 지도에 정합했다. 손으로 깎은 특징점 규칙을 버리자 회전식이든 비반복 스캔 방식이든 가리지 않고 어떤 LiDAR에나 그대로 꽂아 쓸 수 있게 됐다. 지도는 **ikd-Tree**(incremental kd-Tree)에 담았는데, 포인트를 넣고 지울 때 트리 전체를 다시 짓지 않고 필요할 때만 국소 균형을 맞춘다. 저자들은 UAV, 핸드헬드 장치, 자동차를 아우르는 공개 데이터셋 19개 시퀀스에서 기존 방법들을 비교하며 넓은 실외 환경에서도 odometry와 mapping을 최대 100 Hz로 돌렸다고 보고했다.

같은 연구실의 [He et al. 2023. "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry" (Advanced Intelligent Systems)](https://doi.org/10.1002/aisy.202200459)는 아예 스캔 한 바퀴가 모이기를 기다리지 않는 극단으로 갔다. LiDAR 포인트가 하나 날아올 때마다 그 포인트가 찍힌 정확한 시각에 상태를 즉시 갱신하므로, 스캔 내부의 motion distortion이라는 문제 자체가 원천적으로 사라진다. 그 결과 odometry 출력 대역폭은 4~8 kHz까지 치솟았고, IMU 측정값을 운동 모델의 출력으로 취급하는 정식화 덕분에 IMU가 측정 한계를 넘어 포화된 상태에서도 초당 75 rad로 회전하는 기동을 놓치지 않고 추적했다.

> 🔗 **구분.** FAST-LIO는 원시 IMU 측정으로 상태를 앞으로 전파(forward propagation)하고, 스캔 안의 각 포인트를 시간 역순으로 되돌려(backward propagation) 왜곡을 편 뒤 iEKF로 한꺼번에 갱신한다. 반면 factor graph 진영에서 쓰는 방식은 [Forster et al. 2017. "On-Manifold Preintegration" (TRO)](https://doi.org/10.1109/TRO.2016.2597321)의 IMU preintegration factor이며, 바로 뒤에 볼 LIO-SAM이 이 방식을 택했다.

---

## 21.4 LIO-SAM: LiDAR와 IMU를 factor graph로 묶다

같은 시기 visual-inertial SLAM에서는 이미 factor graph가 표준 언어로 자리 잡고 있었고, LiDAR 쪽 역시 Lu–Milios의 pose network로 거슬러 올라가는 그래프 최적화의 기억을 갖고 있었다. Georgia Tech Frank Dellaert 연구실이 만든 [GTSAM](https://gtsam.org/)은 출처가 다른 센서 제약들을 모두 factor라는 공통 문법으로 엮어 주는 라이브러리였다.

LeGO-LOAM을 냈던 Tixiao Shan은 후속작 [Shan et al. 2020. LIO-SAM](https://doi.org/10.1109/IROS45743.2020.9341176)(IROS)에서 LiDAR-IMU 결합을 GTSAM의 factor graph 위로 올렸다. 그래프 구조의 최대 장점은 확장성이다. IMU preintegration, LiDAR odometry, GPS, loop closure 제약을 출처 구분 없이 한 그래프에 나란히 꽂아 넣을 수 있었다. IMU preintegration이 추정한 움직임은 스캔의 왜곡을 펴는 동시에 LiDAR odometry 최적화의 초기값을 대 준다. 실시간성을 지키기 위해 새 스캔을 지도 전체가 아니라 최근 keyframe 몇 장으로 엮은 국소 지도에만 정합한다. [공개 구현](https://github.com/TixiaoShan/LIO-SAM)에서는 실행 내내 유지되는 주 factor graph가 LiDAR odometry factor와 GPS factor를 풀고, 별도의 factor graph가 IMU와 LiDAR odometry factor로 IMU bias를 추정하면서 IMU 주기에 맞춰 위치를 내보낼 수 있게 주기적으로 초기화된다. README에서는 내장된 loop closure 기능이 LeGO-LOAM에서 가져온 ICP 기반 예제일 뿐이라며, 실전에서 쓰려면 Scan Context 같은 전용 장소 인식을 붙일 것을 권한다.

복잡해지는 파이프라인 속에서 의외의 역습도 있었다. 2023년 Bonn 대학의 [Vizzo et al. 2023. "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L)](https://doi.org/10.1109/LRA.2023.3236571)는 "Keep It Small and Simple"이라는 이름 그대로 특징점 추출도, IMU 결합도, 복잡한 그래프도 다 덜어 냈다. 남긴 것은 고전적인 point-to-point ICP, 주행 속도와 상황에 맞춰 대응점 거리 기준을 스스로 조절하는 adaptive threshold, robust kernel, 단순한 등속 motion compensation, 그리고 균일한 point cloud subsampling뿐이었다. 그럼에도 파라미터 손질 하나 없이 자동차, UAV, segway, 핸드헬드 LiDAR 데이터 전반에서 당대 최신 기법과 맞먹는 정확도를 내면서, 3D LiDAR odometry의 기본기가 얼마나 탄탄한지를 다시 일깨웠다.

---

## 21.5 센서 가격 하락과 보급: 2007–2024

알고리즘이 연구실 밖으로 쏟아져 나온 배경에는 극적인 하드웨어 가격 붕괴가 있었다. 2007년 DARPA Urban Challenge에서 우승권 팀들의 차 지붕에 올라간 Velodyne HDL-64E는 한 대에 무려 75,000달러였다. 2011년 IEEE Spectrum이 조명한 Google의 자율주행 Prius 지붕에도 여전히 그 64채널 Velodyne이 얹혀 있었다([IEEE Spectrum 2011](https://spectrum.ieee.org/how-google-self-driving-car-works)). 당시 3D LiDAR는 국방 과제나 거대 기업 프로젝트의 전유물이었다. 2014년 9월 Velodyne이 16채널 VLP-16을 [7,999달러에 출시하며](https://www.prweb.com/releases/breaking_through_the_price_barrier_for_lidar_sensors_velodyne_introduces_3d_real_time_lidar_puck_at_7_999/prweb12139477.htm) 한 차례 문턱을 낮췄지만, 여전히 일반 대학 연구실이 여러 대를 사들이기엔 부담스러운 값이었다.

판이 완전히 뒤집힌 해는 2019년이었다. DJI에서 분사한 중국 업체 Livox가 그해 [Mid-40의 미국 소매가를 단돈 599달러로 발표](https://www.livoxtech.com/news/1)하면서 LiDAR가 드디어 카메라에 근접한 가격대로 내려왔다. 같은 해 1월 Ouster는 128채널 OS1-128을 [18,000달러에 선보였고](https://ouster.com/insights/blog/introducing-the-os-1-128-lidar-sensor), 2020년 10월에는 solid-state 센서 ES2를 공개하며 [2024년 차량 양산 기준 예상 가격을 600달러로 제시](https://investors.ouster.com/news-releases/news-release-details/ouster-announces-first-high-performance-true-solid-state-digital)했다. 물론 채널 수와 시야각, 측정 거리가 제각각이고 소매가와 양산 목표가의 기준도 다르지만, 10여 년 사이에 두 자릿수 배율로 값이 떨어진 흐름만큼은 분명했다.

센서가 싸지고 물리적 구조가 다양해지자 알고리즘도 변해야 했다. 새로 보급된 solid-state LiDAR는 기존 회전식보다 시야각(FoV)이 좁고, Livox 일부 제품은 같은 궤적을 되풀이하지 않는 꽃잎 모양의 비반복 스캔 패턴을 썼다. 360° 회전하며 고르게 찍히는 scan line 구조를 전제로 설계된 원판 LOAM은 이런 센서 앞에서 맥을 추지 못했다. 반면 특징점 가정을 버린 FAST-LIO와 FAST-LIO2는 회전식과 solid-state LiDAR를 모두 품었고, 시야각이 좁은 solid-state 센서에서도 안정적인 실측 성능을 입증했다.

---

## 21.6 Visual SLAM과 LiDAR SLAM이 따로 발전한 이유

카메라 SLAM과 LiDAR SLAM이 10년 넘게 평행선을 달린 까닭은 두 센서가 세상을 재는 물리 법칙부터 달랐기 때문이다. 카메라는 빛의 밝기와 색을 조밀한 격자에 담아내므로 keypoint, 서술자, photometric consistency(같은 물체 표면은 시점이 달라져도 밝기가 비슷하다는 가정)를 중심으로 기법이 자랐다. 반면 LiDAR는 3D 거리를 직접 찍어 형상을 얻는 대신 텍스처 정보가 없으니 edge, plane, range image를 붙잡았다. 다시 찾은 장소를 알아보는 place recognition에서조차 카메라는 DBoW2나 NetVLAD처럼 겉모습(appearance)을 비교하고, LiDAR는 [Scan Context(Kim·Kim, IROS 2018)](https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf)나 [PointNetVLAD](https://arxiv.org/abs/1804.03492)처럼 점군의 3D 기하 분포를 훑었다.

공동체의 무대마저 갈라져 있었다. LOAM이 RSS, LeGO-LOAM과 LIO-SAM이 IROS, FAST-LIO가 RA-L, FAST-LIO2가 TRO에 실리는 동안, 카메라 계열의 이정표는 PTAM이 ISMAR, DTAM이 ICCV, LSD-SLAM이 ECCV, DUSt3R가 CVPR에 나오는 등 컴퓨터 비전 학회에서 더 자주 터져 나왔다.

두 줄기가 본격적으로 손을 맞잡기 시작한 것은 2020년대 초였다. Tixiao Shan은 LIO-SAM을 낸 지 1년 만에 [LVI-SAM (ICRA 2021)](https://arxiv.org/abs/2104.10831)을 발표해 하나의 factor graph 안에서 visual-inertial system(VIS)과 lidar-inertial system(LIS)을 단단히 엮었다. VIS는 LIS의 추정치로 초기화하고 LiDAR 점군에서 시각 특징점의 정확한 깊이를 빌려 오며, 반대로 LIS는 VIS의 추정을 scan matching의 초기값으로 쓴다. 둘이 서로를 떠받치되, 어두운 터널이나 특징 없는 복도에서 한쪽이 쓰러지면 살아남은 다른 쪽이 단독으로라도 주행을 이어 가도록 설계했다.

---

## 21.7 Visual-LiDAR 수렴 시도: 2024-2025

2024–2025년에 접어들며 센서 종류를 가리지 않고 공통 표현을 배우는 foundation model이 부상하자, 카메라와 LiDAR를 한 지붕 아래 통합하려는 시도 역시 새 국면을 맞았다. 한쪽에서는 LiDAR 점군과 카메라 영상을 하나의 특징 공간에 겹쳐 놓는 multi-modal 사전학습이 이어지고 있다. [CLIP(Radford et al., 2021)](https://arxiv.org/abs/2103.00020)이 이미지와 텍스트를 같은 embedding space에 정렬할 때 쓴 contrastive learning 문법을 그대로 빌려온 것이다.

다른 한쪽에서는 각 센서의 원시 출력을 geometric primitive나 neural field 같은 공통 3D 표현으로 먼저 번역한 뒤 단일 backend에서 함께 최적화하려는 시도가 자라고 있다. 그럼에도 현장의 주력 시스템들은 여전히 한 가지 주된 센서를 중심축으로 삼고 나머지 센서를 보조로 붙이는 구조에 머물러 있다.

---

## 예측과 실제

> Zhang·Singh는 2014년 LOAM 논문의 결론에서 loop closure로 drift를 바로잡는 일과, 자신들의 출력을 IMU와 함께 Kalman filter로 합쳐 drift를 더 줄이는 일을 future work로 적었다. 이 일은 10년 안에 이루어졌다. loop closure는 LIO-SAM(2020)이 factor graph에 넣었고, IMU 결합은 FAST-LIO(2021)와 FAST-LIO2(2022)가 iEKF 기반 tightly coupled 방식으로 풀었다.

---

## 아직 열린 문제

**Visual+LiDAR 완전 융합.** LVI-SAM 이후로도 카메라와 LiDAR를 단일 상태 추정기 안에서 tightly coupled로 엮는 방식은 연구실마다 설계가 제각각이다. 두 센서를 함께 굴리려면 시공간 외부 캘리브레이션이라는 까다로운 전제가 따라붙고, 한 추정기 안에서 두 모달리티를 가장 안정적으로 다루는 표준 아키텍처는 아직 합의되지 않았다.

**Solid-state LiDAR에 맞춘 알고리즘.** Livox 일부 모델의 비반복 스캔이나 solid-state 센서의 좁은 시야각에서는 어떤 방향의 운동을 관측할 수 있는지(observability)와 motion distortion의 형태가 기존 회전식과 전혀 다르게 나타난다. FAST-LIO2처럼 원시 포인트를 지도에 곧장 맞추거나 Livox 전용 구현([loam_livox](https://github.com/hku-mars/loam_livox))을 쓰는 대안은 나왔지만, 센서의 시야각과 스캔 패턴에 구애받지 않고 단일 설정으로 두루 통하는 일반 해법은 아직 남아 있다.

**동적 물체 처리.** 2014년 LOAM 논문이 future work 목록에 올리지도 않았던 문제지만, 환경이 정지해 있다는 오래된 가정은 LiDAR SLAM 역시 피해 가지 못한다. SuMa++처럼 semantic segmentation 라벨로 움직이는 점을 걷어 내는 흐름과, ERASOR·Dynablox처럼 순수 기하적 가시성 추론으로 동적 점을 골라내는 흐름이 나란히 다듬어지고 있다.

---

카메라와 LiDAR가 각자의 길에서 성숙한 주류 시스템을 빚어내는 동안, 그 바깥에서는 전혀 다른 발상에서 출발해 반짝였으나 끝내 중심 무대에 오르지 못했거나 완전히 다른 모습으로 변모해 버린 시도들도 있었다. 다음 장에서는 바로 그 "사라진 방법들"의 흔적을 짚어 본다.
