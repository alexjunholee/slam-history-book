# Ch.18 — 실패 사례와 사라진 계보

카메라 기반 SLAM과 LiDAR 기반 SLAM이 자리를 잡아 가는 동안, 로보틱스 연구자 가운데 일부는 두 방식 어느 쪽에도 속하지 않는 방법을 시도했다. 생체 모방 지도, event camera, 사물 단위 semantic 지도, 구조적 prior를 쓰는 방법이었다.

논문과 초기 결과를 충분히 내고도 주류가 되지 못한 방법들이다.

---

## 18.1 RatSLAM — place cell 기반 위상 지도

RatSLAM은 Michael Milford, Gordon Wyeth, David Prasser가 2004년 ICRA에서 발표했다([Milford et al. 2004](https://doi.org/10.1109/ROBOT.2004.1307183)). 세 사람은 University of Queensland에 있었다. RatSLAM은 쥐의 해마에 있는 **place cell**과 **head direction cell**을 본뜬 모델로 로봇의 위치를 추정했다. 중심은 **pose cell**이라 부르는 신경망이다. pose cell은 (x, y, θ) 세 축으로 배열된 competitive attractor network(경쟁 끌개 신경망)이고, 활성이 한곳에 모인 덩어리(activity packet)가 로봇의 위치와 방향을 나타낸다. 바퀴 엔코더로 잰 이동량(path integration)에 따라 이 덩어리가 옮겨 간다. 로봇이 전에 본 장면(local view)을 다시 보면, 그 장면과 연결된 pose cell에 활성이 더해져 덩어리의 위치가 바로잡힌다.

> 🔗 **차용.** [O'Keefe와 Dostrovsky(1971)](https://pubmed.ncbi.nlm.nih.gov/5124915/)가 발견한 place cell은 신경과학 연구에서 인지 지도 이론으로 이어졌다. RatSLAM은 이 생물학적 기제를 공학 시스템으로 옮긴 초기 시도였다.

2008년 IEEE TRO 논문 [Milford & Wyeth 2008](https://doi.org/10.1109/TRO.2008.2004520)에서는 RatSLAM을 실외에서 시험했다. 차 지붕에 노트북 내장 웹캠 하나를 달고 브리즈번 St Lucia 교외를 100분 동안 66 km 달렸다. RatSLAM은 10 Hz로 들어오는 영상을 실시간으로 처리했고, 길이가 최대 5 km인 루프를 51개 넘게 닫았다. 결과로 나온 지도는 semi-metric topological map(반계량 위상 지도)이었다. 논문은 이 지도가 덮은 면적이 다른 연구진의 지도보다 다섯 배 넘게 넓다고 보고했다. 그 연구진은 자기 지도가 2007년까지 레이저 센서 하나로 만든 지도 가운데 가장 크다고 밝혔다.

RatSLAM을 공학적으로 더 키운 연구는 있었지만 SLAM의 주류가 되지 못했다. 장소가 늘수록 계산량이 커졌고, 정밀도가 더 큰 문제였다. RatSLAM의 위상 지도(topological map)로는 "여기 와 본 적이 있다"는 것까지 알 수 있었지만, 미터 단위의 metric 위치는 안정적으로 내지 못했다. 자율주행과 조작(manipulation)에는 정확한 좌표가 필요했다.

> 📜 **예언 vs 실제.** Milford와 Wyeth는 2008년 TRO 논문 결론에서 RatSLAM을 "vision-only SLAM의 대안적 접근"이라고 불렀다. 기존 state-of-the-art SLAM이 다루기 어려운 환경을 지도로 만들었고, odometry 오차가 크고 환경이 모호해도 큰 루프를 반복해서 믿을 만하게 닫았다고 주장했다. 이 주장은 부분적으로 맞았다. 위상 지도는 지금도 일부 place recognition 연구에 나온다.

RatSLAM은 기하 없이도 장소를 표현할 수 있다는 생각을 남겼다. 이 생각은 place recognition 연구로 퍼졌다. 2012년 Milford와 Wyeth가 [SeqSLAM](https://doi.org/10.1109/ICRA.2012.6224623)을 발표했다. 이미지 하나 대신 이미지 시퀀스를 비교해 장소를 알아보는 방식은 visual place recognition 연구의 한 갈래가 됐다.

---

## 18.2 biologically-inspired SLAM의 공학적 한계

RatSLAM은 biologically-inspired SLAM(생물 모방 SLAM) 가운데 가장 완성된 사례였다. 2000년대 중반부터 2010년대 초반까지 인지 지도, entorhinal grid cell, hippocampal replay를 본뜬 SLAM 변형도 꾸준히 나왔다. 이들 모두 비슷한 문제를 안고 있었다.

생물학적 모델은 뇌가 공간을 *어떻게* 표현하는지 설명한다. 뇌가 *왜* 그렇게 표현하는지, 그 방식이 공학적 목적에도 맞는지는 별개의 질문이다. 쥐의 해마가 일하는 조건과 로봇을 평가하는 센서·계산·정확도 조건은 서로 다르다.

공학적 SLAM에는 미터 이하의 위치 정확도, 실시간 처리, 새 환경에 빨리 적응하는 능력, 검증할 수 있는 오차 범위가 필요하다. 인지 모델로는 이 조건을 보장하기 어려웠다.

---

## 18.3 Event SLAM — 하드웨어와 알고리즘 성숙 격차

Patrick Lichtsteiner, Christoph Posch, Tobi Delbruck는 [ISSCC 2006 논문](https://tilde.ini.uzh.ch/users/tobi/public_html/papers/lichtsteiner_ISSCC2006_D27_09.pdf)에서 Dynamic Vision Sensor(DVS)를 공개했고, [JSSC 2008 논문](https://doi.org/10.1109/JSSC.2007.914337)으로 확장했다. 논문에 적힌 소속은 Lichtsteiner와 Delbruck가 ETH Zürich(취리히의 Institute of Neuroinformatics, INI), Posch가 빈의 ARC Seibersdorf Research다. DVS의 각 픽셀은 밝기의 로그값이 얼마나 바뀌었는지를 혼자서 임계값과 비교한다. 바뀐 양이 임계값을 넘으면 밝아졌는지(ON) 어두워졌는지(OFF)를 담은 이벤트를 다른 픽셀과 상관없이 비동기(asynchronous)로 내보낸다. 셔터로 모든 픽셀을 한꺼번에 읽지 않고, 픽셀마다 이벤트가 난 시각을 마이크로초 단위로 기록한다. 프레임이 없는 카메라였다.

> 🔗 **차용.** DVS는 생물의 망막이 변화를 감지하는 방식에서 착안한 센서였다. Event SLAM은 이 센서에서 출발했다.

이벤트 카메라의 장점은 분명했다. 시간 해상도가 마이크로초 단위이고, 빠르게 움직여도 영상이 번지지 않는다. 동적 범위(dynamic range)가 넓어(HDR) 터널 안과 직사광선 아래를 모두 다룰 수 있다. 전력도 적게 쓴다. [Gallego 등의 서베이](https://arxiv.org/abs/1904.08405)에 따르면 칩 수준에서 대부분 약 10 mW를 쓴다.

2014년 ICRA에서 [Weikersdorfer et al. 2014](https://doi.org/10.1109/ICRA.2014.6906882)가 DVS에 depth 센서를 더한 event 기반 3D SLAM을 발표했다. 같은 해 다른 그룹들도 event 기반 optical flow와 depth 추정을 내놓았다. 2016년부터 2018년 사이 University of Zurich의 Davide Scaramuzza 연구실(RPG)에서 Henri Rebecq 등이 [EVO](https://doi.org/10.1109/LRA.2016.2645143)(RA-L 2017)와 [ESIM](https://proceedings.mlr.press/v87/rebecq18a.html)(CoRL 2018)을 발표하면서 event SLAM 파이프라인이 구체적인 모양을 갖췄다.

실제 환경에서는 제약이 컸다. 해상도가 문제였다. JSSC 2008 논문의 DVS는 128×128 픽셀로, VGA(640×480) 카메라의 약 19분의 1이었다. feature matching과 지도 작성이 해상도에 직접 좌우되는 SLAM에서 이 차이는 컸다. 알고리즘도 문제였다. 프레임을 전제로 만든 알고리즘을 이벤트 스트림에 맞게 다시 설계해야 했고, 그 작업에 시간이 걸렸다.

event를 쓰는 연구는 이후 장소 인식으로도 넓어졌다. [EventVLAD](https://ieeexplore.ieee.org/document/9635907/)(Lee & Kim, IROS 2021)는 event stream에서 잡음을 걷어 낸 edge 이미지를 복원하고, 이 이미지에서 NetVLAD descriptor를 뽑아 장소를 알아봤다. 조명이 크게 바뀌는 조건처럼 frame 기반 VPR(visual place recognition)이 약한 환경에 event를 써 본 시도였다.

---

## 18.4 Semantic SLAM — 사물을 landmark로 쓰는 방식

2017년부터 2019년까지 CVPR, ECCV, IROS에서 "semantic"을 다룬 연구가 빠르게 늘었다. 딥러닝으로 instance segmentation과 object detection 성능이 빠르게 오르던 때였다. 이런 semantic 이해를 SLAM에 넣으려는 연구도 이어졌다.

그보다 앞선 2013년 CVPR에서 Imperial College London의 Renato Salas-Moreno, Richard Newcombe, Hauke Strasdat, Paul Kelly, Andrew Davison이 **SLAM++**를 발표했다([Salas-Moreno et al. 2013](https://doi.org/10.1109/CVPR.2013.178)). SLAM++는 점이나 패치 대신 *사물(object)*을 지도의 기본 단위로 삼았다. 미리 만들어 둔 사물의 3D 모델을 데이터베이스에 넣어 두고, 손에 든 RGB-D 카메라 영상에서 그 사물을 실시간으로 인식했다. 인식에는 point pair feature로 투표하는 방법을 GPU에서 병렬로 돌렸고, 자세는 ICP(Iterative Closest Point)로 다듬었다. 인식한 사물은 카메라와 함께 pose graph에 넣어 계속 최적화했다. 실험에서는 의자 두 종류와 둥근 탁자 두 종류를 인식해 방 하나를 지도로 만들었다. 논문은 다른 방 하나의 지도 메모리를 같은 부피의 KinectFusion 지도와 견주어 약 70분의 1로 추산했다. 점 수천 개 대신 사물 수십 개로 지도를 나타내면 지도가 작아지고, 장소 인식과 loop closure도 사물을 단위로 할 수 있었다.

> 🔗 **차용.** SLAM++의 사물 인식은 컴퓨터 비전의 model-based recognition에서 왔다. 투표 방식은 Drost 등이 제안한 point pair feature 방법을 가져다 썼다.

SLAM++ 뒤로 [SemanticFusion](https://arxiv.org/abs/1609.05130)(McCormac et al., ICRA 2017)은 dense 지도에 semantic label을 붙였고, [MaskFusion](https://arxiv.org/abs/1804.09194)(Rünz et al., ISMAR 2018)은 움직이는 사물을 인식해 따로 추적했다. 같은 무렵 나온 [SuperPoint](https://arxiv.org/abs/1712.07629)(DeTone et al., CVPRW 2018)도 학습을 썼다. SuperPoint는 keypoint detector와 local descriptor를 함께 학습하는 방법이어서 점 단위로 동작한다. Semantic SLAM 연구자들은 semantic 이해를 기하 파이프라인에 넣으면 환경이 바뀌어도 지도가 더 강건해진다고 주장했다.

2019년까지 널리 비교되던 geometric 파이프라인은 ORB-SLAM2와 VINS-Mono였다. LIO-SAM은 2020년에 나왔다. 당시 semantic 시스템들은 저마다 다른 실내 환경, 사물, 데이터셋에서 평가돼 서로 직접 비교하기 어려웠다. 미리 정한 segmentation model과 class 목록에 기대야 한다는 한계는 여러 논문에 되풀이해 나왔다.

semantic을 먼저 세운 SLAM이 주류가 되지 못한 데는 의존성 문제가 있었다. Semantic SLAM은 segmentation이 정확해야 했다. segmentation이 틀리면 지도 전체가 오염됐다. 기하 파이프라인은 feature matching이 일부 실패해도 robust estimation으로 버텼다. 일반화도 문제였다. 특정 사물 class로 학습한 semantic prior는 그 class 밖에서 쓸모가 없었다.

> 📜 **예언 vs 실제.** Salas-Moreno 등은 SLAM++ 논문 결론에서 자신들의 방법을 "보다 일반적인(generic) SLAM 방법으로 가는 첫걸음"이라고 불렀다. 모양이 조금씩 다른 사물까지 다루고, 장기적으로는 시스템이 스스로 사물 class를 나누고 정의하게 되기를 기대했다. 초록에서는 사물 단위 표현으로 지도를 크게 압축할 수 있다고 했다. 이 기대는 일부만 맞았다. 사물 단위 지도는 AR과 일부 manipulation 응용에 쓰였다. 같은 사물이 되풀이되는 실내 환경에서는 압축과 효율 면의 이점이 다시 확인됐다. 주류 geometric SLAM은 2026년에도 sparse point와 keyframe graph를 쓴다. 시스템이 사물을 스스로 나누고 정의하는 단계에는 이르지 못했다. 사물을 landmark로 쓰는 방식은 널리 퍼지지 않았다. semantic은 SLAM 안에서 움직이는 영역을 가려내는 데, 그리고 SLAM 뒤의 semantic mapping과 task planning에 쓰이게 됐다.

semantic을 다른 자리에 쓴 연구는 계속 나왔다. [SuMa++](https://doi.org/10.1109/IROS40897.2019.8967704)(Chen et al., IROS 2019)는 LiDAR 스캔에 semantic class를 붙여 움직이는 물체를 걸러 냈다. [Kimera](https://doi.org/10.1109/ICRA40945.2020.9196885)(Rosinol et al., ICRA 2020)는 metric-semantic 3D mesh를 실시간으로 만들었고, 같은 그룹의 [3D Dynamic Scene Graphs](https://doi.org/10.15607/RSS.2020.XVI.079)(Rosinol et al., RSS 2020)가 3D scene graph를 내놓았다. [Hydra](https://doi.org/10.15607/RSS.2022.XVIII.050)(Hughes et al., RSS 2022)는 scene graph를 실시간으로, 계층을 나누어 만들었다. [ConceptGraphs](https://doi.org/10.1109/ICRA57147.2024.10610243)(Gu et al., ICRA 2024)와 [Clio](https://doi.org/10.1109/LRA.2024.3451395)(Maggio et al., RA-L 2024)는 그 위에 open-vocabulary foundation feature를 얹었다. semantic은 지도의 위층으로 자리를 옮겼다. 이 흐름은 2026년에도 이어지고 있고, [Ch.15b](chapter_15b_dynamic.md)(Dynamic·static 분리의 semantic 귀환), [Ch.16](chapter_16_foundation_3d.md)(foundation 3D·metric-semantic 본체), [Ch.19 §19.7](chapter_19_open_problems.md#197-semantic-표현과-open-world)(Semantic의 귀환)에서 이어 다룬다.

---

## 18.5 Manhattan-World 가정 — 좁아진 적용 범위와 보조 제약으로 남은 쓰임

Manhattan-world 가정을 쓴 SLAM도 같은 무렵 나왔다. 이 방식이 쓰이는 범위는 나중에 좁아졌다.

가정은 단순했다. [Coughlan & Yuille 1999](https://doi.org/10.1109/ICCV.1999.790349)의 Manhattan world 개념에 따라, 실내 구조는 대부분 세계 좌표계의 세 직교 축(x, y, z)에 맞춰 있다고 봤다. 벽, 바닥, 천장이 이 방향을 만든다. 이미지 속 평행선 묶음은 소실점(vanishing point)으로 모인다. 각 소실점은 카메라 회전 행렬 R과 방향 벡터 d로 `v = K R d`처럼 쓸 수 있다(K: 카메라 내부 행렬). 서로 직교하는 소실점 세 개를 찾으면 R의 세 열을 바로 구할 수 있다. IMU나 feature matching 없이 기하 제약만으로 회전의 drift를 누를 수 있다는 뜻이다. 이 아이디어를 visual odometry와 합치려는 시도가 이 무렵 나왔다.

긴 복도와 직사각형 방에서는 이 구조적 prior로 drift를 줄일 수 있었다. 그 밖의 환경이 문제였다. 야외로 나가거나, 둥근 구조물이 있거나, 구조가 불규칙한 산업 현장에 들어가면 Manhattan-world 가정 자체가 맞지 않았다. 그래서 이 가정은 맞는 환경에서만 골라 쓰는 보조 제약으로 남았다.

---

## 18.6 사라진 방법의 그 뒤

사라졌다는 말이 뜻하는 바는 사례마다 다르다. RatSLAM의 topological map 아이디어는 SeqSLAM으로 이어졌고, 그 뒤를 이은 방법들이 visual place recognition 분야에 남아 있다. SLAM++와 [LERF](https://arxiv.org/abs/2303.09553)(Kerr et al., ICCV 2023), [LangSplat](https://arxiv.org/abs/2312.16084)(Qin et al., CVPR 2024)은 모두 지도에 의미 정보를 붙인다. LERF와 LangSplat이 SLAM++를 직접 이었다고 볼 인용 근거는 없다. 서로 다른 표현에서 같은 목표가 나타난 경우로 보는 편이 정확하다.

Event camera SLAM은 다르게 흘러갔다. 처음에는 하드웨어 제약이 컸다. 2017년 ISSCC에서 Samsung이 640×480 해상도의 DVS를 발표했고([Son et al. 2017](https://doi.org/10.1109/ISSCC.2017.7870263)), 2021년에는 Sony와 Prophesee가 1280×720 해상도의 IMX636을 [발표](https://www.sony-semicon.com/en/news/2021/2021090901.html)했다. 고속 드론과 HDR 환경에서 쓰임새도 분명해졌다. Guillermo Gallego(TU Berlin) 등 event vision 연구자들은 [서베이](https://arxiv.org/abs/1904.08405)에서 event 기반 depth 추정과 ego-motion 추정 연구를 정리했다.

---

## 🧭 아직 열린 것

**Biologically-inspired SLAM.** foundation model이 대규모 비지도 학습으로 얻는 공간 표현을 인지 지도와 견주는 시각이 있다. transformer 내부 표현을 place cell과 같은 기제로 볼 수 있는지 검증하는 일은 남은 과제다. RatSLAM 같은 방법이 foundation model 안에서 다른 이름으로 돌아올지도 가설로 남아 있다.

**Event camera SLAM의 주류화.** 해상도가 높은 상용 event camera가 나오면서(§18.6) 연구 기반이 넓어졌다. frame 기반 pipeline과 합치는 문제, 새로운 event 표현, 실제 환경 benchmark를 늘리고 평가 기준을 세우는 일이 함께 진행 중이다.

**"Semantic map" 개념의 향방.** 2017년 무렵 빠르게 늘던 semantic SLAM 연구가 주춤한 뒤, semantic 표현은 움직이는 영역을 가려내는 일과 downstream task로 쓰임을 넓혔다. 2023년부터 LERF와 언어 기반 Gaussian splatting이 언어 feature를 dense scene representation에 붙이면서 다른 형태가 나왔다. Ch.15 🧭에서 본 것처럼 semantic Gaussian을 SLAM에 넣은 연구도 나왔다. 지금까지는 geometry가 먼저 갖춰진 뒤에 semantic이 쓰였다. representation이 바뀌면서 이 순서도 바뀔지는 아직 알 수 없다.
