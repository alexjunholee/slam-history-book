# 22장 사라지거나 축소된 SLAM 계보

카메라 기반 SLAM과 LiDAR 기반 SLAM이 거대한 주류 고속도로를 닦아 나가는 동안, 로보틱스 연구자 일부는 그 어느 쪽에도 속하지 않는 샛길로 들어섰다. 쥐의 해마를 흉내 낸 생체 모방 지도, 프레임 없이 빛의 변화만 쏘아 대는 이벤트 카메라, 점이 아니라 의자와 탁자를 기본 단위로 삼는 semantic 지도, 복도의 직교성에 기대는 구조적 prior가 그 샛길들이었다.

이들은 모두 한때 굵직한 논문과 인상적인 초기 시연을 내놓고도 주류 파이프라인의 자리를 차지하지 못했다. 그러나 실패의 이유는 저마다 달랐고, 그 가운데 일부는 전혀 다른 이름과 모습으로 2020년대의 연구 속에 다시 나타났다.

---

## 22.1 RatSLAM — place cell 기반 위상 지도

2004년 ICRA에서 호주 University of Queensland의 Michael Milford, Gordon Wyeth, David Prasser가 발표한 [RatSLAM(Milford et al. 2004)](https://doi.org/10.1109/ROBOT.2004.1307183)은 쥐의 뇌에서 공간을 부호화하는 **place cell**과 **head direction cell**을 본떠 로봇의 위치를 추정했다. 시스템의 심장은 `(x, y, θ)` 세 축으로 배열된 경쟁 끌개 신경망(competitive attractor network), 곧 **pose cell**이다. 이 신경망 안에서 활성이 한곳에 뭉쳐 있는 덩어리(activity packet)가 로봇의 현재 위치와 방향을 뜻한다. 바퀴 엔코더가 이동량을 보고하면(path integration) 활성 덩어리가 격자 위를 옮겨 가고, 로봇이 전에 본 적이 있는 장면(local view)을 다시 마주치면 그 장면과 연결된 pose cell이 깨어나면서 덩어리의 위치를 교정한다.

그 야심은 2008년 IEEE TRO 논문 [Milford & Wyeth 2008](https://doi.org/10.1109/TRO.2008.2004520)에서 극적인 야외 시연으로 증명됐다. 연구진은 승용차 지붕에 노트북 내장 웹캠 단 한 대를 올려놓고 브리즈번 St Lucia 교외 도로를 100분 동안 66 km 달렸다. RatSLAM은 초당 10프레임으로 쏟아지는 저화질 영상을 실시간으로 소화하며 길이가 최대 5 km에 달하는 루프를 51개 넘게 닫았다. 결과물로 얻은 반계량 위상 지도(semi-metric topological map)가 덮은 면적은, 2007년까지 레이저 센서 하나로 작성된 가장 넓은 지도라고 보고된 기존 연구진의 지도보다 다섯 배 넘게 넓었다.

그런데도 RatSLAM은 SLAM의 주류가 되지 못했다. 장소가 쌓일수록 탐색 연산이 커진 문제도 있었지만, 더 치명적인 약점은 정밀도였다. RatSLAM이 빚어낸 위상 지도(topological map)는 "여기에 와 본 적이 있다"는 연결 관계를 훌륭하게 기억했다. 하지만 차선을 지켜야 하는 자율주행과 물체를 집어 올려야 하는 조작(manipulation)에서는 센티미터·미터 단위의 정확한 metric 좌표가 필요한데, 위상 지도는 그 좌표를 안정적으로 내주지 못했다.

그럼에도 RatSLAM이 남긴 불씨는 꺼지지 않았다. 정밀한 3D 기하 복원 없이도 장소를 알아볼 수 있다는 통찰은 곧장 장소 인식(place recognition) 연구로 번졌다. 2012년 Milford와 Wyeth가 내놓은 [SeqSLAM](https://doi.org/10.1109/ICRA.2012.6224623)은 단일 이미지 대신 연속된 이미지 시퀀스를 통째로 비교해 계절과 날씨가 뒤바뀐 도로에서도 장소를 찾아냈고, visual place recognition이라는 독립된 갈래의 주춧돌이 됐다.

> 🔗 **차용.** [O'Keefe와 Dostrovsky(1971)](https://pubmed.ncbi.nlm.nih.gov/5124915/)가 발견한 해마의 place cell은 신경과학에서 인지 지도 이론의 토대를 세웠다. RatSLAM은 바로 그 생물학적 메커니즘을 공학용 로봇 내비게이션 시스템으로 옮겨 심은 가장 대표적인 시도였다.

> 📜 **예언 vs 실제.** Milford와 Wyeth는 2008년 TRO 논문 결론에서 RatSLAM을 "vision-only SLAM의 대안적 접근"이라 부르며, 기존 state-of-the-art SLAM이 쩔쩔매는 거친 환경에서도 큰 오차와 모호한 풍경을 뚫고 거대 루프를 반복해서 닫아 보였다고 자평했다. 절반은 적중했다. 정밀 3D 위치 추정에서는 기하학 기반 SLAM에 밀렸지만, 위상 지도와 순차 기억이라는 아이디어는 오늘날까지 place recognition 연구 한복판에 살아남았다.

---

## 22.2 biologically-inspired SLAM의 공학적 한계

RatSLAM은 생물 모방 SLAM(biologically-inspired SLAM) 가운데 가장 완성도 높은 시스템이었다. 2000년대 중반부터 2010년대 초반까지 해마의 인지 지도, 내후각 피질(entorhinal cortex)의 grid cell, 수면 중 기억 재생(hippocampal replay)을 본뜬 SLAM 변형들이 뒤를 이었지만, 끝내 비슷한 벽에 부딪혔다.

생물학적 모델은 뇌가 공간을 *어떻게* 표상하는지 설명하려는 과학적 가설이다. 그러나 뇌가 *왜* 그런 근사 방식을 택했는지, 그리고 그 방식이 로봇 공학의 요구 스펙과 맞아떨어지는지는 전혀 다른 문제다. 쥐의 해마가 생존을 위해 굴을 찾아가는 조건과, 로봇 벤치마크가 요구하는 센서·연산·정확도 조건은 애초에 달랐다. 공학적 SLAM에서는 미터 이하의 일관된 정밀도, 예측 가능한 실시간 지연, 낯선 환경에서의 즉각적 적응, 수치적으로 검증 가능한 오차 한계가 필요하다. 인지 신경과학의 우아한 모델만으로는 그 까다로운 공학 명세를 보장하기 어려웠다.

---

## 22.3 Event SLAM — 하드웨어와 알고리즘 성숙 격차

또 하나의 샛길은 아예 카메라 센서 자체를 바꾸려는 시도에서 열렸다. 2006년 ISSCC에서 Patrick Lichtsteiner, Christoph Posch, Tobi Delbruck이 공개하고([ISSCC 2006](https://tilde.ini.uzh.ch/users/tobi/public_html/papers/lichtsteiner_ISSCC2006_D27_09.pdf)), 2008년 JSSC로 확장한([JSSC 2008](https://doi.org/10.1109/JSSC.2007.914337)) Dynamic Vision Sensor(DVS)가 출발점이었다. ETH Zürich의 Institute of Neuroinformatics(INI) 소속 Lichtsteiner·Delbruck과 빈 ARC Seibersdorf Research의 Posch가 만든 이 센서는 셔터로 모든 픽셀을 한꺼번에 읽어 들이는 대신, 각 픽셀이 밝기의 로그값 변화를 스스로 감시하다가 임계값을 넘기는 순간에만 밝아졌는지(ON) 어두워졌는지(OFF)를 마이크로초 타임스탬프와 함께 비동기(asynchronous)로 쏘아 올렸다. 말 그대로 프레임이 없는 카메라였다.

이벤트 카메라의 장점은 목록으로 나열하기 좋았다. 마이크로초 단위의 시간 해상도, 고속 기동에서도 전혀 번지지 않는 선명함, 터널 내부와 직사광선 아래를 한 화폭에 담는 넓은 동적 범위(HDR), 그리고 [Gallego 등의 서베이](https://arxiv.org/abs/1904.08405)가 짚은 칩 수준 약 10 mW의 극소 전력 소비까지—모두 기존 프레임 카메라가 쩔쩔매던 약점을 정조준하는 매력적인 숫자였다.

실제 SLAM 접목도 빠르게 뒤따랐다. 2014년 ICRA에서 [Weikersdorfer et al. 2014](https://doi.org/10.1109/ICRA.2014.6906882)가 DVS에 깊이 센서를 얹은 event 기반 3D SLAM을 선보였고, 2016–2018년 사이에는 취리히대 Davide Scaramuzza 연구실(RPG)의 Henri Rebecq 등이 [EVO](https://doi.org/10.1109/LRA.2016.2645143)(RA-L 2017)와 시뮬레이터 [ESIM](https://proceedings.mlr.press/v87/rebecq18a.html)(CoRL 2018)을 잇달아 내놓으며 event SLAM 파이프라인의 얼개를 세웠다. 훗날 [EventVLAD](https://ieeexplore.ieee.org/document/9635907/)(Lee & Kim, IROS 2021)처럼 이벤트 스트림에서 잡음을 걷어 낸 edge 영상을 복원해 NetVLAD 서술자를 뽑음으로써 극심한 조명 변화 속 장소 인식을 돌파하려는 시도까지 이어졌다.

그럼에도 event SLAM이 주류 무대로 올라서지 못한 데는 하드웨어와 알고리즘 사이의 성숙 격차가 가로놓여 있었다. 우선 초기 해상도가 너무 낮았다. JSSC 2008 논문의 DVS는 128×128 픽셀로, 당시 흔한 VGA(640×480) 카메라의 약 19분의 1에 불과했다. 특징점 매칭과 세밀한 지도 작성이 해상도에 직결되는 SLAM에서 이 열세는 뼈아팠다. 더 큰 벽은 알고리즘이었다. 수십 년간 프레임 단위의 정지 영상을 전제로 깎아 온 컴퓨터 비전과 SLAM의 수학 전체를 비동기 이벤트 스트림에 맞게 처음부터 다시 써야 했고, 그 재설계의 비용과 시간은 센서의 이론적 장점만큼 빨리 좁혀지지 않았다.

> 🔗 **차용.** DVS는 생물의 망막 신경절 세포가 정적인 배경보다 밝기 변화에 반응해 신호를 보내는 원리에서 착안한 뉴로모픽 센서였다. event SLAM은 바로 그 생체 모방 하드웨어 위에서 태어난 갈래다.

---

## 22.4 Semantic SLAM — 사물을 landmark로 쓰는 방식

2017년부터 2019년 사이 CVPR, ECCV, IROS 학회장은 "semantic SLAM" 논문으로 들끓었다. 딥러닝이 instance segmentation과 object detection을 실시간 수준으로 끌어올리자, 이 의미 인식을 SLAM의 뼈대 안으로 들이밀려는 시도가 봇물처럼 터진 것이다.

사실 그 선구는 딥러닝 붐보다 앞선 2013년 CVPR에 이미 나와 있었다. Imperial College London의 Renato Salas-Moreno, Richard Newcombe, Hauke Strasdat, Paul Kelly, Andrew Davison이 발표한 **SLAM++**([Salas-Moreno et al. 2013](https://doi.org/10.1109/CVPR.2013.178))는 점이나 패치 대신 *사물(object)*을 지도의 기본 원자로 삼았다. 미리 스캔해 둔 사물 3D 모델을 데이터베이스에 넣어 두고, 손에 든 RGB-D 카메라 영상에서 GPU 병렬 point pair feature 투표로 사물을 실시간 인식한 뒤 ICP로 자세를 맞추어 카메라와 함께 포즈 그래프에 올려 최적화했다. 의자 두 종류와 둥근 탁자 두 종류로 방 한 칸을 재구성한 실험에서, SLAM++는 같은 부피의 KinectFusion 지도와 견주어 지도 메모리를 약 70분의 1로 압축했다고 추산했다. 점 수천 개를 나열하는 대신 사물 수십 개로 방을 요약하면 지도가 가벼워질 뿐 아니라 장소 인식과 loop closure까지 사물 단위로 풀 수 있다는 매력적인 비전이었다.

뒤이어 [SemanticFusion](https://arxiv.org/abs/1609.05130)(McCormac et al., ICRA 2017)이 dense 지도에 semantic label을 입혔고, [MaskFusion](https://arxiv.org/abs/1804.09194)(Rünz et al., ISMAR 2018)이 움직이는 사물을 인식해 개별적으로 추적했다. 같은 시기 [SuperPoint](https://arxiv.org/abs/1712.07629)(DeTone et al., CVPRW 2018)처럼 신경망으로 keypoint detector와 local descriptor를 함께 학습하는 연구도 등장했지만 이는 여전히 점 단위에 머물렀고, semantic SLAM 연구자들은 사물 단위의 의미 이해를 기하 파이프라인의 심장부에 넣어야 환경 변화에도 흔들리지 않는 강건한 지도를 얻을 수 있다고 주장했다.

그러나 2019년까지는 ORB-SLAM2와 VINS-Mono가 벤치마크의 정점을 지켰고, 2020년에는 LIO-SAM이 바통을 이어받았다. 당시의 semantic SLAM 시스템들은 저마다 다른 실내 방과 임의의 사물 데이터셋에서 평가돼 공정한 수평 비교 자체가 어려웠고, 미리 학습된 segmentation 모델과 닫힌 class 목록 바깥을 만나면 무력해진다는 한계를 벗어나지 못했다.

무엇보다 결정적인 걸림돌은 **의존성의 방향**이었다. 사물을 landmark로 세운 SLAM은 segmentation이 빗나가면 지도 전체가 오염되는 취약점을 안고 있었다. 반면 고전적인 기하 파이프라인은 특징점 매칭 일부가 틀려도 RANSAC과 robust kernel로 묵묵히 버텨 냈다. 특정 사물 범주로 훈련된 semantic prior는 훈련 데이터 밖의 세계로 나가자마자 빛을 잃었다.

흥미롭게도 그다음에는 자리바꿈이 일어났다. 사물을 기하 landmark로 쓰려는 시도는 주류에서 물러났지만, semantic 자체는 사라지지 않고 파이프라인의 *바깥과 위층*으로 옮겨 가며 꽃을 피웠다. [SuMa++](https://doi.org/10.1109/IROS40897.2019.8967704)(Chen et al., IROS 2019)는 LiDAR 스캔에서 움직이는 물체를 걸러 내는 필터로 semantic을 썼고, [Kimera](https://doi.org/10.1109/ICRA40945.2020.9196885)(Rosinol et al., ICRA 2020)와 [3D Dynamic Scene Graphs](https://doi.org/10.15607/RSS.2020.XVI.079)(Rosinol et al., RSS 2020), [Hydra](https://doi.org/10.15607/RSS.2022.XVIII.050)(Hughes et al., RSS 2022)는 실시간 metric-semantic mesh 위에 계층적 3D scene graph를 구축했으며, [ConceptGraphs](https://doi.org/10.1109/ICRA57147.2024.10610243)(Gu et al., ICRA 2024)와 [Clio](https://doi.org/10.1109/LRA.2024.3451395)(Maggio et al., RA-L 2024)는 그 위에 open-vocabulary foundation feature를 얹었다. 기하를 대체하는 대신 기하 위에 의미를 쌓아 올리는 이 흐름은 2026년에도 계속되고 있다(→ [19장](chapter_15b_dynamic.md), [20장](chapter_16_foundation_3d.md), [23장 §23.7](chapter_19_open_problems.md#237-semantic-표현과-open-world)).

> 🔗 **차용.** SLAM++의 사물 인식 엔진은 컴퓨터 비전의 model-based recognition 계보에서 왔다. 구체적인 3D 투표 기법은 Drost 등이 제안한 point pair feature 매칭 알고리즘을 GPU로 옮겨 구현한 것이다.

> 📜 **예언 vs 실제.** Salas-Moreno 등은 SLAM++ 논문 결론에서 자신들의 접근을 "보다 일반적인(generic) SLAM 방법으로 가는 첫걸음"이라 부르며, 변형된 사물 인스턴스를 포용하고 장기적으로는 시스템이 스스로 사물 범주를 발견해 정의하게 되기를 기대했다. 초록에서 내세운 지도 압축 효과는 같은 사물이 반복되는 실내 환경과 AR·조작 응용에서 분명히 확인됐다. 그러나 2026년 현재에도 주류 geometric SLAM의 바닥은 여전히 sparse point와 keyframe graph가 지키고 있으며, 시스템이 스스로 사물 클래스를 창조하는 단계에는 이르지 못했다. 대신 semantic은 동적 영역을 가려내는 필터와 SLAM 위층의 계획용 scene graph로 자리를 옮겼다.

---

## 22.5 Manhattan-World 가정 — 좁아진 적용 범위와 보조 제약으로 남은 쓰임

비슷한 시기에 반짝했던 또 하나의 우회로는 환경의 기하적 규칙성을 극대화한 Manhattan-world SLAM이었다. 아이디어는 [Coughlan & Yuille 1999](https://doi.org/10.1109/ICCV.1999.790349)의 Manhattan world 개념에서 왔다. 실내 건축물은 대부분 세계 좌표계의 세 직교 축(x, y, z)에 맞춰 지어지므로, 벽과 바닥·천장이 만들어 내는 평행선 묶음은 영상 속에서 세 개의 직교 소실점(vanishing point)으로 모인다. 각 소실점은 카메라 회전 행렬 `R`과 축 방향 벡터 `d`로 `v = K R d`(K: 카메라 내부 행렬)라는 깔끔한 대수 관계를 이루기 때문에, 서로 직교하는 소실점 세 개만 잡으면 IMU나 특징점 매칭 없이도 카메라 회전 `R`의 세 열을 곧장 계산해 회전 drift를 0으로 눌러 버릴 수 있었다.

이 구조적 prior를 visual odometry와 묶으려는 시도는 길고 반듯한 복도와 직사각형 방에서 인상적인 drift 억제 효과를 냈다. 문제는 문밖으로 한 걸음만 나가도 가정이 무너진다는 점이었다. 둥근 기둥이나 곡면 벽, 야외의 자연 지형, 비정형 파이프가 얽힌 산업 현장에 들어서면 직교 소실점 자체가 사라졌다. 결국 Manhattan-world 가정은 범용 SLAM의 주축이 되지 못한 채, 건축 실내라는 조건이 보장될 때만 꺼내 쓰는 보조 제약으로 물러났다.

---

## 22.6 사라진 방법이 되살아나는 패턴

"사라졌다"는 한마디로 묶기에는 각 갈래의 뒷모습이 저마다 다르다. RatSLAM의 위상 지도 발상은 SeqSLAM을 거쳐 오늘날 visual place recognition 연구의 혈관 속에 남아 있다. SLAM++와 [LERF](https://arxiv.org/abs/2303.09553)(Kerr et al., ICCV 2023)·[LangSplat](https://arxiv.org/abs/2312.16084)(Qin et al., CVPR 2024)은 지도에 의미를 싣는다는 목표를 공유하지만, 후자가 SLAM++의 직접적인 인용 계보를 이었다기보다는 새로운 3D 표현 위에서 같은 염원이 독립적으로 되살아난 경우에 가깝다.

반면 event camera SLAM의 궤적은 하드웨어가 뒤늦게 따라붙으며 다시 열리고 있다. 초기 128×128 해상도의 갑갑함을 딛고 2017년 ISSCC에서 Samsung이 640×480 DVS를 발표했고([Son et al. 2017](https://doi.org/10.1109/ISSCC.2017.7870263)), 2021년에는 Sony와 Prophesee가 1280×720 HD 해상도의 IMX636 센서를 [세상에 내놓았다](https://www.sony-semicon.com/en/news/2021/2021090901.html). 고속 레이싱 드론과 극단적 HDR 환경에서 event 센서의 진가가 재확인되면서, Guillermo Gallego(TU Berlin) 등이 정리한 [서베이](https://arxiv.org/abs/1904.08405)를 중심으로 event 기반 ego-motion과 깊이 추정 연구가 다시 탄력을 받고 있다.

---

## 아직 열린 문제

**Biologically-inspired SLAM.** 대규모 비지도 학습을 거친 foundation model이 내부에 빚어내는 공간 표현을 신경과학의 인지 지도와 나란히 놓고 바라보는 시각이 다시 고개를 들고 있다. transformer의 어텐션 패턴이나 잠재 공간이 실제로 해마의 place cell과 닮은 기제를 학습하는지 검증하는 과제는 아직 풀리지 않았으며, RatSLAM이 꿈꿨던 생물 모방 공간 기억이 전혀 다른 이름으로 돌아올지도 흥미로운 가설로 남아 있다.

**Event camera SLAM의 주류화.** 고해상도 상용 event camera가 손에 닿으면서 연구의 토양은 한결 넓어졌다(§22.6). 그럼에도 전통적인 프레임 기반 파이프라인과 매끄럽게 엮는 하이브리드 구조, 이벤트 스트림을 다루는 보편적 표현, 그리고 실제 환경에서의 공정한 표준 벤치마크를 세우는 일은 여전히 진행 중이다.

**"Semantic map" 개념의 향방.** 2023년 이후 LERF와 언어 기반 Gaussian splatting이 자연어 feature를 dense 3D 표현 속에 직접 심기 시작하면서 지도의 성격이 또 한 번 달라지고 있다. 지금까지는 기하 지도를 먼저 세운 뒤 그 위에 의미를 덧칠하는 순서가 당연시됐지만, 표현 자체가 언어와 기하를 한 몸에 품기 시작한 지금 그 순서마저 뒤바뀔지는 아직 아무도 단정하지 못한다.

---

지금까지 0장에서 22장까지, 우리는 사진측량의 기하학에서 출발해 필터와 그래프, 직접법과 딥러닝, LiDAR와 사라진 샛길들까지 SLAM 반세기의 계보를 모두 건너왔다. 이제 마지막 장에서는 그 모든 여정 끝에서 여전히 우리를 가로막고 서 있는 미해결 난제들을 한자리에 모아, SLAM이라는 분야가 아직 어디까지 가지 못했는지를 정직하게 가늠해 본다.
