# Ch.18 — 실패 사례와 사라진 계보

LOAM과 FAST-LIO2가 성숙해가던 같은 시간, 로보틱스 커뮤니티 안에는 다른 방향으로 걷고 있던 사람들이 있었다. 카메라 계보도 LiDAR 계보도 아닌 계보들. 그들이 선택받지 못했다고 해서 역사에 없던 것은 아니다.

SLAM의 역사는 성공한 계보만으로 이루어지지 않는다. 매 10년마다 충분한 논문과 초기 결과를 갖추고도 주류로 진입하지 못한 접근들이 있었다. 공학적 확장이 막히거나, 더 실용적인 대안이 먼저 자리를 잡은 경우였다. 기술적 실패와는 다른 문제였다.

---

## 18.1 RatSLAM — place cell 기반 위상 지도

2004년 ICRA에서 [Milford et al. 2004](https://doi.org/10.1109/ROBOT.2004.1302555)가 발표한 RatSLAM은 장소 인식 문제를 전혀 다른 방식으로 접근했다. 쥐의 해마 안에 있는 **place cell**과 **head direction cell**의 발화 패턴을 모방해, 로봇이 환경을 탐색하면서 자연스럽게 장소 표현을 형성하게 했다. 계산 모델의 이름은 **Continuous Attractor Network(CAN)**이었다. 뉴런들의 활성화 상태가 2D 격자 위에서 연속적인 활성화 'bump'를 형성하고, 로봇의 속도·회전 입력(path integration)에 따라 그 bump가 격자를 따라 이동하는 구조다. 시각 입력이 들어오면 저장된 장소 표현과 비교해 bump 위치를 보정(correction)한다. 이 loop(이동으로 인한 bump 전파, 시각 매칭으로 인한 보정)이 RatSLAM의 핵심 동작 원리다.

> 🔗 **차용.** [O'Keefe와 Dostrovsky(1971)](https://pubmed.ncbi.nlm.nih.gov/5124915/)의 place cell 발견은 신경과학에서 시작해 인지 지도(cognitive map) 이론으로 이어졌다. RatSLAM은 그 생물학적 메커니즘을 공학 시스템으로 옮긴 최초의 완성된 시도였다. 문은 열렸지만 안으로 들어온 공학자는 많지 않았다.

Milford와 Gordon Wyeth는 Queensland University of Technology(QUT) 로보틱스 연구실을 거점으로, 2004년부터 2008년 사이에 브리즈번 교외 도로에서 실외 주행 실험을 반복했다. 실험 차량은 지붕에 카메라를 달고 교외 주택가를 달렸다. RatSLAM은 그 이미지 스트림을 받아 이미 지나온 길을 알아보고 loop를 닫았다. [Milford & Wyeth 2008](https://doi.org/10.1109/TRO.2008.2004520) IEEE T-RO 논문에는 66km 경로에서 수만 장의 이미지를 처리한 결과가 실렸다. 같은 시기 기하학적 SLAM 시스템들이 몇 백 미터 단위에서 고전하던 때였으니, 숫자만 보면 RatSLAM이 앞서 있었다.

그러나 공학적 확장은 거기서 멈췄다. CAN은 장소 수가 늘수록 계산 복잡도가 올랐다. 더 깊은 문제는 정밀도였다. RatSLAM이 만드는 위상 지도(topological map)는 "여기 왔던 적 있다"는 판단은 했지만, 미터 단위의 metric 위치 추정은 안정적으로 내놓지 못했다. 자율주행과 조작(manipulation)이 요구하는 것은 정확한 좌표였다. 인지 지도는 그 요구에 맞지 않았다.

> 📜 **예언 vs 실제.** Milford·Wyeth는 2008년 T-RO 논문 Conclusion에서 RatSLAM이 "vision-only SLAM의 대안적 접근"이며, 기존 state-of-the-art SLAM에게는 도전이 될 만한 환경—장거리 경로, 큰 누적 오차, 시각적 모호성—에서 반복적이고 신뢰도 높은 loop closure를 수행한다고 주장했다. 대체가 아니라 대안으로서의 위치 주장이었다. 실제 전개는 이 주장 자체에 대해서는 부분적으로 맞았다. RatSLAM은 특정 benchmark에서 경쟁력을 보였다. 그러나 이후 분야 전체의 흐름에서는 2012년 이후 graph-based SLAM과 visual odometry가 정확도·속도 모두에서 앞서 나갔고, 위상 지도는 지금도 일부 place recognition 연구에 등장하지만, metric-topological 통합이라는 RatSLAM의 원래 야망은 다른 방식으로 이어지지 않았다. `[부분적중+무산]`

RatSLAM이 남긴 것은 알고리즘 자체가 아니었다. "장소 표현이 기하학 없이도 가능하다"는 아이디어가 place recognition 문헌에 스며들었다. 2012년 [SeqSLAM](https://doi.org/10.1109/ICRA.2012.6224623)이 같은 Milford 그룹에서 나왔고, 이미지 시퀀스 비교 기반 장소 인식은 visual place recognition 벤치마크의 한 축이 됐다. 계보 자체는 살아남았고, 다만 형태가 달라졌다.

---

## 18.2 biologically-inspired SLAM의 공학적 한계

RatSLAM은 biologically-inspired SLAM의 가장 완성된 사례였지만 혼자가 아니었다. 2000년대 중반부터 2010년대 초반까지 인지 지도, entorhinal grid cell, hippocampal replay를 모방한 SLAM 변형들이 꾸준히 나왔다. 모두 비슷한 문제를 안고 있었다.

생물학적 모델은 뇌가 *어떻게* 공간을 표현하는지 기술하지만, 그것이 *왜* 그 방식인지, 그 방식이 공학적 목적에도 맞는지는 다른 질문이다. 쥐의 해마는 수억 년의 진화가 특정 환경과 행동 패턴에 맞게 빚은 구조다. 로봇이 작동하는 조건과 같지 않다.

공학적 SLAM은 미터 이하의 위치 추정 정확도, 실시간 처리, 새로운 환경에 대한 빠른 적응, 검증 가능한 오류 경계를 요구한다. 인지 모델은 이 조건들을 보장하기 어려웠다. 신경과학과 로봇공학은 서로에게서 영감을 얻을 수 있지만, 그 간격은 짧지 않았다.

2020년대 들어 이 논의는 다시 열릴 여지가 생겼다. Foundation model이 large-scale representation을 스스로 형성하는 방식이 place cell의 emergent 특성과 구조적으로 닮았다는 관찰이 나왔다. 재발견인지, 다른 경로의 수렴인지는 아직 모른다.

---

## 18.3 Event SLAM — 하드웨어와 알고리즘 성숙 격차

Patrick Lichtsteiner, Christoph Posch, Tobi Delbruck가 ETH Zürich Institute of Neuroinformatics(INI)에서 개발한 [Dynamic Vision Sensor(DVS)](https://doi.org/10.1109/JSSC.2007.914337)는 ISSCC 2008에서 처음 공개됐다. 각 픽셀이 독립적으로 대수(log) 광도 변화를 임계값과 비교해 양(ON) 또는 음(OFF) 극성의 이벤트를 비동기(asynchronous)로 출력하는 구조다. 전역 셔터 없이 픽셀별로 발화 시점을 마이크로초 단위로 기록한다. 프레임이 없는 카메라였다.

> 🔗 **차용.** DVS event sensor(Lichtsteiner et al. 2008)는 생물학적 망막의 변화 감지 메커니즘에서 착안한 하드웨어였다. Event SLAM은 이 센서를 손에 쥐고 시작했다. 하드웨어가 알고리즘을 앞서 달렸고, 그 간격을 메우는 데 10년이 걸렸다.

이벤트 카메라의 장점은 목록으로 나열하기 좋았다. μs 단위의 시간 해상도, 고속 운동에서 블러 없음, 고동적 범위(HDR)로 터널과 햇빛 직사 환경 모두 대응, 전력 소비는 기존 카메라의 수십 분의 일. 논문에 쓰기 좋은 숫자들이었다.

2014년 ICRA에서 [Weikersdorfer et al. 2014](https://doi.org/10.1109/ICRA.2014.6906882)는 event 기반 3D SLAM을 발표했다. 같은 해 다른 그룹에서도 event-based optical flow와 depth 추정이 나왔다. 2016-2018년 사이에 Henri Rebecq(Davide Scaramuzza 그룹, University of Zurich RPG 연구실)가 [EVO](https://doi.org/10.1109/LRA.2016.2645143)(RA-L 2017)와 [ESIM](https://proceedings.mlr.press/v87/rebecq18a.html)(CoRL 2018) 등을 발표하면서 event SLAM 파이프라인이 구체화됐다.

현실 환경에서 결과는 기대에 미치지 못했다. 문제는 두 곳에 있었다. 첫째는 해상도였다. 초기 DVS 센서는 128×128 픽셀이었다. 기존 VGA 카메라와 비교할 수 없는 수준이었고, feature matching과 map building이 해상도에 직접 의존하는 SLAM에서 이 제약은 컸다. 둘째는 알고리즘 패러다임 자체였다. 기존 프레임 기반 알고리즘을 이벤트 스트림에 그대로 쓸 수 없었다. 새로운 방식이 필요했고, 그 개발에 시간이 걸렸다.

2014년부터 2018년까지 event SLAM은 controlled 환경과 low-texture 조건에서 좋은 결과를 냈지만, 일반 환경에서 기존 visual-inertial odometry를 앞서지 못했다.

그 사이에도 event 접근의 적용 범위는 odometry 바깥으로 조용히 번졌다. [EventVLAD](https://ieeexplore.ieee.org/document/9635907/)(Lee & Kim, IROS 2021)는 event stream에서 복원한 edge 이미지를 NetVLAD descriptor로 묶어, 급격한 조명 변화와 모션 블러 조건에서도 장소 재인식이 가능함을 보였다. frame 기반 VPR이 어려워하던 환경을 event가 대신 건드리는 시도였다.

---

## 18.4 Semantic SLAM — object-as-landmark 경로의 축소

2017년부터 2019년까지 CVPR, ECCV, IROS 세션 제목에는 "semantic"이 빠지지 않았다. 딥러닝이 instance segmentation과 object detection에서 연속으로 돌파구를 열던 시기였다. SLAM 연구자들은 질문을 던졌다. "이 semantic 이해를 SLAM에 통합하면 무슨 일이 일어날까?" 질문 자체가 틀린 것은 아니었는데, 실행이 담론을 따라가지 못했다.

[Salas-Moreno et al. 2013](https://doi.org/10.1109/CVPR.2013.178)의 **SLAM++**가 그 계보의 첫 대형 선언이었다. Imperial College의 Salas-Moreno와 지도교수 Andrew Davison 그룹은 기존의 포인트나 패치 대신 *사물(object)*을 지도의 기본 단위로 삼았다. 의자, 책상, 모니터 같은 사전 정의된 3D 객체 모델을 데이터베이스에 저장하고, SLAM 실행 중 RGB-D 입력에서 ICP(Iterative Closest Point) 기반 정합으로 그 객체들을 인식해 지도에 올렸다. 포인트 수천 개 대신 객체 수십 개로 지도를 표현하면, 맵 크기가 줄고 장소 인식과 loop closure가 더 의미론적으로 이루어질 수 있었다.

> 🔗 **차용.** SLAM++의 object-level representation은 그래픽스의 scene graph 표현과 컴퓨터비전의 model-based recognition을 결합한 것이었다. 그 아이디어는 2020년대 LERF(Language Embedded Radiance Field)와 LangSplat으로 이어졌다. 표현 단위가 object에서 language feature로 바뀌었을 뿐, "지도가 semantic해야 한다"는 직관은 살아남았다.

SLAM++ 이후 2017-2019년 사이에 [SemanticFusion](https://arxiv.org/abs/1609.05130)(McCormac et al., 2017, ICRA), [MaskFusion](https://arxiv.org/abs/1804.09194)(Rünz et al., 2018, ISMAR), [SuperPoint](https://arxiv.org/abs/1712.07629)(DeTone et al., 2018) 기반 feature 계열 등이 연달아 발표됐다. 이 시기의 공통된 주장은 하나였다. "deep semantic feature는 geometric feature보다 환경 변화에 강건하고, semantic 이해가 통합된 SLAM이 다음 단계다."

실제 전개는 달랐다. 2019년까지 autonomous driving benchmark에서 성능을 끌어올린 것은 ORB-SLAM2, VINS-Mono, LIO-SAM 같은 전통적 geometric 파이프라인이었다. Deep semantic feature를 통합한 시스템들은 특정 실내 환경과 고정된 객체 클래스에서만 경쟁력이 있었다. 새로운 객체 카테고리나 처음 보는 환경에서 semantic prior가 오히려 drift를 키우는 사례도 나왔다.

> 📜 **예언 vs 실제.** Salas-Moreno는 SLAM++ 논문 Conclusion에서 자신들의 방식이 "보다 일반적(generic) SLAM 방법으로 가는 첫 걸음"이라며, 낮은 차원의 형상 변이를 갖는 객체, 나아가 장기적으로는 스스로 객체 클래스를 분할·정의하는 시스템으로 확장되기를 기대했다. 논문 도입부는 이에 더해 객체 단위 표현이 "맵 저장량의 큰 압축"과 "효율성·견고성 이득"을 준다고 주장했다. 실제 전개는 일부만 적중했다. Object-level map은 AR과 특정 manipulation 응용에서 자리를 찾았고, 압축·효율 측면의 이점은 실내 반복 객체 환경에서 재확인됐다. 그러나 주류 geometric SLAM은 2026년 기준에도 sparse point와 keyframe 기반 graph를 유지하고 있고, 객체를 스스로 segmentation·정의하는 단계는 도달하지 못했다. Semantic은 결국 SLAM의 내부가 아니라 하류(downstream) 태스크—semantic mapping, task planning—에 자리를 잡았다. `[부분 적중+경로 전환]`

왜 semantic-first SLAM은 주류가 되지 못했나. 원인은 두 곳에 있었다. 하나는 의존성이었다. Semantic SLAM은 segmentation이 정확해야 했는데, segmentation이 틀리면 지도 전체가 오염됐다. 기하학적 파이프라인은 feature matching이 부분적으로 실패해도 robust estimation으로 버텼다. 다른 하나는 일반화였다. 특정 객체 클래스로 훈련한 semantic prior는 그 클래스 밖에서 쓸모가 없었다. SLAM이 들어가야 할 환경은 그 prior가 상정한 세계보다 훨씬 넓었다.

축소된 것은 object-as-landmark 경로였지, semantic 자체는 아니었다. 같은 시기 다른 경로가 살아남았다. [SuMa++](https://doi.org/10.1109/IROS40897.2019.8967704)(Chen et al., IROS 2019)가 LiDAR point cloud에 semantic class를 덧씌워 동적 물체를 걸러냈고, [Kimera](https://doi.org/10.1109/ICRA40945.2020.9196885)(Rosinol et al., ICRA 2020)가 metric-semantic mesh와 3D scene graph를 묶었다. [Hydra](https://doi.org/10.15607/RSS.2022.XVIII.050)(Hughes et al., RSS 2022)는 그 scene graph를 실시간·계층적으로 확장했고, [ConceptGraphs](https://doi.org/10.1109/ICRA57147.2024.10610243)(Gu et al., ICRA 2024)와 [Clio](https://doi.org/10.1109/LRA.2024.3451395)(Maggio et al., RA-L 2024)에 이르러 open-vocabulary foundation feature가 그 위에 얹혔다. Semantic은 landmark 자리가 아니라 지도의 상위 layer로 올라가 살아남았다. 이 계보는 2026년까지 진행 중이고, [Ch.15b](chapter_15b_dynamic.md)(Dynamic·static 분리의 semantic 귀환), [Ch.16](chapter_16_foundation_3d.md)(foundation 3D·metric-semantic 본체), [Ch.19 §19.7](chapter_19_open_problems.md#197-semantic-표현의-귀환과-open-world)(Semantic의 귀환)에서 이어 다룬다.

---

## 18.5 Manhattan-World 가정 — 적용 범위와 소멸

비슷한 시기, 다른 계보가 조용히 시도됐다가 조용히 사라졌다. Manhattan-world assumption을 이용한 SLAM이었다.

가정 자체는 단순했다. [Coughlan & Yuille 1999](https://doi.org/10.1109/ICCV.1999.790349)의 Manhattan world 개념을 이어받아, 실내 환경은 대부분 세계 좌표계의 세 직교 축(x, y, z)에 정렬된 구조라고 봤다. 벽, 바닥, 천장이 그 방향을 만든다. 이미지 속 평행선 묶음은 소실점(vanishing point)으로 수렴하며, 각 소실점은 카메라의 회전 행렬 R과 방향 벡터 d의 관계 `v = K R d`로 기술된다(K: 카메라 내부 행렬). 세 직교 소실점을 찾으면 R의 세 열을 직접 복원할 수 있다. IMU나 feature matching 없이 기하학적 제약만으로 drift를 억제할 수 있다는 뜻이었다. 이 아이디어를 visual odometry와 결합하려는 시도들이 이 시기에 등장했다.

긴 복도와 직사각형 방에서는 drift가 실제로 줄었다. 문제는 그 밖이었다. 야외로 나가거나, 둥근 구조물이 있거나, 불규칙한 산업 환경에 들어서면 Manhattan-world 가정 자체가 성립하지 않았다. 환경에 꼭 맞춘 prior는 그 환경 밖에서 오히려 발목을 잡았다. 2015년 이후 general-purpose visual-inertial odometry가 성숙하면서 이 계보는 관심을 잃었다. 일부 실내 mapping 도구에 보조 제약으로 남아 있지만, 독립 연구 계보로는 사라졌다.

---

## 18.6 소멸 계보의 재발견 패턴

계보가 죽는다는 것이 무엇을 뜻하는지는 사례마다 다르다. RatSLAM의 topological map 아이디어는 SeqSLAM으로 이어졌고, 그 후예가 visual place recognition 분야에서 살아 있다. SLAM++의 object-level map 직관은 2022년 이후 NeRF와 Gaussian splatting이 언어와 결합하면서 다른 형태로 돌아왔다. [LERF](https://arxiv.org/abs/2303.09553)(Kerr et al., 2023)와 [LangSplat](https://arxiv.org/abs/2312.16084)(Qin et al., 2023)이 그 경우다.

Event camera SLAM은 경로가 달랐다. 알고리즘이 막혔던 것이 아니라 하드웨어가 아직 거기까지 오지 못했던 것이었다. 2022년 이후 640×480 이상의 event camera가 시장에 나왔고, 고속 드론과 HDR 환경에서의 필요가 분명해졌다. [Guillermo Gallego](https://arxiv.org/abs/1904.08405)(TU Berlin)를 중심으로 한 event vision 커뮤니티는 2020-2024년 사이에 event-based depth estimation과 ego-motion 추정에서 경쟁력 있는 결과를 냈다.

영감이 좋아도 공학이 따라오는 데 시간이 걸리고, 센서가 새로워도 알고리즘은 따로 만들어야 한다. 그 간격을 메우는 데 얼마나 걸리느냐는 알고리즘의 성숙도와 하드웨어의 실용화 속도에 달렸다. 그리고 그 사이에 더 나은 대안이 먼저 자리를 잡느냐도.

---

## 🧭 아직 열린 것

**Biologically-inspired SLAM.** Foundation model이 대규모 비지도 학습으로 공간 표현을 형성하는 방식은 인지 지도와 구조적으로 닮은 특성이 있다. Place cell과 유사한 단위가 transformer 내부에서 관찰됐다는 보고가 2023-2024년에 나왔다. 수렴인지 우연인지는 모른다. RatSLAM류의 계보가 foundation model 패러다임 안에서 다른 이름으로 돌아올 가능성은 있다.

**Event camera SLAM의 주류화.** 2022년 이후 상업용 고해상도 event camera가 보급되면서 연구 기반이 넓어졌다. 그러나 event 데이터를 효과적으로 처리하는 알고리즘 패러다임은 아직 안정적인 공통 프레임워크를 갖추지 못했다. Frame 기반 pipeline과의 통합과 새로운 event representation, 그리고 real-world benchmark의 다양화와 평가 기준 정립이 동시에 진행 중이다. 주류화 여부는 2026년 기준에도 판단이 이르다.

**"Semantic map" 개념의 향방.** 2017년 semantic SLAM의 과열이 식은 뒤, semantic 표현은 SLAM 외부—downstream task—로 밀려났다. 2024-2025년 LERF와 Gaussian splatting이 언어 feature를 밀도 있는 scene representation과 결합하면서 다른 형태가 나왔다. 내부화로 이어질지, 다시 downstream으로 남을지는 모른다. geometry가 먼저 옳아야 semantic이 쓸모 있다는 패턴이 이번에도 반복될지, 아니면 representation 자체의 변화가 그 순서를 바꿀지가 관건이다. 2026년 기준 "풀렸다"고 여기는 것들 가운데 미래의 누군가가 이 챕터에 추가할 이름은 아직 나오지 않은 것 중에 있을 것이다.
