# 19장 동적·변형 장면 SLAM

고전 필터링부터 NeRF와 3D Gaussian Splatting에 이르기까지, SLAM의 주류 계보는 오랫동안 침묵의 전제 하나를 공유해 왔다. 카메라는 움직여도 세상은 멈춰 서 있다는 가정이다. 그러나 실험실을 벗어나 차들이 뒤섞인 도심 도로를 달리는 자율주행차, 사람이 오가며 의자와 물건을 옮겨 놓는 집 안의 서비스 로봇, 심지어 숨 쉬고 박동하며 장기 전체가 출렁이는 몸속의 수술 로봇 앞에서는 그 전제가 첫 프레임부터 무너진다.

이 균열을 짚어 온 역사는 생각보다 길다. Jorge Fuentes-Pacheco가 동료 Ruiz-Ascencio, Rendón-Mancha와 함께 쓴 서베이 [*Visual simultaneous localization and mapping: a survey*](https://link.springer.com/article/10.1007/s10462-012-9365-8)는 2012년 *Artificial Intelligence Review* 온라인판(2015년 43권 정식 수록)에서부터 동적이고 복잡한 대규모 환경을 아직 풀리지 않은 핵심 난제로 지목하며, 글의 마지막 절을 아예 "Dynamic and Deformable Environments"로 세웠다. 그 이전부터 움직이는 물체를 다루려는 논문은 있었지만, 분야 전체가 이 문제를 정면으로 체계화하기까지 시간이 걸렸다. 2026년 4월 21일자 *The SLAM Handbook* 원고에서는 Lukas Schmid, José María Martínez Montiel, Shoudong Huang, Daniel Cremers, José Neira, Javier Civera 여섯 명이 집필해 이 주제에만 37쪽을 할애했다. 그 문제의식이 마침내 중심부로 올라왔음을 보여 주는 대목이다.

---

## 19.1 세 개의 축

무엇을 "동적"이라 부를 것인가. Schmid 등은 *The SLAM Handbook* 15장 §15.1에서 Looper et al.(2023)의 정의를 빌려, 동적 여부가 물리 현상 자체가 아니라 *관측*과의 관계에서 갈린다고 설명한다. 똑같은 물리적 이동이라도 어떤 로봇에게는 프레임이 흘러가는 동안 눈에 보이는 **short-term dynamic**이 되고, 다른 로봇에게는 두 번의 방문 사이에 묵묵히 쌓여 버린 **long-term dynamic**이 된다. 둘을 가르는 기준은 장면의 변화율 $\text{Dyn}$과 로봇의 관측률 $\text{Obs}$의 비율이다. $\text{Dyn} \ll \text{Obs}$이면 로봇은 움직이는 물체를 매 프레임 연속해서 목격하지만, $\text{Dyn} \gg \text{Obs}$이면 로봇이 다시 돌아왔을 때 이미 의자가 옮겨지고 상자가 치워진 뒤다.

Handbook은 바로 이 관측 특성(Observation: short-term vs long-term)을 첫 번째 축으로 세우고, 지도에 무엇까지 담을 것인가 하는 재구성 수준(Reconstruction: 자세 추정, 장면 기하 복원, 시간에 따른 장면 이해)을 두 번째 축으로, 그리고 처리를 언제 끝내야 하는가 하는 시간 제약(Time: online vs offline)을 세 번째 축으로 삼아 동적 SLAM의 지형도를 그린다. Handbook의 설명대로라면, 움직이는 물체를 전부 outlier로 버리면서 실시간으로 카메라 자세만 건지는 전형적인 포즈 그래프 SLAM은 이 3차원 설계 공간의 한 귀퉁이에 놓인 특수 해법에 불과하다.

---

## 19.2 Short-term: 마스킹에서 multi-object SLAM으로

가장 손쉬운 대응은 움직이는 물체를 아예 못 본 척 지워 버리는 것이었다. Zaragoza 대학에서 박사과정을 밟던 Berta Bescos가 2018년 RA-L에 발표한 [DynaSLAM](https://arxiv.org/abs/1806.05620)은 ORB-SLAM2의 frontend 앞에 Mask R-CNN을 세워 사람이나 자동차처럼 움직일 가능성이 높은 물체를 마스크로 가리고, 그 안에서는 keypoint를 뽑지 않았다. 발상은 소박했지만 효과는 즉각적이어서, 사람이 걸어 다니는 TUM-RGBD의 walking sequence에서 ATE가 0.9~4.0 cm 수준까지 뚝 떨어졌다.

하지만 움직이는 대상을 무조건 지우면 정작 로봇이 피해 가거나 상호작용해야 할 주변 물체의 궤적 정보까지 함께 버리게 된다. UCL의 Martin Rünz는 Lourdes Agapito의 지도 아래 내놓은 [Co-Fusion(Rünz & Agapito, 2017)](https://arxiv.org/abs/1706.06629)과 [MaskFusion(Rünz et al., 2018)](https://arxiv.org/abs/1804.09194)에서 다른 길을 택했다. 움직이는 물체를 지우는 대신 객체마다 별도의 surfel 모델을 만들어 카메라 궤적과 각 객체의 궤적을 동시에 추적한 것이다. 한편 Heriot-Watt 대학의 Raluca Scona, TUM의 Mariano Jaimez, Daniel Cremers 등이 2018년 ICRA에 발표한 [StaticFusion](https://raluca-scona.github.io/docs/conference-papers/2018_icra_staticfusion.pdf)은 딥러닝 semantic segmentation에 기대지 않고도 카메라 움직임과 정적·동적 픽셀의 확률적 분할을 함께 최적화해 정적인 배경만 지도에 남기는 방법을 보였다.

그 다음 단계는 움직이는 객체들을 아예 정식 상태 변수로 승격시켜 하나의 그래프 안에서 함께 푸는 것이었다. Australian National University의 Jun Zhang 등이 낸 [VDO-SLAM(Zhang et al., 2020)](https://arxiv.org/abs/2005.11052)은 카메라 자세 $T_i^w \in SE(3)$와 동적 객체 $k$의 자세 $T_{k,i}^w \in SE(3)$를 같은 factor graph에 나란히 올려놓고, 객체의 운동이 갑자기 튀지 않도록 smooth motion factor로 묶었다. DynaSLAM을 냈던 Bescos 역시 2021년 [DynaSLAM II(Bescos et al., 2021)](https://arxiv.org/abs/2010.07820)로 돌아와 ORB-SLAM2를 기반으로 stereo·RGB-D 입력에서 객체 추적과 SLAM을 강하게 결합했고, CMU의 Yuheng Qiu는 2022년 ICRA의 [AirDOS](https://arxiv.org/abs/2109.09903)에서 딱딱한 강체 차량을 넘어 사람처럼 관절이 꺾이는 객체(articulated object)까지 그래프 안으로 끌어들였다.

관성 센서가 함께 있을 때는 시맨틱 마스크 없이도 동적 요소를 걸러 낼 수 있었다. KAIST의 Song·Lim·Lee·Myung이 2022년 RA-L에 발표한 [DynaVINS](https://arxiv.org/abs/2208.11500)는 semantic mask나 multi-object tracking을 전혀 쓰지 않고, IMU preintegration이 내놓은 자세 prior와 어긋나는 시각 관측의 가중치를 bundle adjustment에서 낮춰 동적 특징점이 추정을 흔들지 못하게 막았다. 같은 연구실이 2024년 RA-L에 낸 [DynaVINS++](https://arxiv.org/abs/2410.15373)는 이를 adaptive truncated least squares로 재설계해, 움직이는 물체 때문에 생긴 오대응이 IMU bias 추정을 오염시켜 필터가 발산하는 현상까지 차단했다.

최근에는 LiDAR와 대규모 transformer 쪽에서도 대응이 이어지고 있다. Handbook §15.2.3은 LiDAR 기반 moving object segmentation(MOS)의 사례로 Schmid 자신의 [Dynablox(Schmid et al., 2023)](https://arxiv.org/abs/2304.10049)를 꼽는다. 한편 2025년 나온 Wimbauer 등의 [AnyCam](https://arxiv.org/abs/2503.23282)은 동적 물체가 뒤섞인 일상 영상에서 transformer로 카메라 자세와 intrinsics를 단번에 추정한다.

> 🔗 **차용.** VDO-SLAM은 고전 graph SLAM의 정식화를 한 치도 벗어나지 않는다. 움직이는 자동차 한 대를 지도에 편입시키려면, 새로운 최적화 기법을 발명하는 대신 factor graph에 그 자동차의 자세 변수와 운동 제약 factor를 추가하기만 하면 된다.

---

## 19.3 Long-term: 시간을 가로지르는 지도

단기 움직임이 매 프레임의 추적 정확도를 위협한다면, 장기 변화는 로봇이 어제 만든 지도를 오늘 다시 꺼내 쓸 때 드러난다. 어제 창가에 있던 의자가 오늘 아침 복도 쪽으로 밀려 있는 식이다.

이 문제를 기억 구조로 푼 선구적 시스템이 캐나다 Sherbrooke 대학의 Mathieu Labbé가 Michaud의 지도로 개발한 [RTAB-Map](https://introlab.github.io/rtabmap/)이다. 인간의 기억 체계를 본떠 short-term, working, long-term memory의 세 층을 두고, 방문 빈도와 경과 시간에 따라 지도 노드를 옮겨 실었다. 최근 노드는 working memory에 남겨 즉시 조회하고, 드물게 찾는 노드는 long-term memory로 내렸다. Labbé는 2019년 JFR 논문에서 이 구조를 수개월·다중 세션에 걸친 대규모 SLAM으로 확장하는 방법을 정리했다.

누적된 LiDAR 지도에서 과거의 유령 흔적을 청소하는 접근도 나왔다. KAIST URL의 임형태·황성원·명현이 2021년 발표한 [ERASOR](https://arxiv.org/abs/2103.04316)는 지도 전체와 개별 스캔의 구역별 높이 분포를 맞대어 동적 점이 찍혀 있을 법한 구역을 짚어 낸 뒤, 각 구역의 지면을 추정해 지면 점은 남기고 움직였던 물체의 흔적만 도려냈다.

하지만 오래 쓰는 지도에서 가장 미묘한 함정은 따로 있다. Handbook §15.3이 거듭 강조하는 **absence of evidence vs evidence of absence**(아직 못 본 것과 정말로 없어진 것의 구별)다. 방에 들어섰는데 어제 보던 의자가 시야에 없다면, 의자가 치워진 것일까 아니면 책상 뒤에 가려 아직 못 본 것일까. 둘을 가리지 못하면 지도 청소 모듈은 멀쩡히 서 있는 물체를 지워 버리고, 변화 감지 모듈은 가려진 영역을 엉뚱한 변화로 오판한다. Schmid가 2022년 ICRA에 낸 [Panoptic Multi-TSDF](https://arxiv.org/abs/2109.10165)는 객체마다 독립된 TSDF submap을 꾸리고 현재 추적 중인 submap만 active 상태로 두어 관측을 합치되, 몇 프레임 동안 보이지 않는 submap은 inactive로 잠가 과거 기억이 함부로 지워지지 않게 방어했다. 이어 MIT Carlone 그룹에서 Schmid 등이 2024년 발표한 [Khronos](https://arxiv.org/abs/2402.13817)는 아예 spatio-temporal metric-semantic SLAM을 정식화해, loop closure가 일어난 뒤 change detection을 다시 돌려 각 물체가 언제 나타나고 언제 이동했는지까지 복원함으로써 3D 지도에 시간 축을 심었다.

2026년에도 이 흐름은 계속 진화하고 있다. KAIST의 Jang과 Myung 등이 공개한 [Chamelion](https://arxiv.org/abs/2602.08189)은 공사 현장이나 배치가 수시로 바뀌는 실내에서 dual-head 네트워크로 온라인 변화 감지와 장기 지도 유지를 동시에 수행한다. 시간 차이를 두고 정확히 정합된 실세계 변화 데이터를 모으기가 워낙 까다롭다 보니, 다른 장면의 요소를 이식해 구조 변화를 합성하는 데이터 증강 기법을 고안해야 했다.

한편 불규칙한 사건뿐 아니라 주기성과 흐름을 모델링하려는 줄기도 이어져 왔다. 영국 Lincoln 대학의 Tomáš Krajník과 Tom Duckett 등이 2014년부터 발전시킨 **frequency maps**(FreMEn)는 문이 열리고 닫히거나 주차장이 차고 비는 주기적 변화를 Fourier 분석으로 포착한다. 스웨덴 Örebro 대학의 Tomasz Kucner, Martin Magnusson, Achim Lilienthal이 연구한 Maps of Dynamics(MoD)는 사람들이 주로 걷는 동선 같은 *typical motion pattern*을 아예 지도의 속성으로 기록한다. 2022년 arXiv에 올라온 [Changing-SLAM(Soares et al.)](https://arxiv.org/abs/2209.10710)은 ORB-SLAM3 위에서 본래 움직이는 물체의 특징점은 걸러 내고, 잠재적으로 움직일 수 있는 물체는 Extended Kalman Filter로 따로 추적하면서 장기 변화는 Bayesian filter와 장기 data association으로 엮었다.

> 🔗 **차용.** Panoptic Multi-TSDF는 여러 TSDF 블록을 따로 관리하는 volumetric submapping 기법과, 객체 하나마다 독립된 3D 모델을 부여하는 object-centric mapping을 하나로 합쳐 장기 지도의 훼손을 막았다.

---

## 19.4 Deformable: 형상이 변할 때

물체가 강체(rigid body)로서 통째로 자리를 옮기는 수준을 넘어, 배경의 표면 자체가 고무나 피부처럼 휘고 늘어나는 세계도 있다. University of Washington의 Newcombe, Fox, Seitz는 2015년 CVPR 최우수 논문상을 받은 [DynamicFusion](https://grail.cs.washington.edu/projects/dynamicfusion/)으로 비강체(non-rigid) SLAM의 물꼬를 텄다. DynamicFusion은 KinectFusion의 정적 TSDF 공간 위에 희소한 embedded deformation graph를 띄우고, 각 노드의 회전과 이동을 매 프레임 최적화함으로써 카메라 앞에서 표정을 바꾸는 얼굴이나 몸통을 실시간으로 복원했다. 곧이어 Erlangen-Nürnberg 대학의 Matthias Innmann 등이 2016년 [VolumeDeform](https://arxiv.org/abs/1603.08161)으로 색상 단서를 보태 정밀도를 높였고, 2017년 Miroslava Slavcheva의 [KillingFusion](https://campar.in.tum.de/pub/slavcheva2017cvpr/slavcheva2017cvpr.pdf)은 Killing vector field 정칙화를 도입해 변형이 매끄럽고 거의 등거리(isometric)를 유지하게 하면서도 손바닥을 폈다 오므리는 식의 위상 변화까지 허용했다. 2018년 MIT에서 Tedrake의 지도를 받은 Wei Gao가 발표한 [SurfelWarp](https://arxiv.org/abs/1904.13073)는 TSDF 대신 surfel 표현으로 비강체 추적을 이어 갔다.

단안 RGB 카메라만으로 이 난제에 매달린 연구실은 Zaragoza의 Civera와 Montiel 그룹이었다. Montiel의 박사과정 학생 José Lamarca가 2021년 IEEE TRO에 실은 [DefSLAM](https://arxiv.org/abs/1908.08918)은 ORB 특징 대응을 바탕으로 매 프레임 Shape-from-Template(SfT)로 변형을 추적하고 keyframe마다 isometric NRSfM으로 기준 template를 갱신했다. 같은 그룹의 Juan J. Gómez Rodríguez가 2023년 arXiv에 공개하고 2024년 IEEE TRO에 게재한 [NR-SLAM](https://arxiv.org/abs/2308.04036)은 기존 메시 기반 방법이 평면 토폴로지를 가정해 관 모양 장기나 구멍 난 표면을 다루지 못한다는 한계를 짚고, dynamic deformable graph와 visco-elastic 시간 정칙화를 결합해 임의의 표면 토폴로지를 수용했다. Handbook §15.4.3은 바로 이 계보를 단안 카메라용 deformable SLAM의 표준 파이프라인으로 정리한다.

이 기술이 가장 절실하게 향하는 현장은 외과 수술실이다. University of Technology Sydney의 Jingwei Song 등이 2018년 RA-L에 발표한 [MIS-SLAM](https://doi.org/10.1109/LRA.2018.2856519)은 stereo 복강경 영상으로 수술 중 장기의 박동과 변형을 추적했다. 하버드 의대 Brigham and Women's Hospital의 Haoyin Zhou와 Jagadeesan Jayender가 만든 EMDQ(Expectation Maximization and Dual Quaternion)는 SURF 특징점으로 내시경 카메라의 움직임과 조직 변형을 동시에 풀어 최소침습수술(minimally invasive surgery, MIS) 내비게이션을 겨냥했다.

하지만 변형 장면을 다루는 순간 기하학적으로 피할 수 없는 근본적 모호성이 고개를 든다. Handbook §15.4.1이 강조하는 **Floating Map Ambiguity**다. 일반적인 deformable SLAM은 관측된 변형 가운데 강체 변환으로 설명되는 성분은 모두 카메라 궤적 탓으로 돌린다. 그러나 영상 속에서 무언가 30 cm 옆으로 비켜 났을 때, 손이 30 cm 움직인 것인지 카메라가 반대 방향으로 30 cm 움직인 것인지는 영상만으로는 원리적으로 가를 수 없는 ill-posed 문제다. Handbook이 지적하듯 결국 변형하는 물체의 변위에 물리적 prior를 걸어 둘을 억지로라도 떼어 놓아야 하며, DefSLAM의 isometric prior와 NR-SLAM의 visco-elastic prior가 바로 그 역할을 맡는다.

> 📜 **예언 vs 실제.** Newcombe는 DynamicFusion(2015)의 Limitations and Discussion에서, 손을 주먹 쥔 채 재구성을 시작했다가 손가락을 펴는 것처럼 닫힌 위상에서 열린 위상으로 급격히 바뀌는 장면을 다루기 어렵다고 적었다. 또 KinectFusion처럼 고정 TSDF 메모리가 공간 크기를 가두고, 장면이 넓어질수록 가림이 늘어 거대해지는 warp field를 풀기 어려워진다는 점도 짚었다. 그중 위상 변화의 한계는 2년 뒤인 2017년 KillingFusion이 정면으로 돌파했다.

> 🔗 **차용.** DynamicFusion의 embedded deformation graph는 컴퓨터 그래픽스에서 Sumner, Schmid, Pauly가 2007년 메시 애니메이션용으로 제안한 ED graph를 가져온 것이다. 원래 3D 캐릭터의 표면을 구부리던 희소 제어 그래프가 실시간 비강체 SLAM의 최적화 변수로 변신했다.

---

## 아직 열린 문제

**Absence vs evidence of absence.** 지도에서 사물이 진짜 사라졌는지 아니면 다른 물체 뒤에 가려 잠시 안 보이는 것인지를 구별하는 문제는 Handbook §15.3이 거듭 강조하는 핵심 난제다. Panoptic Multi-TSDF와 Khronos가 물체 단위 submap과 사후 변화 감지로 실마리를 잡았지만, 가림이 빈번한 일상 공간에서 완결된 해법까지는 거리가 남았다.

**Floating Map Ambiguity.** deformable SLAM은 카메라의 움직임과 변형 물체의 강체 이동을 가르는 일을 여전히 isometric·visco-elastic 같은 경험적 prior에 기대어 우회하고 있다. Handbook §15.5는 카메라 궤적과 지도 변형을 둘 다 정확히 복원할 수 있는 이론적 조건이 무엇인지, 두 강체 운동을 분명히 분리할 수 있는지, 그리고 추정의 불확실성을 정량화할 수 있는지를 아직 열려 있는 근본 질문으로 남겼다.

**Online deformable SLAM.** 계산 속도 역시 현실의 벽에 걸려 있다. DefSLAM은 10×10 노드 template 기준으로 deformable tracking thread에 프레임당 약 50 ms, deformable mapping thread에 약 400 ms가 걸렸다(i7-4700HQ CPU). C++로 새로 짠 NR-SLAM 역시 CPU 전용 구현이었고, 처리 시간은 따로 보고하지 않았다. Khronos가 RGB-D 입력에서 단기 움직임과 장기 배치를 online으로 함께 다뤘지만, 그 대상은 강체 이동과 배치 변화였지 장기 조직처럼 표면 자체가 늘어나고 찌그러지는 변형은 아니었다. Handbook §15.5는 동적 객체 추적을 상태 변수에 편입할수록 계산 비용이 급증하며, NeRF나 3DGS 같은 최신 미분 가능 표현에서도 동적 장면을 실시간 SLAM에 올리기에는 메모리와 연산 부담이 여전하다고 짚는다.

**의료 MIS의 실세계 격차.** MIS-SLAM과 NR-SLAM은 Hamlyn 데이터셋의 생체(in vivo) 복강경 영상으로 검증됐고, NR-SLAM은 실제 대장내시경 영상에서도 정성 평가를 거쳤다. 그럼에도 NR-SLAM 논문 스스로 깊이 불연속과 격렬한 변형이 겹치는 장면에서는 지도를 안정적으로 넓히기 어렵다고 인정했으며, 수술 도구가 시야를 가리는 시퀀스에서는 도구를 따로 분할해 가리는 SD-DefSLAM보다 오차가 컸다. 2024년에는 EndoGS처럼 3D Gaussian을 수술 장면에 들이민 시도도 나왔지만, 실제 수술실의 연기와 출혈, 도구 가림을 견디는 실시간 변형 SLAM은 여전히 진행 중인 과제다.

이처럼 정적 세계라는 가장 오래된 가정에 균열이 가는 동안, 학습 기반 SLAM의 다른 한쪽에서는 아예 카메라 모델도, 단계적 매칭 파이프라인도 건너뛰고 이미지에서 곧장 3D 기하를 쏟아 내는 거대 모델들이 등장하고 있었다.
