# Ch.15b — 정적 세계 가정 밖: Dynamic과 Deformable SLAM

Jorge Fuentes-Pacheco는 동료 Ruiz-Ascencio, Rendón-Mancha와 함께 쓴 [*Visual simultaneous localization and mapping: a survey*](https://link.springer.com/article/10.1007/s10462-012-9365-8)를 2012년 *Artificial Intelligence Review*에 온라인으로 공개했고, 이 글은 2015년 43권에 실렸다. 초록은 동적이고 복잡한 대규모 환경의 visual SLAM을 아직 활발히 연구 중인 분야로 꼽았고, 서베이의 마지막 절 제목은 "Dynamic and Deformable Environments"였다. 그 전에도 움직이는 물체를 다룬 논문은 있었다. 2026년 4월 21일자 *The SLAM Handbook* 원고는 이 주제에 한 장 37쪽을 썼다. 저자는 Lukas Schmid, José María Martínez Montiel, Shoudong Huang, Daniel Cremers, José Neira, Javier Civera 여섯 명이다. 이 장은 SLAM이 정적 환경을 가정해 왔다고 짚고, 그 가정이 맞지 않는 예로 도시를 달리는 자율주행차, 사람이 오가는 집 안의 서비스 로봇, 장면 전체가 변형되는 몸속의 수술 로봇을 든다.

---

## 15b.1 세 개의 축

Schmid et al.은 Handbook 15장 §15.1에서 Looper et al.(2023)의 정의를 따라 동적 효과를 나눈다. 동적인지 정적인지는 *관측*에 따라 갈린다. 같은 물리적 운동이 어떤 로봇에게는 short-term dynamic(관측하는 동안 보이는 움직임)이 되고, 다른 로봇에게는 long-term dynamic(관측 사이에 쌓인 변화)이 된다. 이를 정하는 것은 장면의 변화율 $\text{Dyn}$과 로봇의 관측률 $\text{Obs}$의 관계다. $\text{Dyn} \ll \text{Obs}$이면 로봇은 움직임을 프레임마다 보고, $\text{Dyn} \gg \text{Obs}$이면 다음에 왔을 때 장면이 이미 바뀌어 있다.

Handbook은 이 관측 특성(Observation)을 첫 번째 축으로 두고 short-term과 long-term을 나눈다. 두 번째 축인 재구성 수준(Reconstruction)은 pose만 추정할지, scene geometry까지 복원할지, 시간에 따른 장면 이해까지 갈지를 정한다. 세 번째 축인 시간 제약(Time)은 online으로 처리할지 offline으로 처리할지를 정한다. Handbook의 그림 설명에 따르면, 동적인 부분을 outlier로 버리면서 실시간으로 자세를 추정하는 일반적인 pose-graph SLAM은 이 공간의 한 점에 해당한다.

---

## 15b.2 Short-term: 마스킹에서 multi-object SLAM으로

Zaragoza에서 박사과정을 하던 Berta Bescos는 2018년 [DynaSLAM](https://arxiv.org/abs/1806.05620)을 RA-L에 발표했다. ORB-SLAM2의 frontend에 Mask R-CNN을 넣어 사람과 자동차처럼 움직일 수 있는 물체를 미리 가리고, 가린 영역에서는 keypoint를 뽑지 않았다. 방법은 간단했지만 TUM-RGBD의 walking sequence에서 ATE가 0.9~4.0 cm로 내려갔다.

UCL의 Martin Rünz는 움직이는 물체를 지우지 않고 따로 추적했다. 그는 Lourdes Agapito의 지도로 [Co-Fusion(Rünz & Agapito, 2017)](https://arxiv.org/abs/1706.06629)과 이듬해 [MaskFusion(Rünz et al., 2018)](https://arxiv.org/abs/1804.09194)을 연달아 내놓았다. 두 시스템은 객체마다 surfel 모델을 따로 두고 카메라 궤적과 객체 궤적을 함께 추정했다. Heriot-Watt의 Raluca Scona, TUM의 Mariano Jaimez와 Daniel Cremers 등이 2018년 ICRA에 낸 [StaticFusion](https://raluca-scona.github.io/docs/conference-papers/2018_icra_staticfusion.pdf)은 semantic segmentation을 쓰지 않았다. 카메라 운동과 정적·동적 확률 분할을 함께 추정하고, 정적인 부분만 지도에 합쳤다.

다음 단계에서는 움직이는 객체를 상태 변수에 넣어 함께 추정했다. Australian National University의 Jun Zhang 등이 낸 [VDO-SLAM(Zhang et al., 2020)](https://arxiv.org/abs/2005.11052)은 동적 객체마다 factor graph의 변수를 두었다. 카메라 자세 $T_i^w \in SE(3)$와 객체 $k$의 자세 $T_{k,i}^w \in SE(3)$가 같은 그래프에 들어가고, smooth motion factor가 객체의 운동이 갑자기 바뀌지 않도록 제약했다. Zaragoza의 Bescos는 2021년 [DynaSLAM II(Bescos et al., 2021)](https://arxiv.org/abs/2010.07820)에서 ORB-SLAM2를 바탕으로, stereo와 RGB-D 입력에서 객체 추적과 SLAM을 강하게 결합했다. CMU의 Yuheng Qiu가 2022년 ICRA에 발표한 [AirDOS](https://arxiv.org/abs/2109.09903)는 사람처럼 관절이 있는 객체(articulated object)까지 다뤘다.

> 🔗 **차용.** VDO-SLAM은 Ch.6의 graph SLAM 정식화를 그대로 썼다. 움직이는 자동차 하나를 지도에 넣으려면 factor graph에 변수와 factor를 더하면 된다.

관성 센서를 쓰는 쪽에서는 KAIST의 Song·Lim·Lee·Myung이 2022년 RA-L에 [DynaVINS](https://arxiv.org/abs/2208.11500)를 발표했다. DynaVINS는 semantic mask와 multi-object tracking을 쓰지 않았다. IMU preintegration으로 얻은 자세 prior와 어긋나는 관측의 가중치를 bundle adjustment에서 낮춰, 동적 특징이 상태 추정에 끼어들지 못하게 했다. 같은 그룹이 2024년 RA-L에 낸 [DynaVINS++](https://arxiv.org/abs/2410.15373)는 이 과정을 adaptive truncated least squares로 다시 짰다. 동적 물체 때문에 생긴 잘못된 대응이 IMU bias 추정으로 번져 추정이 발산하는 문제도 막았다.

Handbook §15.2.3은 LiDAR 기반 moving object segmentation(MOS, 움직이는 점을 골라내는 일)의 예로 Schmid 자신의 [Dynablox(Schmid et al., 2023)](https://arxiv.org/abs/2304.10049)를 든다. 2025년의 [AnyCam](https://arxiv.org/abs/2503.23282)(Wimbauer et al.)은 transformer로, 동적 물체가 있는 일상 영상에서 카메라 자세와 intrinsics를 한꺼번에 추정한다.

---

## 15b.3 Long-term: 시간을 가로지르는 지도

long-term 변화는 방문과 방문 사이에 생긴다. 어제 본 의자가 오늘은 옆으로 밀려 있는 경우가 그렇다.

Sherbrooke의 Mathieu Labbé가 Michaud의 지도로 개발한 [RTAB-Map](https://introlab.github.io/rtabmap/)은 사람의 기억 모델을 본떴다. short-term, working, long-term memory 계층을 두고 시간과 관측 빈도에 따라 노드를 옮긴다. 최근 노드는 working memory에 두고, 드물게 방문하는 노드는 long-term memory로 보낸다. Labbé는 2019년 JFR 논문에서 이 구조를 여러 세션에 걸친 대규모 SLAM에 쓰는 방법을 정리했다.

KAIST URL의 임형태·황성원·명현이 2021년 발표한 [ERASOR](https://arxiv.org/abs/2103.04316)는 누적된 LiDAR 지도에서 움직이는 물체가 남긴 흔적을 지웠다. 지도와 개별 스캔의 영역별 높이 범위를 비교해 동적 점이 있을 법한 구역을 고르고, 구역마다 지면을 추정해 지면 점과 동적 물체의 점을 구별했다.

Handbook §15.3은 **absence of evidence vs evidence of absence**(못 본 것과 없는 것)를 거듭 구별한다. 의자가 없어졌는지 내가 못 봤는지 가리지 못하면, map cleaning은 남아 있는 객체를 지우고 change detection은 가려진 영역을 변화로 잘못 판정한다. Schmid가 2022년 ICRA에 발표한 [Panoptic Multi-TSDF](https://arxiv.org/abs/2109.10165)는 이 문제를 submap 구조로 다뤘다. 객체마다 submap을 따로 두고, 지금 추적 중인 submap만 active로 두어 새 데이터를 합쳤다. 몇 프레임 동안 검출되지 않은 submap은 inactive로 돌려, 과거 데이터가 망가지지 않게 했다. MIT Carlone 그룹에서 Schmid 등이 2024년 낸 [Khronos](https://arxiv.org/abs/2402.13817)는 spatio-temporal metric-semantic SLAM 문제를 정의했다. loop closure 뒤에 change detection을 다시 돌려 각 객체가 언제 바뀌었는지까지 추정하므로, metric-semantic 지도에 시간 축이 더해진다.

> 🔗 **차용.** Panoptic Multi-TSDF는 여러 TSDF submap을 함께 관리하는 volumetric submapping과, 객체마다 모델을 두는 object-centric mapping을 합쳤다.

KAIST의 Jang과 Myung 등이 2026년 공개한 [Chamelion](https://arxiv.org/abs/2602.08189)은 공사 현장이나 배치가 자주 바뀌는 실내 같은 환경에서, dual-head 네트워크로 online change detection과 장기 지도 유지를 한다. 시간에 따라 정합한 실제 데이터를 모으기 어려워서, 다른 장면의 요소를 가져와 구조 변화를 합성하는 data augmentation을 썼다.

반복되는 변화를 다룬 연구도 있다. 영국 Lincoln 대학의 Tomáš Krajník, Tom Duckett 등이 2014년부터 내놓은 **frequency maps**(FreMEn)는 주기적으로 되풀이되는 환경 변화를 Fourier 분석으로 모델링한다. 스웨덴 Örebro 대학의 Tomasz Kucner, Martin Magnusson, Achim Lilienthal이 연구한 Maps of Dynamics(MoD)는 사람 흐름 같은 *typical motion pattern*을 지도에 직접 담는다. 2022년 arXiv에 공개된 [Changing-SLAM(Soares et al.)](https://arxiv.org/abs/2209.10710)은 ORB-SLAM3를 바탕으로, 미리 동적이라고 아는 물체의 특징점은 걸러내고 움직일 수 있는 물체는 Extended Kalman Filter로 추적하며, 장기 변화는 Bayesian filter와 장기 data association으로 다뤘다.

---

## 15b.4 Deformable: 형상이 변할 때

배경 자체가 변형되는 경우도 있다.

출발점은 2015년 CVPR best paper로, University of Washington의 Newcombe, Fox, Seitz가 발표한 [DynamicFusion](https://grail.cs.washington.edu/projects/dynamicfusion/)이었다. KinectFusion의 canonical TSDF에 embedded deformation graph를 얹어, 카메라 앞에서 변형되는 얼굴이나 몸통을 실시간으로 비강체(non-rigid) 복원했다. 노드마다 회전과 이동을 둔 변형 그래프를 매 프레임 최적화했다. 이어 Erlangen-Nürnberg 대학의 Matthias Innmann 등이 2016년 [VolumeDeform](https://arxiv.org/abs/1603.08161)으로 색 정보를 더했다. 2017년 Miroslava Slavcheva가 낸 [KillingFusion](https://campar.in.tum.de/pub/slavcheva2017cvpr/slavcheva2017cvpr.pdf)은 Killing vector field 정칙화로 변형이 매끄럽고 거의 등거리(isometric)가 되게 했고, 위상 변화까지 허용했다. MIT에서 Tedrake의 지도를 받은 Wei Gao가 2018년에 발표한 [SurfelWarp](https://arxiv.org/abs/1904.13073)는 TSDF 대신 surfel로 지도를 표현했다.

> 🔗 **차용.** DynamicFusion의 embedded deformation graph는 컴퓨터 그래픽스에서 Sumner, Schmid, Pauly가 2007년 발표한 ED graph를 가져온 것이다. 메시 변형에 쓰던 희소 제어 그래프가 실시간 비강체 재구성의 변수가 됐다.

Zaragoza의 Civera와 Montiel은 이 문제를 꾸준히 다뤘다. Montiel의 지도로 박사과정을 한 José Lamarca는 2021년 [DefSLAM](https://arxiv.org/abs/1908.08918)을 IEEE TRO에 발표했다. DefSLAM은 프레임마다 Shape-from-Template(SfT)로 tracking하고, keyframe마다 isometric NRSfM으로 template를 다시 계산했다. ORB 특징으로 대응을 찾았다. 같은 그룹의 Juan J. Gómez Rodríguez는 2023년 arXiv에 [NR-SLAM](https://arxiv.org/abs/2308.04036)을 공개했고, 이 논문은 2024년 IEEE TRO에 실렸다. NR-SLAM 논문은 기존 mesh 기반 방법이 평면 토폴로지를 가정해 관이나 구멍 난 표면을 다루지 못한다고 지적하고, dynamic deformable graph로 임의의 표면 토폴로지를 다뤘다. visco-elastic 변형 모델로 시간 방향 정칙화도 넣었다. Handbook §15.4.3은 이 연구들을 단안 카메라용 deformable SLAM 파이프라인으로 정리한다.

University of Technology Sydney의 Jingwei Song 등이 2018년 RA-L에 낸 [MIS-SLAM](https://doi.org/10.1109/LRA.2018.2856519)은 stereo 내시경 영상으로 수술 중 장기의 변형을 추적했다. Brigham and Women's Hospital(Harvard)의 Haoyin Zhou와 Jagadeesan Jayender가 개발한 EMDQ(Expectation Maximization and Dual Quaternion) 알고리즘은 SURF 특징으로 카메라 운동과 조직 변형을 함께 추정했다. 목표는 최소침습수술(minimally invasive surgery, MIS) 중의 수술 내비게이션이다.

Handbook §15.4.1은 **Floating Map Ambiguity**를 강조한다. deformable SLAM은 관측된 변형 가운데 rigid transformation으로 설명되는 부분을 모두 카메라 궤적으로 돌린다고 가정한다. 그러나 운동의 강체 성분과 비강체 성분을 가르는 일은 ill-posed(해가 하나로 정해지지 않는) 문제다. 손이 30 cm 움직였는지 카메라가 30 cm 움직였는지는 영상만으로 가를 수 없다. Handbook은 변형하는 물체의 변위에 prior를 두어야 이 둘을 가를 수 있다고 쓴다. DefSLAM의 isometric prior와 NR-SLAM의 visco-elastic prior가 그런 prior다.

> 📜 **예언 vs 실제.** Newcombe는 DynamicFusion(2015)의 Limitations and Discussion에서, 닫힌 위상에서 열린 위상으로 빠르게 바뀌는 장면(손을 쥔 채 재구성을 시작해 손을 펴는 경우)을 다루기 어렵다고 적었다. 장면을 더 키우기 어려운 이유로는 KinectFusion처럼 TSDF 메모리가 기하 범위를 제한한다는 점과, 장면이 커질수록 카메라에 가려지는 부분이 늘어 커지는 warp field를 추정하기 어려워진다는 점을 들었다. 위상 변화는 2017년 KillingFusion이 다뤘다.

---

## 🧭 아직 열린 것

**Absence vs evidence of absence.** 지도에서 객체가 사라졌는지, 가려서 못 봤는지를 구별하는 일은 Handbook §15.3이 거듭 짚은 문제다. §15b.3에서 본 Panoptic Multi-TSDF와 Khronos가 이 문제를 다뤘다.

**Floating Map Ambiguity.** deformable SLAM에서 카메라 운동과 객체 운동을 나누는 문제는 isometric·visco-elastic prior로 피해 가고 있다. Handbook §15.5는 카메라 궤적과 지도의 변형을 둘 다 정확히 추정할 수 있는 조건, 변형하는 물체의 강체 운동과 카메라의 강체 운동을 분명히 가를 수 있는지, 결과의 불확실성을 정확히 추정할 수 있는지를 열린 질문으로 남겼다.

**Online deformable SLAM.** DefSLAM 논문은 10×10 노드 template 기준으로 deformable tracking thread가 프레임당 약 50 ms, deformable mapping thread가 약 400 ms 걸린다고 보고했다(i7-4700HQ CPU). NR-SLAM 논문은 C++로 새로 구현해 CPU에서만 돌린다고 적었고, 처리 시간은 따로 보고하지 않았다. Khronos는 RGB-D 입력으로 짧은 시간의 움직임과 긴 시간의 변화를 online으로 함께 다뤘다. 다만 대상은 사람과 물체의 움직임, 물체 배치의 변화였고, 조직처럼 형태가 바뀌는 표면의 변형은 추정하지 않았다. Handbook §15.5는 동적 객체 추적을 상태 추정에 넣으면 계산 비용이 늘어난다고 적었다. NeRF와 3DGS 같은 미분 가능한 표현에 대해서도, 동적 장면의 실시간 SLAM에 쓰려면 메모리와 계산 비용 문제가 남아 있다고 정리했다.

**의료 MIS의 실세계 격차.** MIS-SLAM과 NR-SLAM은 Hamlyn 데이터셋의 생체(in vivo) 복강경 영상으로 평가했고, NR-SLAM은 실제 대장내시경 영상에서도 정성적으로 시험했다. NR-SLAM 논문은 깊이 불연속과 강한 변형이 함께 나타나는 장면에서는 지도를 만들고 넓히기 어렵다고 한계를 밝혔다. 수술 도구가 시야를 가리는 시퀀스에서는 도구를 분할해 가리는 SD-DefSLAM보다 오차가 컸다. 2024년에는 EndoGS 같은 Gaussian 기반 시도도 나왔다.
