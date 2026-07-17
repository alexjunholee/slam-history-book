# Ch.15b — 정적 세계 가정이 무너지는 자리: Dynamic과 Deformable SLAM

2015년 Javier Fuentes-Pacheco는 동료 Ruiz-Ascencio, Rendón-Mancha와 함께 [*Visual simultaneous localization and mapping: a survey*](https://link.springer.com/article/10.1007/s10462-012-9365-8)를 *Artificial Intelligence Review*에 발표했다. 그 서베이의 마지막 절이 "Dynamic and Deformable Environments"였다. 그 이전에도 움직이는 물체를 다룬 논문은 있었지만 대부분 RANSAC이 걸러내야 할 outlier로 취급했다. 이 서베이는 동적 환경을 독립 주제로 묶어 다룬 이른 문헌이었다. 10년이 지난 2025년, *The SLAM Handbook*은 이 주제에 37페이지를 할애했다. 저자는 Lukas Schmid, José María Martínez Montiel, Shoudong Huang, Daniel Cremers, José Neira, Javier Civera 여섯 명이다. 정적 세계는 SLAM의 출발점이었지만, 거리의 자율주행차와 집안의 서비스 로봇, 장기의 내시경은 모두 그 가정 밖에서 작동해야 했다.

---

## 15b.1 세 개의 축

Schmid et al.이 Handbook Ch.15 §15.1에서 그린 프레임은 이전의 "dynamic SLAM" 정의를 다시 쓴다. 환경이 동적인지 정적인지는 환경의 속성이 아니라 *관측의 속성*이다. 같은 물리적 운동이 한 로봇에게는 short-term dynamic, 다른 로봇에게는 long-term dynamic이 된다. 관측률 $\text{Obs}$와 변화율 $\text{Dyn}$의 비율이 결정한다. $\text{Dyn} \ll \text{Obs}$이면 프레임 사이에서 움직임이 보이고, $\text{Dyn} \gg \text{Obs}$이면 방문 사이에서 장면이 변해 있다.

Observation axis는 short-term과 long-term을 가른다. Reconstruction axis는 pose만 추정할지, scene geometry까지 복원할지, 4D spatio-temporal 이해까지 갈지를 정한다. Time axis는 online과 offline을 가른다. 이전 서술은 "동적 객체를 어떻게 제거할 것인가"라는 단일 질문으로 필드를 압축했는데, 이 과제는 3축 분류 공간의 한 구석에 지나지 않는다. 서로 다른 영역을 연구한 사람들이 같은 단어를 서로 다른 문제에 써온 이유다.

---

## 15b.2 Short-term: 마스킹에서 multi-object SLAM으로

첫 해법은 단순했다. 움직이는 것을 지웠다.

Zaragoza에서 박사과정을 하던 Berta Bescos는 2018년 [DynaSLAM](https://arxiv.org/abs/1806.05620)을 RA-L에 발표했다. ORB-SLAM2의 frontend에 Mask R-CNN을 끼워 넣어 사람·자동차를 사전 마스킹하는 시스템이었다. 마스킹된 영역은 keypoint 추출에서 제외됐다. 간단했지만 동작했다. TUM-RGBD의 walking sequence에서 ATE가 한 자릿수 cm로 내려갔다.

같은 시기 UCL의 Martin Rünz는 다른 선택을 했다. 움직이는 물체를 지우지 말고 따로 추적하자. Lourdes Agapito 지도하에 [Co-Fusion(Rünz & Agapito, 2017)](https://arxiv.org/abs/1706.06629)과 이듬해 [MaskFusion(Rünz et al., 2018)](https://arxiv.org/abs/1804.09194)을 연달아 내놓았다. 각 객체에 독립된 surfel 모델을 할당해, 카메라 궤적과 객체 궤적을 동시에 추정했다. Edinburgh의 Raluca Scona와 Imperial의 Stefan Leutenegger가 2018년 ICRA에 낸 [StaticFusion](https://arxiv.org/abs/1806.05628)은 또 다른 경로였다. Semantic segmentation 없이 residual clustering만으로 dynamic region을 분리했다. Segmentation 오류에 의존하지 않는 방향이다.

움직이는 객체를 state에 포함시켜 함께 추정했다. QUT의 Jun Zhang이 이끈 [VDO-SLAM(Zhang et al., 2020)](https://arxiv.org/abs/2005.11052)은 각 동적 객체를 factor graph의 변수로 올렸다. 카메라 포즈 $T_i^w \in SE(3)$와 객체 $k$의 포즈 $T_{k,i}^w \in SE(3)$가 같은 그래프에 공존했다. Constant-velocity factor가 객체의 선속도·각속도에 연속성 제약을 걸었다. SE(3)과 객체 SE(3)의 product manifold 위에서 joint optimization이 돌아갔다. Zaragoza의 Bescos는 2021년 [DynaSLAM II(Bescos et al., 2021)](https://arxiv.org/abs/2010.07820)에서 ORB-SLAM2 기반으로 같은 아이디어를 구현했다. CMU의 Yuheng Qiu가 2022년 RA-L에 발표한 [AirDOS](https://arxiv.org/abs/2109.09903)는 인간처럼 관절이 있는 객체까지 articulated body로 확장했다.

> 🔗 **차용.** VDO-SLAM의 factor graph 확장은 Ch.6 graph SLAM에서 Dellaert와 Kaess가 세운 iSAM 전통을 직접 계승한다. 변수를 하나 늘리고 factor를 하나 더 다는 것이, dynamic SLAM에서는 움직이는 자동차 하나를 지도에 올리는 일로 바뀌었다.

관성 쪽에서는 KAIST URL의 Song·Lim·Lee·Myung이 2022년 RA-L에 [DynaVINS](https://arxiv.org/abs/2208.11500)를 발표했다. DynaVINS는 semantic mask도 multi-object tracking도 쓰지 않았다. IMU preintegration이 준 pose prior와 어긋나는 관측은 bundle adjustment에서 factor weight를 낮추는 식으로, 동적 특징이 joint state로 새어 들어가는 경로를 끊었다. 같은 그룹이 2024년 RA-L에 낸 [DynaVINS++](https://arxiv.org/abs/2410.15373)는 이 아이디어를 adaptive truncated least squares로 다시 짜, dynamic feature가 IMU bias 추정으로 역전파되며 발산하는 실패 양상까지 잡았다.

Handbook은 이 계보를 §15.2.3 "Dense Dynamic SLAM"으로 정리하면서 Schmid 본인의 [Dynablox(Schmid et al., 2023)](https://arxiv.org/abs/2304.10049)를 LiDAR MOS의 현재형으로 배치한다. 2025년의 [AnyCam](https://arxiv.org/abs/2503.23282)은 transformer 기반으로 일상 영상에서 직접 4D를 뽑는다. Rünz가 2017년 문을 연 "simultaneous tracking + reconstruction" 계통의 2025년판이다.

---

## 15b.3 Long-term: 시간을 가로지르는 지도

Short-term이 프레임 사이의 운동이라면, long-term은 방문 사이의 변화다. 어제 본 의자가 오늘은 옆으로 밀려 있다. 이 문제는 다른 계보에서 자랐다.

Sherbrooke의 Mathieu Labbé가 Michaud 지도 아래 2013년부터 개발한 [RTAB-Map](https://introlab.github.io/rtabmap/)은 인간 기억 모델에서 직접 빌려왔다. short-term, working, long-term memory의 계층을 두고, 시간과 관측 빈도에 따라 노드를 옮겼다. 한 세션 안에서는 작동 메모리에 남고, 자주 방문하지 않으면 장기 메모리로 내려가고, 의미가 없어지면 폐기되는 구조다. 2019년 JFR 논문에서 Labbé는 이 구조가 다중 세션 SLAM에서 어떻게 스케일하는지 정리했다. 한국과학기술원 김아영 팀의 임현준이 2021년 발표한 [ERASOR](https://arxiv.org/abs/2103.04316)는 지도를 깨끗이 만드는 문제를 scene differencing으로 풀었다. 같은 장소를 두 번 지나갔을 때 사라진 점을 찾아낸다.

Handbook이 §15.3 전체를 통과하는 프레임 하나가 있다. **absence of evidence vs evidence of absence**. 의자가 없는 것인지, 내가 못 본 것인지를 구별해야 한다. 이 구분이 빠지면 map cleaning은 정당한 객체를 지우고, change detection은 가려진 영역을 잘못 판정한다. Schmid가 2022년 RA-L에 발표한 [Panoptic Multi-TSDF](https://arxiv.org/abs/2109.10165)는 이 문제를 submap 구조로 풀었다. 각 객체를 독립 submap으로 관리하고, local consistency 하에서 active와 inactive를 구분했다. 같은 그룹이 2024년 낸 [Khronos](https://arxiv.org/abs/2402.13817)는 graduated non-convexity로 association을 견고화하고, loop closure 이후에도 deformable geometric change detection을 돌려, 각 객체의 변화 시점까지 추정한다. Metric-semantic 지도가 4D spatio-temporal 지도로 바뀐다.

> 🔗 **차용.** Panoptic Multi-TSDF의 submap 구조는 Ch.7 ORB-SLAM의 Atlas가 세운 다중 지도 관리 구조를 다른 재료로 다시 짠 것이다. keyframe submap이 panoptic object submap으로 바뀌었을 뿐, "지도 하나가 너무 커지면 쪼갠다"는 원칙은 그대로다.

같은 질문이 LiDAR 쪽에서는 따로 굴러갔다. KAIST URL의 Jang·Lee·Nahrendra·Myung이 2026년 공개한 [Chamelion](https://arxiv.org/abs/2602.08189)은 dual-head 네트워크 위에 scene-mixing augmentation을 얹어, 공사 현장이나 재배치가 잦은 실내처럼 구조가 시시각각 뒤집히는 transient 환경에서 change detection을 ground truth 없이 돌린다. Khronos가 RGB-D·panoptic 쪽에서 4D를 세웠다면, Chamelion은 포인트 클라우드 위에서 long-term map maintenance 쪽으로 그 질문을 끌고 간다.

이 계보의 또 다른 축에는 반복성을 다루는 연구가 있다. 스웨덴 Örebro의 Tomáš Krajník과 Achim Lilienthal이 2014년부터 발전시킨 **frequency maps**는 주기적 사건(출근길 차량 흐름, 낮과 밤의 조명 변화)을 Fourier 기반으로 모델링한다. Stockholm Royal Institute of Technology의 Martin Magnusson 그룹이 2019년 정리한 Maps of Dynamics(MoD)는 *typical motion pattern*을 지도에 직접 인코딩했다. "이 복도에서는 사람이 왼쪽으로 걷는다"가 지도의 일부가 된다. 2023년 발표된 [Changing-SLAM(Schmid et al., 2023)](https://arxiv.org/abs/2301.09479)은 ORB-SLAM 확장 위에 Kalman 필터로 short-term을, semantic class 매칭으로 long-term을 동시에 다룬 시도다.

---

## 15b.4 Deformable: 형상이 변할 때

배경조차 움직이면 어떻게 되는가. Zaragoza의 Civera와 Montiel이 오랫동안 이 질문 앞에 서 있었다.

시작은 다른 곳이었다. 2015년 CVPR best paper는 Microsoft Research의 Newcombe, Fox, Seitz가 발표한 [DynamicFusion](https://grail.cs.washington.edu/projects/dynamicfusion/)이었다. KinectFusion의 canonical TSDF에 embedded deformation graph를 얹어, 카메라 앞에서 변형하는 객체(얼굴, 몸통)를 실시간 비강체로 복원했다. 회전·이동이 노드마다 할당된 변형 그래프가 매 프레임 최적화됐다. 같은 계열에서 TU München의 Matthias Innmann이 2016년 [VolumeDeform](https://arxiv.org/abs/1603.08161)으로 색 정보를 더했고, 2017년 Miroslava Slavcheva가 낸 [KillingFusion](https://campar.in.tum.de/pub/slavcheva2017cvpr/slavcheva2017cvpr.pdf)은 Killing vector field 정칙화를 들여와 위상 변화(손이 몸통과 붙었다 떨어지는)까지 허용했다. MIT에서 Tedrake 지도로 나온 Wei Gao의 2019년 [SurfelWarp](https://arxiv.org/abs/1904.13073)는 TSDF 대신 surfel을 골라 exploration 친화성을 확보했다.

> 🔗 **차용.** DynamicFusion의 embedded deformation graph는 컴퓨터 그래픽스에서 Sumner, Schmid, Pauly가 2007년 발표한 ED graph를 직접 가져왔다. 메시 변형을 위한 희소 제어 그래프였던 것이, 실시간 비강체 SLAM의 변수 표현이 되었다.

Montiel 지도 아래 박사를 한 Juan Lamarca가 2021년 [DefSLAM](https://arxiv.org/abs/1908.08918)을 RA-L에 발표했다. isometric NRSfM으로 keyframe마다 template를 다시 계산하고, ORB frontend와 Lucas-Kanade optical flow를 섞어 trace를 유지했다. 평면 토폴로지를 가정하는 한계가 있었다. 같은 그룹의 Juan J. Gómez Rodríguez는 2023년 [NR-SLAM](https://arxiv.org/abs/2308.04036)으로 그 한계를 치웠다. dynamic deformable graph로 임의 토폴로지를 다루고, visco-elastic 모델로 시간 방향 정칙화를 넣었다. Handbook §15.4.2가 이 계보를 "deformable SLAM의 monocular 계통"으로 정리한다.

Tsinghua의 Song이 2018년 낸 [MIS-SLAM](https://ieeexplore.ieee.org/document/8458232)은 stereo endoscopy로 수술 중 장기의 변형을 추적했다. Children's National의 Jayender 그룹이 개발한 EMDQ(Expectation Maximization + Dual Quaternion)는 SURF feature 위에서 부드러운 deformation field를 추정했다. 이들 시스템이 겨냥하는 것은 minimally invasive surgery의 실제 환경에서 intra-operative navigation을 돌리는 일이다.

Handbook §15.4.1은 **Floating Map Ambiguity**를 강조한다. 비강체 객체의 rigid motion과 카메라의 rigid motion은 prior 없이는 구별되지 않는다. 손이 30cm 움직인 것인지 카메라가 30cm 움직인 것인지, 관측만으로는 어느 쪽도 말할 수 없다. Absolute scale 복원은 단안 SLAM의 오래된 scale ambiguity와는 성격이 다르다. Scale만이 아니라 trajectory와 deformation이 동시에 결합하여 ill-posed가 된다. DefSLAM과 NR-SLAM이 isometric prior, visco-elastic prior로 이 ambiguity를 부분적으로 깨지만, 원리적 해법은 2026년 기준에도 없다.

> 📜 **예언 vs 실제.** Newcombe는 DynamicFusion(2015) §7 Future Work에서 "extension to larger scenes and topology changes"와 "integration with loop closure"를 다음 과제로 꼽았다. 토폴로지 변화는 2017년 KillingFusion이 응답했다. 대규모 scene은 surfel 기반 SurfelWarp(2019)가 일부 풀었다. loop closure와의 통합은 2024년 Khronos에 와서야 deformable geometric change detection이라는 이름으로 등장했다. 9년이 걸린 셈이다.

---

## 15b.5 연결을 읽는 한 가지 방식

이 연구들을 세 개의 고정된 학파로 나누기는 어렵다. 다만 논문과 공저 관계, 연구자의 이동을 따라가면 세 계보가 겹쳐 보인다. **Zaragoza 계보**(Montiel, Neira, Civera, Lamarca, Rodríguez)는 MonoSLAM(Ch.5)과 ORB-SLAM(Ch.7)에서 DynaSLAM·DefSLAM·NR-SLAM으로 이어지며 단안 기하를 밀어붙였다. **Dense dynamic reconstruction 계보**는 KinectFusion에서 DynamicFusion으로, SLAM++에서 Co-Fusion·MaskFusion으로 이어졌고 Davison·Newcombe·Agapito·Rünz·Cremers의 연구가 여러 기관에서 맞물렸다. **Schmid의 이동 경로**는 TUM의 Cremers 그룹에서 MIT의 Carlone 그룹을 거쳐 JPL로 이어지며 Dynablox·Panoptic Multi-TSDF·Khronos를 연결한다. 이는 Handbook이 선언한 공식 분류가 아니라, 연구 계보를 읽기 위한 이 장의 해석이다.

---

## 🧭 아직 열린 것

**Absence vs evidence of absence.** 지도에서 객체가 사라졌는지, 가려서 못 봤는지를 구별하는 문제는 long-term SLAM의 근원적 난제로 남아 있다. Schmid의 Panoptic Multi-TSDF는 active submap 구조로 부분 답을 내놓았다. Handbook은 가림이 70%를 넘는 경우를 여전히 풀기 어려운 extreme-environment 사례로 든다. 다만 이를 Panoptic Multi-TSDF의 보편적인 오차 임계값으로 보고한 것은 아니다. 2026년 기준, 이 문제에 원리적 해법을 주장한 논문은 없다.

**Floating Map Ambiguity.** Deformable SLAM에서 카메라의 rigid motion과 객체의 rigid motion을 분리하는 문제는 isometric·visco-elastic prior로만 우회되고 있다. prior 없이 두 motion을 식별하는 조건이 무엇인지, 어떤 관측이 ambiguity를 깨는지는 미해결이다. Lamarca의 [2023년 IJRR 논문](https://arxiv.org/abs/2302.03710)이 관측 조건을 일부 정리했지만 일반 이론은 아직 없다.

**Online deformable SLAM.** DefSLAM과 NR-SLAM은 실시간에 근접했지만, Khronos 수준의 change-aware 통합을 단안 RGB에서 online으로 돌리는 시스템은 없다. Optimization 계산량이 실시간 한계를 넘어선다. GPU 가속과 learned prior가 가능성을 열고 있으나 검증된 파이프라인이 아직 없다.

**의료 MIS의 실세계 격차.** MIS-SLAM과 NR-SLAM이 phantom과 ex vivo 데이터에서는 동작하지만, 실제 수술 환경의 혈액·연기·도구 가림·급격한 조명 변화 앞에서는 견고성이 떨어진다. 2024년 EndoGS 같은 Gaussian 기반 시도가 나오고 있지만 배포 수준에 도달한 시스템은 보고되지 않았다.

이 장이 따라온 질문은 변하는 세계를 어떤 표현으로 담을 것인가였다. GS-SLAM·SplaTAM·MonoGS는 장면 표현의 속도와 밀도를 바꿨지만 정적 세계 가정은 유지했다. Ch.16의 DUSt3R와 후속 계보는 representation pipeline을 다듬는 대신, 기하 prior 자체를 학습하는 길로 들어간다.
