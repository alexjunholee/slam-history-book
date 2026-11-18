# 8장 특징 기반 SLAM: ORB-SLAM

그래프 SLAM 연구가 성숙하면서 포즈 그래프 최적화는 SLAM backend의 표준으로 자리를 잡았다. Kümmerle 등(2011)의 g2o와 Kaess 등(2012)의 iSAM2 덕분에 대규모 지도도 거듭 최적화할 수 있게 됐고, loop closure에 드는 계산 비용도 실용적인 수준으로 내려왔다. backend가 튼튼해지자 자연스럽게 다음 질문이 front-end로 향했다. 영상에서 어떤 특징을 뽑아 어떻게 추적해야 수천 장의 프레임에 걸쳐 안정적인 제약을 만들어 낼 수 있을까?

단서 가운데 하나는 이미 2007년에 나와 있었다. Klein과 Murray의 PTAM은 tracking과 mapping을 두 스레드로 분리해 실시간 단안 SLAM의 가능성을 열었지만, 작은 실내 작업 공간을 벗어나지 못했다. 2015년 스페인 Zaragoza 대학의 Raúl Mur-Artal은 이 이원화 구조에 Rublee 등(2011)의 ORB 서술자, Gálvez-López와 Tardós(2012)의 DBoW2 장소 인식, 그리고 Strasdat의 covisibility 기반 지역 지도와 scale drift를 고려한 loop closing을 엮어 ORB-SLAM을 완성했다. 그렇게 탄생한 ORB-SLAM 계열은 이후 10년 가까이 SLAM 연구의 기준선으로 쓰였다.

---

## 8.1 ORB-SLAM (2015)

2015년 IEEE Transactions on Robotics에 발표된 [Mur-Artal, Montiel & Tardós 2015. ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671)은 이름 그대로 ORB 특징점을 시스템 전체의 공통 언어로 삼은 단안 SLAM이다. 저자들은 tracking과 mapping뿐 아니라 relocalization과 loop closing까지 전 과정을 단 하나의 특징점 표현으로 일관되게 처리한 점을 첫 번째 기여로 내세웠다.

시스템은 Tracking, Local Mapping, Loop Closing이라는 세 스레드가 병렬로 맞물려 돌아간다. PTAM의 Tracking–Mapping 구조에 Loop Closing 스레드를 따로 떼어 붙여, DBoW2로 재방문 장소를 찾고 Essential graph 위에서 포즈 그래프 최적화를 수행하게 한 것이다. 덕분에 지도를 고치고 루프를 닫는 무거운 작업이 진행되는 동안에도 Tracking 스레드는 끊김 없이 실시간 추적을 이어 갈 수 있었다(loop closure 뒤 전역 Bundle Adjustment를 전담하는 네 번째 스레드는 ORB-SLAM2에서 추가됐다).

논문은 frontend 특징점을 고른 기준도 분명히 못 박았다. 30 Hz 카메라 입력을 놓치지 않으려면 프레임당 특징 추출 시간이 33 ms보다 훨씬 짧아야 했는데, SIFT와 SURF는 약 300 ms, A-KAZE도 약 100 ms가 걸려 실시간 예산에 맞지 않았다. 반대로 계산이 빠른 BRIEF와 LDB는 회전 불변성이 없어 장소 인식에 쓰기 어려웠다. 두 조건을 모두 만족한 것이 Willow Garage의 Rublee 등이 2011년 ICCV에서 발표한 ORB(Oriented FAST and Rotated BRIEF)였다. FAST keypoint에 밝기 기반 방향을 부여하고 그 각도에 맞춰 BRIEF 비교 쌍을 회전시킨 ORB는 원 논문 기준으로 SIFT보다 약 100배 빨랐고, 당시 SIFT와 SURF를 얽어매던 특허·라이선스 제약에서도 자유로웠다. 게다가 binary 서술자라 GPU 없이 CPU만으로도 Hamming 거리 계산을 통해 빠르게 매칭할 수 있었다.

ORB-SLAM은 크기 변화와 회전 변화에 대응하기 위해 정교한 전처리를 거친다. 먼저 원본 영상을 1.2배씩 축소해 8단계 image pyramid를 만들고, 각 단계에서 FAST keypoint를 따로 검출한다. 이어 패치 안 픽셀 밝기의 1차 모멘트로 밝기 중심(intensity centroid)을 구한 뒤, keypoint에서 그 중심을 향하는 방향각 θ만큼 BRIEF 비교 쌍을 회전시켜 회전에 불변인 256비트 binary vector를 얻는다. 두 서술자 사이의 거리는 비트별 XOR 연산 뒤 1인 비트를 세는 Hamming 거리로 단번에 계산한다.

keyframe을 관리하는 전략도 PTAM과 정반대였다. 추적을 놓치지 않으려면 새 keyframe을 과감히 추가해야 하지만, 그대로 쌓아 두면 최적화가 금세 느려진다. ORB-SLAM은 일단 keyframe을 넉넉히 삽입한 뒤, **Covisibility graph**를 살펴 중복된 keyframe을 과감히 솎아 내는 전략을 택했다. Covisibility graph는 두 keyframe이 함께 관측하는 landmark가 15개 이상일 때 엣지로 잇고 그 개수를 가중치로 삼는 그래프다. Local Mapping은 이 그래프에서 현재 keyframe과 강하게 연결된 지역 window를 골라 그 안에서만 bundle adjustment를 돌린다.

한편 루프를 닫을 때는 그래프 전체를 그대로 쓰지 않고 **Essential graph**라는 더 희소한 골격을 꺼내 든다. Essential graph는 covisibility graph에서 함께 보는 landmark가 100개 이상인 엣지, spanning tree, 그리고 loop closure 엣지만 남긴 부분 그래프다. keyframe이 수천 개로 불어나도 Essential graph의 엣지 수는 적어 포즈 그래프 최적화가 몇 초 안에 끝난다.

loop closure의 방아쇠를 당기는 장소 인식은 [Gálvez-López & Tardós 2012. DBoW2](https://doi.org/10.1109/TRO.2012.2197158)가 맡는다. DBoW2는 binary 서술자로 vocabulary tree를 구축한 첫 기법으로, k-means++ seeding을 적용한 k-medians 군집화로 서술자를 계층적으로 묶어 트리 모양의 사전을 만든다. 원래 논문은 FAST keypoint와 BRIEF로 실험했고, ORB-SLAM은 같은 절차로 ORB vocabulary를 구축했다. 분기 수 $k_w$와 깊이 $L_w$로 이루어진 트리에서 말단 노드(word)의 수는 $k_w^{L_w}$가 되며, DBoW2와 ORB-SLAM 모두 $k_w=10$, $L_w=6$을 써서 단어 100만 개짜리 vocabulary를 구성했다. 각 word에는 데이터베이스 전체에서의 희귀도를 반영한 TF-IDF(Term Frequency–Inverse Document Frequency) 가중치가 매겨져, 장소를 잘 구별하는 단어일수록 더 큰 영향력을 행사한다. 새 프레임이 들어오면 서술자를 트리 깊이 $L_w$만큼 내려 보내 해당 word를 정하고, inverted index에서 그 word가 등장한 keyframe 목록만 조회하면 되므로 지도 전체를 훑지 않고도 루프 후보를 즉시 찾아낸다.

이렇게 짜인 구조 위에서 Tracking 스레드는 매 프레임 현재 자세를 추정한다. 직전 프레임과 특징점을 매칭해 초기 자세를 잡은 뒤, 맵 포인트를 고정한 상태에서 현재 카메라 자세 $\mathbf{T}_{cw} \in SE(3)$만 다듬는 motion-only bundle adjustment를 푼다. 3D–2D 대응 $\{(\mathbf{X}_i, \mathbf{u}_i)\}$에 대한 기본 reprojection 목적함수는 다음과 같다.

$$\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \left\| \mathbf{u}_i - \pi(\mathbf{T}\mathbf{X}_i) \right\|^2$$

여기서 $\pi$는 카메라 투영 함수, $\mathbf{X}_i$는 맵 포인트의 월드 좌표, $\mathbf{u}_i$는 이미지 좌표를 뜻한다. 실제 최적화에서는 이상치의 영향을 줄이기 위해 robust loss와 관측 가중치를 함께 씌운다. 만약 추적을 잃으면 DBoW2로 비슷한 keyframe 후보를 찾은 뒤 EPnP와 RANSAC으로 초기 자세를 복구하는 relocalization에 들어가며, 이웃 keyframe과 맵 포인트를 함께 다듬는 local BA는 Local Mapping 스레드가 이어받는다.

성능은 곧바로 수치로 입증됐다. KITTI 시퀀스 00(564×496 m 영역, keyframe 1,391개)에서 ORB-SLAM의 keyframe 궤적 RMSE는 6.68 m를 기록했다(논문 Table V, 다섯 번 실행한 값의 중앙값). 단안 시스템 특성상 절대 scale을 알 수 없어 참값과 비교할 때는 similarity 변환으로 궤적을 정렬해 오차를 쟀으며, 긴 주행에서 누적 오차를 붙잡아 준 주역은 바로 DBoW2와 Essential graph였다.

> 🔗 **차용.** ORB-SLAM의 Tracking–Local Mapping 이원 구조는 PTAM(Klein & Murray, 2007)에서 직접 이어졌다. 논문 서론 역시 PTAM의 핵심 설계를 토대로 삼았다고 밝힌다.

> 🔗 **차용.** 시스템의 이름 자체가 된 서술자는 Willow Garage의 Rublee, Rabaud, Konolige, Bradski가 발표한 [Rublee et al. 2011. ORB](https://doi.org/10.1109/ICCV.2011.6126544)다. Mur-Artal의 공헌은 검증된 부품들을 한데 꿰어 빈틈없는 파이프라인으로 조립한 데 있다.

> 🔗 **차용.** [Strasdat et al. 2011. Double Window Optimisation](https://doi.org/10.1109/ICCV.2011.6126517)은 PTAM의 frontend를 유지하되 covisibility graph로 고른 지역 지도에서만 추적하고, 안쪽 window에서는 BA를, 바깥 window에서는 포즈 그래프 최적화를 돌렸다. ORB-SLAM은 covisibility 기반 지역 지도 선택을 이 논문에서, 7자유도(similarity) 포즈 그래프로 루프를 닫는 발상을 Strasdat 등의 2010년 scale drift 연구에서 가져왔으며, 여기에 Essential graph라는 더 희소한 골격을 새로 얹었다.

---

## 8.2 ORB-SLAM2 (2017) — stereo/RGB-D

첫 번째 ORB-SLAM(2015)은 단안 카메라만을 위한 시스템이었다. 단안 영상만으로 복원한 지도와 궤적은 전체에 임의의 배율(scale)을 곱해도 투영 관측과 똑같이 맞기 때문에 실제 미터 단위의 크기를 복원할 수 없다.

2017년 발표된 [Mur-Artal & Tardós 2017. ORB-SLAM2](https://doi.org/10.1109/TRO.2017.2705103)는 stereo 카메라와 RGB-D 센서를 품어 이 scale 모호성을 정면으로 해결했다. stereo 카메라는 두 렌즈 사이의 간격(baseline)을 알기에 삼각측량으로 곧바로 깊이를 얻고, RGB-D 센서는 물리적으로 깊이를 직접 측정하므로 어느 쪽이든 첫 순간부터 절대 scale이 고정된다.

내부 아키텍처는 기존의 세 스레드 골격을 그대로 둔 채, loop closure가 끝난 뒤 전체 맵을 다듬는 전역 BA 전담 네 번째 스레드를 보탰다. 센서별로 달라지는 대목은 frontend의 입력 처리다. stereo 모드에서는 rectified 좌우 영상에서 ORB를 추출해 대응점을 찾고, 좌우 매칭에 성공한 점은 **stereo 관측**으로, 한쪽 영상에서만 잡힌 점은 **monocular 관측**으로 구분해 활용한다. 깊이가 계산된 stereo 점은 다시 baseline의 40배를 기준으로 가까운 점과 먼 점으로 나눈다. 가까운 점은 카메라의 translation을 정밀하게 붙잡아 주고, 먼 점은 시차 변화가 작아 회전 추정에 주로 기여하기 때문이다.

덕분에 **Stereo 초기화**는 단안처럼 두 프레임 사이의 시차를 기다릴 필요 없이 첫 프레임 한 장에서 곧바로 끝난다. 단안 초기화가 fundamental matrix나 homography에 기대어 임의 scale의 지도를 세웠다면, stereo는 첫 keyframe에서 좌우 영상의 수평 시차(disparity) $d$와 baseline $b$, 초점 거리 $f$로부터 깊이를 바로 계산한다.

$$Z = \frac{b \cdot f}{d}$$

ORB-SLAM2는 첫 프레임을 곧장 원점 keyframe으로 등록하고, 깊이를 구한 stereo 특징점 전체로 초기 지도를 띄운다. RGB-D 초기화도 원리는 같다. depth 영상의 픽셀 $(u, v)$에서 깊이 값 $Z$를 읽어 역투영(back-projection)으로 3D 좌표를 세우면 처음부터 scale이 정해진 지도가 만들어진다.

EuRoC MAV(Micro Aerial Vehicle) 데이터셋의 Machine Hall 01 시퀀스에서 stereo ORB-SLAM2는 translation RMSE 0.035 m를 달성하며(Table II), 같은 표에서 비교된 Stereo LSD-SLAM보다 높은 정확도를 보였다. 물론 한계도 드러나서, 심한 motion blur가 낀 V2_03 시퀀스에서는 추적을 일부 놓치기도 했다.

무엇보다 결정적이었던 것은 공개 방식이었다. Mur-Artal과 Tardós는 mono, stereo, RGB-D 세 모드를 하나의 깔끔한 C++ 코드베이스로 묶어 GitHub에 공개했고, ROS 예제까지 함께 실어 두었다. 저장소의 star는 수천 개를 넘어섰고, ORB-SLAM2는 연구실에서 곧바로 꺼내 쓸 수 있는 사실상의 표준 구현이 됐다.

---

## 8.3 ORB-SLAM3 (2021): Atlas와 Visual-Inertial

2021년 IEEE Transactions on Robotics에 실린 [Campos et al. 2021. ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644)에서는 저자진이 세대교체를 이뤘다. Carlos Campos와 Richard Elvira가 공동 1저자를 맡고 Juan J. Gómez Rodríguez, J.M.M. Montiel, Juan D. Tardós가 함께 이름을 올렸으며, 모두 Zaragoza 대학 소속이되 Mur-Artal은 저자 목록에서 빠졌다.

ORB-SLAM3가 내놓은 핵심 확장은 **Atlas**(멀티맵 구조)와 **Visual-Inertial** 모드다. 먼저 Atlas는 서로 분리된 지도 여러 개를 하나의 시스템 안에서 함께 관리한다. 추적이 끊어졌을 때 기존 ORB-SLAM과 ORB-SLAM2는 현재 지도 안에서 relocalization을 시도할 뿐, 복구에 실패하면 앞선 지도를 유지한 채 새 지도를 이어 가지 못했다. ORB-SLAM3는 추적이 실패하면 진행 중이던 지도를 닫고 곧바로 새 지도를 띄워 추적을 이어 가며, 나중에 로봇이 이전 지도의 영역을 다시 지나면 두 지도를 하나로 병합한다.

여기에 더해진 Visual-Inertial(VI) 모드는 카메라와 IMU 측정을 하나의 최적화 안에 묶는다. 카메라가 빠르게 회전해 영상이 흐려질 때도 IMU가 빈틈을 메워 추적을 지켜 줄 뿐만 아니라, 가속도계가 미터 단위의 가속도와 중력 방향을 재는 덕분에 단안 카메라로도 절대 scale을 복원할 수 있다. 관성 측정 처리는 Forster 등이 RSS 2015에서 처음 내놓고 2017년 IEEE TRO에 확장 발표한 [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321)를 따른다.

핵심 아이디어는 keyframe $i$와 $j$ 사이에 쏟아지는 IMU 측정을 매번 다시 적분하지 않고 미리 한 번만 묶어 두는 데 있다. 가속도계와 자이로스코프 측정을 각각 bias $\mathbf{b}$와 noise $\mathbf{n}$을 포함한 $\tilde{\mathbf{a}}_t = \mathbf{a}_t + \mathbf{b}_a + \mathbf{n}_a$, $\tilde{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t + \mathbf{b}_g + \mathbf{n}_g$로 모델링하면, 두 keyframe 사이의 상대 회전·속도·위치 증분은 다음과 같이 사전적분(preintegration)된다.

$$\Delta\mathbf{R}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp}\bigl((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\bigr)$$
$$\Delta\mathbf{v}_{ij} = \sum_{k=i}^{j-1} \Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t$$
$$\Delta\mathbf{p}_{ij} = \sum_{k=i}^{j-1}\!\left[\Delta\mathbf{v}_{ik}\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\right]$$

여기서 $\mathrm{Exp}(\cdot)$는 $\mathfrak{so}(3)$의 지수 사상이다. bundle adjustment 도중 bias 추정값이 조금 바뀌더라도 처음부터 적분을 되풀이할 필요 없이 1차 선형 근사로 증분을 즉시 보정할 수 있다. ORB-SLAM3는 이렇게 만든 preintegrated 항($\Delta\mathbf{R}$, $\Delta\mathbf{v}$, $\Delta\mathbf{p}$)을 factor graph의 inertial edge로 꽂아 넣고 visual reprojection residual과 함께 최적화한다.

결과는 뚜렷했다. EuRoC MAV 11개 시퀀스의 평균 ATE(절대 궤적 오차) RMSE에서 mono-inertial ORB-SLAM3는 0.043 m를 기록해, 같은 표(Table II)에 오른 VINS-Mono(0.110 m)와 stereo-inertial Kimera(0.119 m)를 크게 앞섰다. 논문은 단일 세션 mono-inertial 기준으로 ORB-SLAM3가 VINS-Mono보다 2.6배 정확하다고 강조했다.

> 🔗 **차용.** ORB-SLAM3의 관성 항 처리는 Forster 등의 On-Manifold Preintegration(TRO 2017, 초판 RSS 2015)을 그대로 가져다 썼다. 연속된 IMU 측정을 bias 보정과 함께 SO(3) manifold 위에서 미리 적분해 둔 뒤, 그 결과를 factor graph 최적화의 제약으로 결합하는 구조다.

---

## 8.4 2020년대에도 비교 기준으로 쓰이는 이유

딥러닝 기반 SLAM이 쏟아진 2023년 이후에도 학회 논문의 비교표에는 여전히 ORB-SLAM3가 빠지지 않는다. 가장 큰 비결은 실용성과 견고함의 균형에 있다. ORB 특징점은 웬만한 조명 변화에 크게 흔들리지 않으며, binary 서술자 특유의 가벼운 계산 덕분에 프레임마다 많은 점을 뽑아도 실시간을 유지해 추적 실패를 줄인다. 반면 학습 기반 특징점은 특정 벤치마크에서 더 정밀한 대응을 찾다가도, 학습 데이터와 동떨어진 환경에 놓이면 성능이 급격히 무너지곤 한다.

여기에 비교 실험의 편의성까지 겹쳤다. 단일 코드베이스가 mono, stereo, RGB-D, IMU 조합을 모두 지원하고 ROS와도 매끄럽게 연결되니, 새 알고리즘을 제안하는 연구자로서는 센서 설정별로 ORB-SLAM3를 나란히 세워 검증하기가 가장 수월하다.

물론 DROID-SLAM(Teed & Deng, NeurIPS 2021)처럼 여러 벤치마크에서 ORB-SLAM3보다 낮은 오차를 기록한 시스템도 등장했다. 하지만 DROID-SLAM을 실시간으로 돌리려면 고성능 GPU가 뒷받침되어야 하는 반면(→ 16장 「아직 열린 문제」), ORB-SLAM3는 CPU만으로도 거뜬히 동작하며 ARM 임베디드 보드 위에서도 실행된다는 점이 여전히 강력한 강점으로 남아 있다.

---

## 예측과 실제

> 📜 **예언 vs 실제.** 2015년 ORB-SLAM 논문의 Section IX-C(Future Work)에서 Mur-Artal은 두 가지 방향을 짚었다. 첫째는 시차가 작아 지도에 올리지 못한 먼 점(points at infinity)을 tracking에 활용해 카메라 회전을 더 정밀하게 붙잡자는 제안이었고, 둘째는 정확하게 추정한 keyframe과 풍부한 covisibility 정보를 골격 삼아 희소한 지도를 조밀한(dense) 복원으로 키우자는 구상이었다. 먼 점 활용은 후속 VI-SLAM과 여러 연구에 일부 스며들었고, 희소한 keyframe 위에 조밀한 표현을 얹는 발상은 2020년대 NeRF-SLAM과 Gaussian Splatting 기반 SLAM에서 꽃을 피웠다. 정작 그 절에 적히지 않았던 stereo, RGB-D, IMU 통합이 ORB-SLAM2(2017)와 ORB-SLAM3(2021)를 통해 현실이 됐다는 점도 흥미롭다.

> 📜 **예언 vs 실제.** Campos 등(2021)은 ORB-SLAM3 논문 결론에서 텍스처가 부족한 환경을 가장 잦은 실패 원인으로 꼽으며, 자신들이 정식화한 네 가지 data association 문제에 photometric 기법을 접목하는 방향을 유망한 과제로 들었다. 실제로 저자들은 이 아이디어를 인체 내부 내시경 영상으로 지도를 만드는 연구에 시험하고 있었다. 그러나 2023–2025년의 실제 흐름은 DSO나 LDSO 같은 direct 방식과 별개로, ORB-SLAM3의 앞단에 SuperPoint나 LightGlue 같은 learned frontend를 갈아 끼우는 쪽으로 더 활발히 움직였다. 그럼에도 ORB-SLAM3 공식 저장소의 main branch는 2026년 현재까지도 여전히 전통적인 ORB 서술자를 기본값으로 유지하고 있다.

---

## 아직 열린 문제

**지도의 장기 재사용.** Atlas로 여러 지도를 따로 보관하다 합칠 수는 있게 됐지만, 조명이 극단적으로 달라지면 지도 병합 자체가 막힌다. 아침에 만든 지도를 저녁에 다시 찾아와 연결하려 해도 겉모습이 너무 달라지면 DBoW2가 같은 장소로 알아채지 못한다. 날씨와 계절이 바뀌는 실외 환경에서 지도를 장기간 재사용하는 일은 여전히 해결해야 할 과제다.

**전통 feature는 아직 기준선인가.** SuperPoint와 SuperGlue, LightGlue, DINOv2 기반 특징점을 얹은 시스템들이 표준 벤치마크에서 ORB-SLAM3의 오차를 앞지르기 시작했다. 하지만 학습 분포를 벗어난 낯선 환경에서는 여전히 전통적인 ORB보다 못한 결과를 내는 사례가 보고된다.

**넓은 실외의 drift.** 수 km를 달리는 도심 주행에서는 ORB-SLAM3도 LiDAR SLAM의 정밀도를 따라잡지 못한다. 시시각각 변하는 조명과 날씨, 움직이는 차량, 텍스처 없는 도로 구간이 겹치면 서서히 drift가 쌓여 버린다.

---

ORB-SLAM 세 편이 특징점 기반 SLAM의 기준을 세우던 바로 그 시기, Newcombe와 Engel은 특징점 추출 자체를 건너뛰고 픽셀 밝기 값을 직접 최적화하는 정반대 접근법을 밀어붙이고 있었다. 두 진영은 2010년대 내내 서로를 겨누며 발전했고, 실제로 DSO 논문은 loop closure와 relocalization을 끈 단안 ORB-SLAM과의 비교에서 TUM monoVO와 ICL-NUIM의 정확도를 앞섰다고 보고했다(→ 11장 §11.3). 한편 ORB-SLAM3가 단안 카메라의 한계를 넘기 위해 끌어다 쓴 IMU 사전적분(preintegration)은 그 자체로 독자적인 수학적 계보를 이루고 있었다. 다음 장에서는 2009년 시드니에서 싹터 SO(3) 다양체 위에서 완성된 이 관성 적분의 역사를 따라간다.