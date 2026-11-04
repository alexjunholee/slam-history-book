# Ch.8 — Direct 방법: DTAM에서 DSO까지

Richard Newcombe는 Imperial College에서 Andrew Davison의 박사과정 학생이었다. MonoSLAM은 EKF로 점 100개 안팎을 추적해 실시간으로 돌았다. 2011년 Newcombe는 GPU 한 장으로 화면의 모든 픽셀을 쓰면서도 실시간으로 도는 시스템을 만들었다. 그 시스템이 DTAM이다.

ORB-SLAM은 feature를 먼저 뽑고 그 feature만 추적한다. corner 검출과 ORB 서술자로 고른 점 1,000~2,000개를 남기고, 나머지 픽셀은 버린다. direct 방법(밝기를 직접 비교하는 방법)은 대응점의 기하 오차 대신 픽셀 밝기 차이를 측정값으로 쓴다. 모든 픽셀을 쓰는 dense 방법부터 일부만 선택하는 sparse 방법까지 여기에 속한다.

같은 무렵 뮌헨 TUM의 Daniel Cremers 그룹은 Gauss-Newton image alignment와 optical flow 같은 컴퓨터 비전 방법을 SLAM에 가져오고 있었다. Cremers의 학생 Jakob Engel은 2014년 LSD-SLAM을, 2016년 DSO를 내놓았다. 두 시스템은 쓰는 픽셀의 밀도가 다르다.

---

## 1. 모든 픽셀: DTAM

Newcombe, Lovegrove, Davison은 2011년 ICCV에서 [DTAM](https://doi.org/10.1109/ICCV.2011.6126513)을 발표했다. DTAM은 "Dense Tracking and Mapping in Real-Time"의 약자다. 이름대로 모든 픽셀을 써서 추적과 지도 작성을 실시간으로 한다.

시스템은 추적과 지도 작성 두 부분으로 이루어진다. 추적 단계에서는 dense 모델에서 합성한 영상에 현재 프레임 전체를 photometric alignment로 맞춘다. 특징점 추출이나 서술자 매칭 없이 픽셀 intensity 차이만 최소화한다. 지도 작성 단계에서는 keyframe 가까이에서 찍은 여러 프레임의 photometric 오차를 cost volume(픽셀마다 깊이 후보별 비용을 쌓은 3차원 배열)에 모은다. 이 비용에 regularization 항을 더한 에너지를 최소화해 keyframe의 inverse depth map을 구한다.

$$E(\mathbf{u}) = \frac{1}{|\mathcal{I}|}\sum_{i \in \mathcal{I}} \Bigl\| I_r\bigl(\pi(\mathbf{p}(\mathbf{u}))\bigr) - I_i\bigl(\pi(KT_i\mathbf{p}(\mathbf{u}))\bigr) \Bigr\|_1 + \lambda \, g\,\|\nabla \mathbf{u}\|_\epsilon$$

여기서 $\mathbf{u}$는 역 깊이(inverse depth) 맵, $\mathbf{p}(\mathbf{u})$는 $\mathbf{u}$로 역투영한 3D 점, $K$는 카메라 내부 행렬, $T_i$는 참조 프레임 기준 $i$번 프레임의 rigid body 변환, $\pi$는 원근 투영이다. 데이터 항은 프레임마다 photometric 오차의 L1 norm을 구해 평균한다. $\|\nabla \mathbf{u}\|_\epsilon$은 inverse depth gradient의 Huber norm이고, $g$는 참조 영상의 gradient가 큰 곳에서 작아지는 픽셀별 가중치다. Huber norm은 작은 gradient에는 제곱 norm을, 큰 gradient에는 L1 norm(total variation)을 쓴다. 그래서 면은 매끄럽게 하면서 깊이 경계는 남긴다. 이 최적화를 실시간으로 돌리려면 GPU가 필요하다. 논문은 당시 소비자용 GPU인 NVIDIA GTX 480 한 장에서 실행했다.

> 🔗 **차용.** 같은 해 Newcombe가 주도한 [KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378)(ISMAR 2011)은 depth sensor로 dense 재구성을 했다. 지도 표현에는 [Curless & Levoy 1996](https://doi.org/10.1145/237170.237269)의 TSDF(truncated signed distance function)를 썼다. DTAM은 단안(monocular) 카메라로 dense depth map을 추정했다.

DTAM은 실내 장면을 실시간으로 dense하게 복원해 보였다. 다만 조명 변화에 약했다. 논문이 시연한 곳은 실내의 작은 공간이었다.

<!-- DEMO: dtam_photometric_residual.html -->

---

## 2. Semi-dense: LSD-SLAM

Engel, Schöps, Cremers가 ECCV 2014에 낸 [LSD-SLAM](https://doi.org/10.1007/978-3-319-10605-2_54)(Large-Scale Direct Monocular SLAM)은 모든 픽셀을 쓰지 않는다. 그 대신 GPU 없이 CPU에서 실시간으로 돈다. LSD-SLAM은 semi-dense 방식이다. 이미지에서 gradient 크기가 임계값 이상인 픽셀만 쓴다. 벽의 평평한 부분은 버리고 edge 근처 픽셀만 남긴다. corner detector는 쓰지 않는다.

추적은 SE(3) direct image alignment다. 현재 프레임을 keyframe으로 warping해 photometric residual을 Gauss-Newton으로 최소화한다. 지도는 keyframe마다 semi-dense depth map을 하나씩 둔다. keyframe 사이의 관계는 Sim(3) pose graph로 관리한다. 그래서 단안 카메라의 scale drift도 함께 다룬다. 큰 loop closure 후보는 appearance 기반 검색(FAB-MAP)으로 찾는다. 후보마다 양쪽 방향으로 Sim(3) 정렬을 따로 하고, 두 결과가 통계적으로 비슷할 때만 받아들인다.

> 🔗 **차용.** Gauss-Newton photometric registration은 이미지 정렬 분야의 고전이다. [Lucas & Kanade 1981](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf) tracker와 그 inverse compositional 변형([Baker & Matthews 2004](https://doi.org/10.1023/B:VISI.0000011205.11775.fd))이 LSD-SLAM frontend의 바탕이다.

tracking과 mapping을 나눈 점은 PTAM과 같다. 다만 LSD-SLAM에는 ORB나 BRIEF 같은 binary descriptor가 없고, 픽셀 밝기만 측정값으로 쓴다.

논문은 실내뿐 아니라 실외의 큰 장면에서 만든 semi-dense 지도도 보여줬다. 조명 변화에는 약했다. 터널 진입, 창문 역광, 갑작스러운 플래시에서는 photometric consistency 가정이 깨진다. 그러면 시스템이 불안정해진다.

<!-- DEMO: lsd_slam_semidense.html -->

---

## 3. Sparse direct: DSO

Engel, Koltun, Cremers의 [DSO](https://doi.org/10.1109/TPAMI.2017.2658577)(Direct Sparse Odometry)는 2016년 arXiv에 먼저 공개됐고, 2018년 IEEE TPAMI에 실렸다. DSO는 LSD-SLAM보다 적은 픽셀을 쓴다. 그 대신 photometric calibration을 꼼꼼히 한다.

DSO는 새 keyframe마다 후보 픽셀 2,000개를 고른다. 최적화에 남기는 활성 점은 모든 활성 프레임을 합쳐 2,000개다. 후보를 고를 때는 영상 전체에 고르게 퍼지도록 하고, 주변보다 gradient가 큰 픽셀을 고른다. 이 점들로 sliding window bundle adjustment를 한다. 최적화 변수는 camera pose, inverse depth, affine brightness 파라미터 $(a_i, b_i)$다. 윈도우를 벗어난 프레임은 marginalization으로 제거한다. 이때 Schur complement로 해당 변수를 소거하므로 최적화 문제의 크기가 일정하게 유지된다. 전체 계산량은 창 크기와 픽셀 수, 반복 횟수에 달려 있다.

DSO는 카메라의 photometric 모델을 나눠서 다룬다. vignetting(렌즈 가장자리로 갈수록 어두워지는 현상)은 미리 캘리브레이션해 보정한다. camera response function(센서가 빛을 비선형으로 기록하는 특성, gamma curve)도 미리 역함수를 추정해 선형 밝기로 바꾼다. 프레임의 노출 시간 $t_i$는 입력 메타데이터로 받고, 남은 affine brightness 변화 $(a_i, b_i)$를 실시간으로 추정한다:

$$E_{pj} = \sum_{\mathbf{p} \in \mathcal{N}_p} w_{\mathbf{p}} \left\| \left( I_j\!\left[\mathbf{p}'\right] - \frac{t_j e^{a_j}}{t_i e^{a_i}} I_i[\mathbf{p}] - \left(b_j - \frac{t_j e^{a_j}}{t_i e^{a_i}} b_i\right) \right) \right\|_\gamma$$

여기서 $t_i, t_j$는 노출 시간, $(a_i, b_i)$와 $(a_j, b_j)$는 각 프레임의 affine brightness 파라미터(gain과 bias), $\|\cdot\|_\gamma$는 Huber loss이다. 위 잔차는 전처리 단계의 photometric calibration으로 vignetting을 보정한 intensity에 적용한다.

> 🔗 **차용.** Photometric camera calibration의 형식적 기반은 [Debevec & Malik 1997](https://doi.org/10.1145/258734.258884)의 HDR 복원 연구에서 왔다. DSO는 response function을 미리 보정하고, 실행 중에는 프레임별 affine brightness 파라미터만 추정한다.

논문은 TUM monoVO, EuRoC, ICL-NUIM 데이터셋에서 DSO를 단안 ORB-SLAM과 비교했다. ORB-SLAM은 loop closure 검출과 relocalization을 끄고 돌렸다. TUM monoVO와 ICL-NUIM에서는 DSO가 정확도와 강건성 모두 앞섰고, EuRoC에서는 ORB-SLAM이 더 정확했지만 더 자주 실패했다. 저자들은 그 이유로 EuRoC에 photometric calibration이 없다는 점과, 드론이 작은 loop를 돌거나 왔던 길을 되짚는 구간이 많아 ORB-SLAM의 local mapping이 작은 loop 여럿과 일부 큰 loop를 암묵적으로 닫을 수 있었다는 점을 들었다. ORB-SLAM이 10초 넘게 관측되지 않은 keypoint와는 매칭하지 못하게 하자 정확도는 DSO와 비슷해졌고 강건성은 여전히 낮았다.

> 📜 **예언 vs 실제.** DSO는 사전 photometric calibration을 요구했다. 2018년 Bergmann, Wang, Cremers는 [online photometric calibration](https://doi.org/10.1109/LRA.2017.2777002)을 RA-L에 냈다. 이 방법은 KLT로 추적한 대응점을 써서 노출 시간, response function, vignetting을 추정한다. direct VO와 나란히 실시간으로 돌며, 들어오는 프레임을 바로 보정한다. TUM monoVO 데이터셋에서 이 보정을 거친 auto-exposure 영상으로 DSO를 돌린 결과는 미리 보정한 영상으로 돌린 결과와 비슷했다.

> 📜 **예언 vs 실제.** DTAM 뒤 dense 재구성 연구는 주로 KinectFusion처럼 RGB-D depth sensor를 썼고, 단안 dense 재구성은 2020년대 NeRF와 3DGS 이후 다시 활발해졌다.

---

## 4. VI-DSO와 후속 연구

von Stumberg, Usenko, Cremers는 ICRA 2018에서 DSO에 IMU를 결합한 [VI-DSO](https://doi.org/10.1109/ICRA.2018.8462905)를 발표했다. direct 방법은 조명이 급변할 때 자주 실패한다. 이때 IMU 측정이 자세 추적을 돕는다. 단안 카메라의 scale 모호성도 IMU로 풀 수 있다.

VI-DSO는 DSO의 sliding window photometric bundle adjustment에 IMU preintegration factor를 더했다. 논문은 scale을 복원하고 어려운 조건에서도 덜 실패한다고 보고했다.

Cremers 그룹은 2022년 [DM-VIO](https://doi.org/10.1109/LRA.2021.3140129)에서도 direct photometric frontend에 IMU를 tightly coupled로 결합했다. 같은 그룹의 [Basalt](https://arxiv.org/abs/1904.06504)(RA-L 2020)는 direct 방식 대신 patch 기반 optical flow로 점을 추적하는 VIO다. direct VIO 연구와 VINS-Mono, OpenVINS 같은 feature 기반 VIO 연구는 같은 시기에 함께 나왔다.

> 🔗 **차용.** VI-DSO의 IMU factor는 [Forster et al. 2017. On-Manifold Preintegration (IEEE TRO)](https://doi.org/10.1109/TRO.2016.2597321)의 preintegration을 바탕으로 한다.

---

## 5. direct method의 한계

Direct method는 descriptor로 요약한 특징점 대신 영상 밝기 자체를 잔차에 넣는다. 픽셀을 고르는 범위는 방법마다 다르다. DTAM은 모든 픽셀의 밝기를 썼다. LSD-SLAM은 gradient가 있는 semi-dense 영역을 썼다. DSO는 주변보다 gradient가 큰 픽셀을 영상 전체에서 고르게 골랐다. 강한 gradient가 없는 영역에서는 임계값을 낮춰 약한 밝기 변화도 썼다. 그래서 corner가 드문 곳에서도 정보를 얻는다. 하지만 밝기가 거의 일정한 면에서는 자세가 바뀌어도 잔차가 거의 변하지 않는다.

**Photometric calibration.** DSO가 가정하는 vignetting 보정, response curve 보정, 노출 정보를 소비자용 카메라에서 얻으려면 따로 작업해야 한다. 스마트폰 카메라는 HDR 합성, auto-exposure, 실시간 화이트 밸런스를 내부 파이프라인으로 처리한다. 그 세부는 제조사만 안다. 이런 카메라에서는 DSO photometric 모델의 기본 가정이 맞지 않는다.

**조명 변화.** direct 방식은 photometric consistency(같은 점은 프레임이 바뀌어도 밝기가 같다는 가정)에 기댄다. 자동 노출, 역광, 플리커처럼 프레임 사이 밝기가 급변하면 이 가정이 바로 깨진다. DSO의 affine brightness 모델은 완만한 밝기 변화만 흡수한다. 실외에서 구름이 지나가거나 실내에서 형광등이 깜빡이는 장면에서는 추적이 실패할 수 있다.

**카메라 교체.** 통제된 데이터셋에서는 DSO가 ORB-SLAM보다 나은 시퀀스가 있었다. 다만 ORB-SLAM은 photometric calibration 없이 여러 카메라에서 돌고, 카메라를 바꿔도 바로 쓸 수 있다. DSO는 카메라마다 vignetting·response curve를 따로 캘리브레이션해야 했다.

**학습 기반 feature.** [SuperPoint](https://arxiv.org/abs/1712.07629)(2018) 같은 학습 기반 feature와 [LightGlue](https://arxiv.org/abs/2306.13643)(2023) 같은 학습 기반 matcher가 나왔다. 이들은 어려운 조건에서 손으로 설계한 descriptor보다 대응점을 더 잘 찾는다. 그러면서도 descriptor matching 방식을 그대로 쓴다.

<!-- DEMO: photometric_calibration_demo.html -->

---

## 🧭 아직 열린 것

**조명이 급변할 때의 direct tracking.** §5에서 본 photometric consistency 가정이 깨지는 문제는 아직 풀리지 않았다. VI-DSO처럼 IMU를 쓰면 나아진다. 조명 모델을 실행 중에 추정하는 방법으로는 §3에서 본 Bergmann 등의 online photometric calibration이 있다.

**Texture가 없는 장면.** feature 기반 방법은 corner가 없는 벽 앞에서 실패한다. direct 방식도 균질한 면에서는 자세가 바뀌어도 밝기 잔차가 거의 그대로다. 두 방식 모두 실내 복도, 대규모 창고, 균질한 실외 지형 같은 환경에서 약하다. LSD-SLAM은 gradient가 있는 픽셀만 골라 썼다. 그런 픽셀이 화면 한쪽에 몰리면 자세가 제대로 정해지지 않는 degeneracy가 생긴다.

**학습한 photometric 모델.** 지금 direct SLAM의 photometric 모델은 단순한 affine brightness 보정과 고정된 camera response function이다. neural radiance field 계열 연구는 장면의 appearance를 신경망으로 모델링한다. Ch.14의 iMAP은 이렇게 학습한 지도에서 렌더링한 색·깊이와 관측의 차이로 카메라 자세를 추적했다.

RGB-D 카메라는 depth를 직접 재므로 밝기가 변해도 dense 재구성을 할 수 있다. Ch.9는 RGB-D SLAM을 다룬다.
