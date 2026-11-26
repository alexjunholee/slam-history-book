# 11장 직접법: DTAM에서 DSO까지

2011년 Richard Newcombe는 소비자용 GPU 한 장만으로 화면의 모든 픽셀을 실시간 추적·복원하는 시스템 DTAM을 내놓았다. 당시 Newcombe는 Imperial College London에서 Andrew Davison의 지도를 받던 박사과정생이었고, 스승 Davison의 MonoSLAM은 EKF로 100개 안팎의 희소한 점만 추적하는 데도 벅찼다.

ORB-SLAM 계열의 특징점 기반(feature-based) 방법은 먼저 영상에서 코너를 검출해 ORB 서술자를 뽑고, 그중 1,000~2,000개의 점만 남긴 채 나머지 픽셀은 버린다. 반면 직접법(direct method)은 중간 요약 단계를 건너뛰고 픽셀 밝기 값 자체를 측정값으로 삼아 기하 오차가 아닌 밝기 차이(photometric error)를 최소화한다. 화면의 모든 픽셀을 다 쓰는 dense 방법부터 경계 근처만 취하는 semi-dense, 밝기 기울기가 큰 점 몇천 개만 고르는 sparse direct 방법까지 스펙트럼은 넓지만 뿌리는 하나다.

Imperial College의 Newcombe가 모든 픽셀을 밀어붙이던 바로 그 무렵, 뮌헨 공대(TUM)의 Daniel Cremers 그룹은 컴퓨터 비전의 고전인 Gauss-Newton 영상 정렬과 optical flow 기법을 SLAM으로 옮겨 심고 있었다. Cremers의 제자 Jakob Engel은 2014년 LSD-SLAM으로 GPU 없는 실시간 semi-dense SLAM을 열었고, 2016년에는 DSO로 정밀한 photometric calibration을 결합한 sparse direct odometry의 정점을 찍었다. 같은 직접법 안에서도 쓰는 픽셀의 밀도와 설계 철학이 극적으로 달라지는 과정이었다.

---

## 11.1 모든 픽셀: DTAM

2011년 ICCV에서 Newcombe, Lovegrove, Davison이 발표한 [DTAM](https://doi.org/10.1109/ICCV.2011.6126513)은 "Dense Tracking and Mapping in Real-Time"이라는 이름 그대로 화면의 모든 픽셀을 남김없이 동원해 추적과 지도 작성을 실시간으로 해냈다.

파이프라인은 추적과 지도 작성 두 축이 맞물려 돈다. 추적 단계에서는 지금까지 쌓은 dense 모델에서 합성해 낸 영상에 현재 카메라 프레임 전체를 photometric alignment로 직접 맞춘다. 특징점 검출이나 서술자 매칭 없이 오직 픽셀 밝기 차이만 줄여 자세를 찾는다. 지도 작성 단계에서는 keyframe 근처에서 찍은 여러 프레임의 photometric 오차를 깊이 후보별로 쌓아 3차원 배열인 cost volume을 만든다. 이어 여기에 공간적 매끄러움을 강제하는 regularization 항을 얹어 에너지를 구성하고, 이 에너지를 최소화해 keyframe의 inverse depth map을 복원한다.

$$E(\mathbf{u}) = \frac{1}{|\mathcal{I}|}\sum_{i \in \mathcal{I}} \Bigl\| I_r\bigl(\pi(\mathbf{p}(\mathbf{u}))\bigr) - I_i\bigl(\pi(KT_i\mathbf{p}(\mathbf{u}))\bigr) \Bigr\|_1 + \lambda \, g\,\|\nabla \mathbf{u}\|_\epsilon$$

여기서 $\mathbf{u}$는 역 깊이(inverse depth) 맵, $\mathbf{p}(\mathbf{u})$는 $\mathbf{u}$로 역투영한 3D 점, $K$는 카메라 내부 행렬, $T_i$는 참조 프레임 기준 $i$번 프레임의 rigid body 변환, $\pi$는 원근 투영이다. 데이터 항이 각 프레임의 photometric 오차를 L1 norm으로 재어 평균한다면, 두 번째 항의 $\|\nabla \mathbf{u}\|_\epsilon$은 inverse depth gradient에 Huber norm을 씌우고 참조 영상의 gradient가 큰 경계에서 작아지는 가중치 $g$를 곱한 것이다. Huber norm은 변화가 작은 곳에서는 제곱 norm으로 표면을 매끈하게 다듬고, 변화가 큰 곳에서는 L1 norm(total variation)으로 전환해 물체의 깊이 경계가 뭉개지지 않게 지켜 준다. 물론 픽셀 수십만 개에 대해 이런 비볼록 에너지를 매 프레임 풀려면 막대한 병렬 연산이 필요했고, 논문에서는 당시 최신 GPU였던 NVIDIA GTX 480 한 장에 기대어 실시간 성능을 끌어냈다.

DTAM은 단안 카메라만으로도 실내 책상 위 물체들을 실시간으로 빈틈없이 복원해 보여 주었다. 하지만 밝기 가정에 민감해 조명이 바뀌면 쉽게 흔들렸고, 시연 무대 역시 조명이 통제된 작은 실내 공간을 벗어나지 못했다.

<!-- DEMO: dtam_photometric_residual.html -->

> 🔗 **차용.** 같은 해 Newcombe가 Microsoft Research와 함께 내놓은 [KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378)(ISMAR 2011)은 능동 depth 센서로 조밀한 재구성을 해냈으며, 지도 표현으로 [Curless & Levoy 1996](https://doi.org/10.1145/237170.237269)의 TSDF(truncated signed distance function)를 가져왔다. DTAM은 depth 센서 없이 단안 카메라의 밝기 정보만으로 그에 필적하는 dense depth map을 추정하려 한 도전이었다.

---

## 11.2 Semi-dense: LSD-SLAM

2014년 ECCV에서 Engel, Schöps, Cremers가 발표한 [LSD-SLAM](https://doi.org/10.1007/978-3-319-10605-2_54)(Large-Scale Direct Monocular SLAM)은 모든 픽셀을 다 쓰겠다는 욕심을 내려놓는 대신, GPU 없이 CPU만으로도 실시간으로 동작하는 큰 규모의 단안 SLAM을 겨냥했다. 핵심 아이디어는 **semi-dense** 표현이었다. 텍스처가 없는 밋밋한 벽면은 어차피 밝기 변화가 없어 자세 추정에 아무런 정보를 주지 못하므로 버리고, 이미지에서 밝기 gradient가 임계값 이상인 경계(edge) 근처 픽셀만 골라 남긴 것이다. 물론 여기에도 코너 검출기는 들어가지 않는다.

추적에는 SE(3) direct image alignment를 쓴다. 현재 프레임을 keyframe에 warping한 뒤 photometric residual을 Gauss-Newton으로 줄여 나간다. 지도는 각 keyframe에 매달린 semi-dense depth map들의 집합이며, keyframe 사이를 7자유도 Sim(3) 포즈 그래프로 묶어 단안 카메라 특유의 scale drift까지 함께 보정한다. 큰 루프를 돌고 돌아왔을 때는 appearance 기반 검색(FAB-MAP)으로 후보를 찾은 뒤, 두 프레임 사이에 양방향으로 Sim(3) 정렬을 따로 돌려 두 결과가 통계적으로 일치할 때만 loop closure를 수락했다.

tracking과 mapping을 두 스레드로 나눈 골격은 PTAM과 닮았지만, ORB나 BRIEF 같은 서술자를 일절 쓰지 않고 오직 픽셀 밝기만으로 일관되게 시스템을 짰다는 점이 달랐다. 덕분에 LSD-SLAM은 실외의 넓은 거리까지 semi-dense 지도로 엮어 낼 수 있었다. 하지만 밝기 일관성이라는 근본 가정의 한계는 피하지 못해, 터널에 진입하거나 창문 역광을 마주하거나 플래시가 터지는 순간에는 추적이 급격히 불안정해졌다.

<!-- DEMO: lsd_slam_semidense.html -->

> 🔗 **차용.** LSD-SLAM의 frontend를 떠받치는 Gauss-Newton photometric registration은 영상 정렬 분야의 오랜 고전이다. [Lucas & Kanade 1981](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf) tracker와 연산 효율을 극대화한 inverse compositional 변형([Baker & Matthews 2004](https://doi.org/10.1023/B:VISI.0000011205.11775.fd))이 그 직계 조상이다.

---

## 11.3 Sparse direct: DSO

Engel, Koltun, Cremers의 [DSO](https://doi.org/10.1109/TPAMI.2017.2658577)(Direct Sparse Odometry)는 2016년 arXiv에 먼저 공개되고 2018년 IEEE TPAMI에 실렸다. 흥미롭게도 DSO는 LSD-SLAM보다 픽셀을 *더 적게* 쓰면서 오히려 정확도를 크게 끌어올렸다. 공간적 매끄러움을 강제하는 regularization을 과감히 덜어 내고, 그 연산 여유를 정교한 photometric calibration과 결합 최적화에 쏟아부은 덕분이다.

DSO는 새 keyframe이 들어올 때마다 영상 전체에 고르게 퍼지도록 후보 픽셀 2,000개를 뽑고, 전체 활성 윈도우를 통틀어 최적화에 남기는 활성 점 수도 2,000개 안팎으로 묶어 둔다. 점은 주변보다 gradient가 두드러진 픽셀 위주로 고른다. 이 점들을 모아 sliding window bundle adjustment로 camera pose, inverse depth, 프레임별 affine brightness 파라미터 $(a_i, b_i)$를 동시에 푼다. 윈도우를 빠져나가는 옛 프레임은 Schur complement를 이용한 marginalization으로 소거해 최적화 문제의 크기를 항상 일정하게 유지한다.

DSO를 차별화한 결정적 장치는 카메라의 영상 생성 과정을 물리적으로 쪼개어 모델링한 photometric calibration이었다. 렌즈 가장자리가 어두워지는 vignetting과 센서가 빛을 비선형으로 기록하는 camera response function(gamma curve)은 사전에 캘리브레이션해 선형 밝기 공간으로 되돌려 놓고, 프레임별 노출 시간 $t_i$는 메타데이터로 넘겨받은 뒤 남은 미세한 밝기 변화만 실시간 affine 파라미터 $(a_i, b_i)$로 추정했다. 이를 반영한 잔차식은 다음과 같다.

$$E_{pj} = \sum_{\mathbf{p} \in \mathcal{N}_p} w_{\mathbf{p}} \left\| \left( I_j\!\left[\mathbf{p}'\right] - \frac{t_j e^{a_j}}{t_i e^{a_i}} I_i[\mathbf{p}] - \left(b_j - \frac{t_j e^{a_j}}{t_i e^{a_i}} b_i\right) \right) \right\|_\gamma$$

여기서 $t_i, t_j$는 노출 시간, $(a_i, b_i)$와 $(a_j, b_j)$는 각 프레임의 gain과 bias에 해당하는 affine brightness 파라미터, $\|\cdot\|_\gamma$는 Huber loss다. 위 식은 전처리에서 vignetting과 response function을 이미 보정한 밝기 영상에 적용된다.

논문에서는 공정한 순수 odometry 비교를 위해 loop closure와 relocalization을 끈 단안 ORB-SLAM을 맞상대로 세웠다. 그 결과 photometric calibration이 제공된 TUM monoVO와 합성 데이터셋 ICL-NUIM에서는 DSO가 정확도와 강건성 모두에서 ORB-SLAM을 앞섰고, EuRoC에서는 ORB-SLAM이 더 낮은 오차를 내는 대신 추적을 더 자주 놓쳤다. 저자들은 그 까닭으로 EuRoC에 photometric calibration이 없다는 점을 지목했다. 이와 함께 드론이 작은 루프를 맴돌며 왔던 길을 되짚는 구간이 많아, ORB-SLAM의 local mapping이 사실상 작은 루프들을 암묵적으로 닫아 준 효과도 꼽았다. 실제로 ORB-SLAM이 10초 넘게 안 보인 keypoint를 다시 매칭하지 못하게 제한하자 두 방법의 정확도는 비슷해졌고 강건성은 여전히 DSO 쪽이 높았다.

> 📜 **예언 vs 실제.** DSO가 요구한 사전 photometric calibration은 실용화의 높은 문턱이었다. 그러나 2018년 Bergmann, Wang, Cremers가 RA-L에 [online photometric calibration](https://doi.org/10.1109/LRA.2017.2777002)을 발표하면서 사정이 달라졌다. KLT로 추적한 대응점의 밝기 변화를 단서로 노출 시간, response function, vignetting을 주행 중에 실시간 추정해 프레임을 즉시 보정하는 기법이다. 실제로 TUM monoVO에서 이 온라인 보정을 거친 auto-exposure 영상으로 DSO를 돌렸을 때, 미리 정밀 캘리브레이션을 마친 영상과 맞먹는 성능이 나왔다.

> 📜 **예언 vs 실제.** DTAM 이후 조밀 재구성 연구의 주력은 KinectFusion처럼 능동 RGB-D 센서를 쓰는 쪽으로 빠르게 이동했고, 단안 영상만으로 dense 지도를 복원하는 흐름은 2020년대 NeRF와 3DGS가 등장한 뒤에야 다시 본격적으로 불붙었다.

> 🔗 **차용.** DSO의 photometric calibration 틀은 [Debevec & Malik 1997](https://doi.org/10.1145/258734.258884)의 고전적인 HDR 영상 복원 연구에서 빌려 왔다. 비선형 response function을 미리 풀고 실행 중에는 프레임별 affine 밝기 변화만 쫓는 분리 전략이 거기서 나왔다.

---

## 11.4 VI-DSO와 후속 연구

direct 방법의 아킬레스건은 조명이 갑자기 출렁이는 순간에 추적이 쉽게 무너진다는 점이었다. 2018년 ICRA에서 von Stumberg, Usenko, Cremers는 DSO에 관성 센서를 결합한 [VI-DSO](https://doi.org/10.1109/ICRA.2018.8462905)를 내놓아 이 약점을 정면으로 보강했다. 단안 DSO의 sliding window photometric bundle adjustment에 IMU preintegration factor를 나란히 꽂아 넣자, 시각 정보가 흔들리는 구간에서도 IMU가 자세를 붙잡아 주어 실패율이 떨어졌을 뿐만 아니라 단안 카메라의 오랜 숙제였던 절대 scale까지 한꺼번에 복원됐다.

Cremers 그룹은 이어 2022년 [DM-VIO](https://doi.org/10.1109/LRA.2021.3140129)에서 delayed marginalization을 더해 direct VIO의 초기화 안정성을 한층 끌어올렸고, 같은 그룹의 [Basalt](https://arxiv.org/abs/1904.06504)(RA-L 2020)는 direct photometric BA 대신 patch 기반 optical flow로 점을 추적하는 또 다른 갈래를 개척했다. 이렇게 2010년대 후반과 2020년대 초반에는 direct VIO와 feature 기반 VIO(VINS-Mono, OpenVINS)가 팽팽히 맞서며 나란히 진화했다.

> 🔗 **차용.** VI-DSO에 들어간 IMU factor 역시 [Forster et al. 2017. On-Manifold Preintegration (IEEE TRO)](https://doi.org/10.1109/TRO.2016.2597321)의 공식을 그대로 채택했다.

---

## 11.5 direct 방법의 한계

직접법은 서술자라는 중간 요약을 덜어 내고 영상 밝기 자체를 잔차에 넣으면서 픽셀 활용 범위에 따라 다양하게 변주됐다. DTAM은 전체 픽셀을, LSD-SLAM은 기울기가 큰 semi-dense 영역을 취했고, DSO는 고르게 분산된 2,000여 점을 쓰되 텍스처가 적은 영역에서는 임계값을 낮춰 약한 기울기까지 포착했다. 덕분에 코너가 드문 장면에서도 정보를 쥐어짜 낼 수 있었지만, 아예 밝기가 고른 민무늬 벽면 앞에서는 카메라가 움직여도 밝기 잔차가 꿈쩍하지 않는다는 근본적 한계가 여전했다.

**Photometric calibration.** DSO가 전제하는 vignetting·response curve·노출 시간 정보는 일반 소비자용 카메라에서 손쉽게 얻기 어렵다. 스마트폰 카메라는 내부 파이프라인에서 HDR 합성, auto-exposure, 실시간 화이트 밸런스를 제조사만 아는 규칙으로 수시로 돌리기 때문에 DSO의 물리적 밝기 모델이 그대로 들어맞지 않는다.

**조명 변화.** 직접법은 같은 3D 점이 프레임이 바뀌어도 같은 밝기로 찍힌다는 photometric consistency 가정 위에 서 있다. DSO의 affine brightness 모델이 완만한 노출 변화는 흡수해 주지만, 구름이 지나가거나 실내 형광등이 깜빡이고 역광이 들이치는 급격한 변화 앞에서는 가정이 곧바로 깨진다.

**카메라 교체.** 통제된 벤치마크에서는 DSO가 ORB-SLAM을 앞서는 구간도 있었으나, 현장 투입의 문턱은 달랐다. ORB-SLAM은 카메라를 바꿔 끼워도 내장 파라미터만 알면 곧바로 돌릴 수 있는 반면, DSO는 카메라를 바꿀 때마다 vignetting과 response curve를 새로 캘리브레이션해야 하는 번거로움이 뒤따랐다.

**학습 기반 feature.** 설상가상으로 2010년대 후반 이후 [SuperPoint](https://arxiv.org/abs/1712.07629)(2018) 같은 학습 기반 특징점과 [LightGlue](https://arxiv.org/abs/2306.13643)(2023) 같은 학습 기반 매처가 등장하면서 특징점 진영이 크게 반격했다. 이 기법들은 극심한 조명·시점 변화에서도 대응점을 튼튼하게 잡아내는 동시에, 기존의 검증된 서술자 매칭 파이프라인에 그대로 얹힐 수 있었다.

<!-- DEMO: photometric_calibration_demo.html -->

---

## 아직 열린 문제

**조명이 급변할 때의 direct tracking.** photometric consistency가 무너지는 순간을 어떻게 버텨 낼지는 아직 완전히 풀리지 않았다. VI-DSO처럼 IMU를 붙이면 급격한 흔들림은 버틸 수 있고, Bergmann 등의 online photometric calibration처럼 주행 중에 밝기 모델을 추정하는 보완책도 있지만 근본적인 취약성은 남아 있다.

**텍스처가 없는 장면.** 특징점 방법이 코너 없는 벽 앞에서 길을 잃듯, 직접법에서도 밝기 변화가 거의 없는 흰 벽이나 균질한 복도·창고·평원에서는 자세 변화에 따른 잔차 기울기가 사라진다. 특히 LSD-SLAM처럼 밝기 기울기가 큰 픽셀만 취하는 방식에서는 그 픽셀들이 화면 한쪽 구석에 몰릴 경우 자세가 풀리지 않는 기하적 퇴화(degeneracy)가 발생한다.

**학습한 photometric 모델.** 고전 direct SLAM이 쓰는 photometric 모델은 고정된 camera response function과 단순한 affine 보정 수준에 머물러 있다. 최근 neural radiance field 계열 연구는 장면의 외관과 조명 반응을 신경망 자체로 모델링하며 이 틀을 깨고 있다. 예컨대 iMAP은 신경망으로 학습한 지도에서 렌더링한 색·깊이와 실제 관측을 비교해 카메라 자세를 추적했다.

한편 단안 카메라의 밝기 변화에 매달리는 대신, 아예 깊이 값을 직접 재는 RGB-D 센서를 손에 쥐면 조명이 흔들려도 훨씬 안정적으로 빈틈없는 3D 표면을 쌓아 올릴 수 있다. 다음 장에서는 DTAM과 같은 해에 발표되어 실내 3D 재구성의 판도를 바꾼 KinectFusion과 그 후예들을 살펴본다(→ 12장).