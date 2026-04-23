# Ch.3 — Structure from Motion: Longuet-Higgins에서 COLMAP까지

Harris와 Lowe가 이미지 안에서 "볼 만한 점"을 골라내는 방법을 다듬는 동안, 다른 계보는 그 점들이 두 장의 사진에 동시에 찍혔을 때 무엇을 알 수 있는가를 물었다. 특징을 *검출*하는 문제와 특징으로부터 *공간을 재구성*하는 문제는 같은 시기에 각자 발전했고, 2000년대 중반에야 하나의 파이프라인으로 합쳐졌다.

1981년 케임브리지 이론심리학자 H.C. Longuet-Higgins는 *Nature*에 세 페이지짜리 논문을 실었다. 제목은 "A Computer Algorithm for Reconstructing a Scene from Two Projections". 그는 두 장의 사진에 찍힌 같은 점들의 좌표 여덟 쌍만으로 카메라가 어떻게 움직였는지, 그리고 그 장면이 3차원에서 어떤 형태인지를 동시에 풀어낼 수 있음을 보였다. 로봇공학자도 컴퓨터 비전 연구자도 아니었다. 그 세 페이지에서 Structure from Motion(SfM)이 시작되었고, 2016년 Johannes Schönberger의 COLMAP이 나오면서야 그 수학이 공학으로 구현되었다.

---

## 3.1 Essential Matrix와 8-point Algorithm

Longuet-Higgins의 출발점은 단순했다. 두 카메라로 같은 점을 찍으면, 그 점의 이미지 좌표 쌍 사이에 대수적 제약이 존재한다. 좌표계를 정규화하면 이 제약은 행렬 하나로 집약된다. 그는 이것을 **essential matrix** $\mathbf{E}$로 정의했다.

두 카메라의 중심을 각각 $\mathbf{O}_1$, $\mathbf{O}_2$, 대응점을 정규화 좌표 $\mathbf{x}_1$, $\mathbf{x}_2$라 하면 제약은:

$$\mathbf{x}_2^\top \mathbf{E} \mathbf{x}_1 = 0$$

$\mathbf{E}$는 카메라 사이의 회전 $\mathbf{R}$과 이동 $\mathbf{t}$로부터 $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$로 인수분해된다. 여기서 $[\mathbf{t}]_\times$는 $\mathbf{t}$의 skew-symmetric 행렬이다.

Essential matrix는 스케일 모호성을 제거하면 자유도가 5이다. 그러나 5개 대응점으로 푸는 non-linear 5-point algorithm(Nistér 2004)이 등장하기 전까지, 표준 접근은 rank-2 제약과 단위 스케일 제약을 강제하기 전 단계에서 행렬을 9개 원소 중 스케일 1개를 고정해 8개의 미지수로 보고 8개 대응점으로 선형 시스템을 푸는 것이었다. 이것이 **8-point algorithm**이다. Longuet-Higgins 자신은 정확히 8개 점으로 유일해를 구하는 절차를 제시했다. 구현은 간단했고, 계산량도 작았다.

문제는 수치 안정성이었다. 이미지 좌표가 수백~수천 픽셀 단위이면 계수행렬의 원소 크기가 크게 달라져 SVD가 불안정해진다.

> 🔗 **차용.** Hartley는 1997년 정규화된 8-point algorithm에서 이미지 좌표를 평균 0, 평균 거리 $\sqrt{2}$로 선형 변환한 뒤 essential matrix를 추정하는 방식을 내놓았다. Longuet-Higgins의 기하학은 그대로 두고, 수치 조건만 고쳤다. 이후 모든 교과서가 이 정규화 버전을 표준으로 삼았다.

Fundamental matrix $\mathbf{F}$는 essential matrix의 일반화다. 카메라 내부 파라미터 $\mathbf{K}$를 알지 못해도 $\mathbf{x}_2^\top \mathbf{F} \mathbf{x}_1 = 0$이 성립한다. 두 카메라의 내부 파라미터를 각각 $\mathbf{K}_1$, $\mathbf{K}_2$라 하면 관계는 $\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \mathbf{K}_1^{-1}$이다. 같은 카메라로 찍은 경우($\mathbf{K}_1 = \mathbf{K}_2 = \mathbf{K}$)에는 $\mathbf{F} = \mathbf{K}^{-\top} \mathbf{E} \mathbf{K}^{-1}$로 단순화된다. SfM 파이프라인에서 $\mathbf{K}$를 모를 때는 $\mathbf{F}$를 먼저 추정하고, $\mathbf{K}$를 알 때는 $\mathbf{E}$를 직접 푼다.

---

## 3.2 Tomasi-Kanade Factorization

1981년 이후 십 년간 SfM은 주로 두 장 사진 사이의 기하학으로 연구되었다. 여러 장 사진을 동시에 처리하는 방법은 별도의 문제였다. 1992년 Carlo Tomasi와 Takeo Kanade가 CMU에서 **factorization method**를 발표하면서 이 문제의 윤곽이 드러났다.

아이디어는 다음과 같다. $F$장의 프레임, $P$개의 포인트를 관측한다면, 이미지 좌표를 $2F \times P$ 행렬 $\mathbf{W}$로 쌓을 수 있다. 각 원소 $w_{fp}$는 $f$번째 프레임에서 $p$번째 포인트의 좌표다. orthographic(scaled orthographic) 카메라 모델 아래에서 $\mathbf{W}$는 rank 3 행렬이다. 원 논문(Tomasi & Kanade 1992)은 정확히 이 가정에서 출발했다. 그러면:

$$\mathbf{W} = \mathbf{M} \mathbf{S}$$

여기서 $\mathbf{M}$은 $2F \times 3$ 모션 행렬, $\mathbf{S}$는 $3 \times P$ 구조 행렬이다. SVD로 $\mathbf{W}$의 상위 3개 특이값만 유지하면 $\mathbf{M}$과 $\mathbf{S}$를 동시에 얻는다.

한 번의 SVD로 모든 프레임의 모션과 모든 포인트의 3D 위치를 한꺼번에 추정한다는 점이 핵심이었다. 계산 복잡도는 $O(F \cdot P)$로 가볍고, 구현이 쉬웠다.

> 🔗 **차용.** Nistér는 2004년 Visual Odometry 논문에서 Tomasi-Kanade의 batch factorization을 실시간으로 축소했다. 모든 프레임을 모으는 대신 sliding window 안의 몇 프레임만 factorize하거나 five-point algorithm으로 두 프레임씩 처리했다. batch 정확도 대신 latency를 택했다.

한계는 orthographic/affine 가정에 있었다. Affine 카메라는 원근 왜곡(perspective distortion)을 무시한다. 이 모델은 장면의 깊이 변화가 카메라까지의 거리에 비해 충분히 작을 때(즉 원거리 소물 촬영)에만 유효하다. 카메라와 가까운 장면, 시야각이 넓은 렌즈, 혹은 전경·배경 깊이 차이가 큰 환경에서는 오차가 컸다. 1990년대 후반부터 perspective camera로의 확장이 여러 방향에서 시도되었고, 이는 bundle adjustment의 재발견으로 이어졌다.

---

## 3.3 Hartley & Zisserman과 정전(canon)화

Tomasi-Kanade의 factorization이 multiple view 문제의 틀을 잡았다면, 남은 과제는 perspective camera로의 확장과 흩어진 수학을 하나의 언어로 묶는 일이었다.

2000년 Richard Hartley와 Andrew Zisserman의 교과서 *Multiple View Geometry in Computer Vision*이 나왔다. 680쪽. 1981년부터 1990년대까지 여기저기 흩어진 SfM 수학을 사영기하(projective geometry)의 언어로 통합했다.

Hartley & Zisserman이 한 것은 단순 정리가 아니었다. essential matrix, fundamental matrix, homography, camera calibration, bundle adjustment를 모두 사영기하의 단일 프레임워크에서 끌어냈다. 각자 따로 돌던 개념들이 같은 뿌리에서 나온다는 것이 처음으로 명확해졌다.

bundle adjustment는 이 책에서 특히 무게 있게 다뤘다. Triggs et al.(1999)이 Ch.1에서 정식 도입한 reprojection error 최소화 문제를, Hartley & Zisserman은 사영기하 프레임워크 안에 놓고 *robust cost function* $\rho$를 명시적으로 얹었다. outlier가 섞인 실제 데이터에서 최적화가 무너지지 않도록 Huber나 Cauchy 함수로 오차를 눌렀다. Levenberg-Marquardt로 풀되, Jacobian의 희소 구조를 써서 계산량을 줄였다.

2000년대 초반 SLAM·VO 논문 대부분이 이 교과서를 표준 참조로 달았다. 개념 정의가 이 책 하나로 통일되면서, Photo Tourism 같은 대규모 응용은 개념 재정의 없이 구현에 집중할 수 있었다.

---

## 3.4 Photo Tourism과 Bundler: 인터넷 규모의 SfM

2006년 Noah Snavely, Steven Seitz, Richard Szeliski는 SIGGRAPH 논문 "[Photo Tourism](https://doi.org/10.1145/1179352.1141964)"을 발표했다. 인터넷에 업로드된 관광지 사진들(피렌체 두오모, 로마 트레비 분수)을 모아서 3D 재구성을 시도했다.

설정 자체가 도전이었다. 카메라도 날씨도 구도도 제각각이었고, 일부 사진은 관계없는 실내 컷이 섞여 있었다. 체계적으로 촬영한 데이터셋이 아니라, 수천 명이 아무 순서 없이 올린 이미지들이었다.

Snavely의 파이프라인은 다음 순서로 작동했다. SIFT 특징 검출과 매칭으로 이미지 쌍 사이의 대응점을 찾는다. Fundamental matrix로 기하적으로 불일치하는 매칭을 RANSAC으로 제거한다. 연결성이 높은 이미지 쌍부터 시작해 카메라를 하나씩 추가하는 incremental SfM을 수행한다. 카메라를 추가할 때마다 bundle adjustment로 전체 포즈와 포인트를 재최적화한다.

Flickr 관광객 사진 1,000~2,000장으로 로마·피렌체 성당 외관을 수십만 포인트의 sparse point cloud로 뽑아냈다. 통제되지 않은 인터넷 이미지로 이 규모를 재구성한 시도는 이전에 없었다.

이 파이프라인의 구현체가 Bundler였다. Snavely가 오픈소스로 풀었고, SfM 연구자들의 기본 출발점이 되었다.

<!-- DEMO: sfm_incremental.html -->

---

## 3.5 COLMAP: 공학적 성숙

> 📜 **예언 vs 실제.** Snavely et al. 2006 논문은 향후 과제로 수십만 장 규모 확장, 온라인 처리, 동적 장면 처리를 꼽았다. 수십만 장 처리는 COLMAP(Schönberger 2016)과 OpenSfM이 달성했다. 온라인 처리는 SLAM이 별도 계보로 답했다—SfM의 incremental refinement 대신 fixed-lag smoother와 loop closure로. 동적 장면 처리는 2026년 기준 여전히 열린 문제다. 세 예측 중 하나는 SfM이, 하나는 다른 분야가, 하나는 아무도 풀지 못했다. `[진행형]`

2016년 Johannes Schönberger와 Jan-Michael Frahm은 CVPR 논문 "[Structure-from-Motion Revisited](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)"를 발표했다. 제목의 "Revisited"는 겸손한 표현이었지만, 실제로는 Bundler 이후 십 년간 쌓인 개선들을 체계적으로 묶은 재설계였다.

COLMAP이 Bundler와 가장 크게 달라진 점은 세 곳이다.

첫째, 카메라 추가 순서. Bundler는 연결성이 높은 쌍부터 시작했지만, 어떤 쌍을 먼저 확장할지에 대한 체계적 기준이 없었다. COLMAP은 초기 이미지 쌍 선택과 카메라 등록 순서를 triangulation angle, feature track 길이, visibility score 기반으로 자동화했다. 재구성의 안정성이 크게 높아졌다.

둘째, bundle adjustment 주기. 매 카메라 추가 후 full bundle adjustment는 비용이 크다. COLMAP은 local bundle adjustment(최근 추가된 카메라와 공유 포인트가 많은 카메라들만 묶어서 최적화)와 주기적 global bundle adjustment를 교대하는 방식을 도입했다.

셋째, 기하적 검증. 매칭된 특징점 쌍에 대해 fundamental matrix와 homography 두 모델로 각각 RANSAC을 돌린다. Fundamental matrix는 일반적인 비평면 장면, homography는 평면 장면이나 순수 회전을 모델링한다. COLMAP은 두 모델의 inlier 수를 비교해 장면 유형을 판별하고, 어느 쪽에도 들어오지 않는 매칭을 걸러낸다. 불량 매칭과 평면-퇴화(planar degeneracy) 상황에서 Bundler보다 버텼다.

> 🔗 **차용.** COLMAP의 incremental bundle adjustment 전략은 Snavely의 Bundler 파이프라인을 모듈화하고 각 단계의 품질 관리를 추가한 것이다. 알고리즘의 핵심 수학(essential matrix 추정, triangulation, Levenberg-Marquardt)은 Hartley & Zisserman 교과서의 것이다. COLMAP의 기여는 새 수학보다 엔지니어링 판단의 체계화에 있었다.

COLMAP이 사실상 표준이 된 것은 성능 때문만이 아니었다. 코드베이스가 정돈되어 있었고, 문서도 충분했으며, CUDA 가속으로 수천 장도 시간 내에 돌렸다. 2020년 NeRF가 나온 뒤 모든 NeRF 학습 코드가 COLMAP 출력(카메라 포즈 + sparse point cloud)을 입력으로 받았다. 3D Gaussian Splatting도 마찬가지였다. COLMAP은 SfM 도구이기 이전에 3D 재구성 연구의 입구가 되었다.

---

## 3.6 SfM과 SLAM의 분화

SfM과 SLAM은 같은 수학을 쓰면서도 근본적으로 다른 문제를 푼다. 이 구분이 뚜렷해진 것은 2000년대 초였다.

SfM은 *오프라인*이다. 모든 이미지를 수집한 뒤 처리하므로 시간 제약이 없고, 전체 데이터를 반복 참조하면서 global bundle adjustment를 여러 번 돌릴 수 있다. 카메라 포즈가 틀렸으면 되돌아가 다시 계산하면 된다.

SLAM은 *온라인*이다. 센서 데이터가 실시간으로 유입되고, 현재 시점의 로봇 위치를 그 자리에서 내놓아야 한다. 과거 데이터를 무한정 참조할 수 없으며, 지도가 자라면서 계산량이 커지고, 루프를 돌아 처음 방문한 장소로 돌아왔을 때 accumulated drift를 교정해야 한다.

두 분야가 가장 크게 갈리는 지점은 루프 클로저다. SfM에서는 global bundle adjustment가 모든 불일치를 정리한다. SLAM에서는 루프가 닫히는 순간을 탐지해서 그 시점의 drift를 국소적으로 교정해야 한다. 이를 위한 기법(visual place recognition, pose graph optimization, covisibility-based local optimization)은 SfM에 존재하지 않는 SLAM 고유의 문제였다.

불확실성 전파도 달랐다. SLAM은 현재 포즈의 불확실성을 실시간으로 추적하고 새 관측마다 갱신한다. EKF나 factor graph 형태의 probabilistic 표현이 필요하다. SfM에서는 최적화가 끝난 뒤 covariance를 사후에 계산하면 되고, 실시간 추적은 필수가 아니다.

Davison의 MonoSLAM(2003)은 스스로를 "real-time SfM"으로 불렀다. 그러나 EKF 상태벡터에 카메라 포즈와 landmark를 함께 유지하는 구조는 SfM의 global batch와 달랐다. 2000년대를 거치며 두 분야는 각자의 문제 설정을 가진 독립된 계보로 갈라졌다.

---

## 3.7 🧭 아직 열린 것

**동적 물체 포함 SfM.** COLMAP을 포함한 모든 현행 SfM 시스템은 static world를 가정한다. 장면의 모든 포인트가 움직이지 않는다는 전제로 bundle adjustment를 풀기 때문에, 자동차나 보행자가 많은 장면에서는 오염된 매칭이 최적화를 왜곡한다. RANSAC이 일부를 걸러내지만 근본적인 해결은 아니다. Dynamic SfM을 위한 세그멘테이션 통합, 물체별 독립 모션 추정 등 연구가 진행 중이나, COLMAP 수준의 범용 구현체는 2026년 기준 없다.

**SfM과 SLAM의 경계 흐려짐.** 2023년 DUSt3R(Wang et al.)는 사전 훈련된 네트워크 하나로 이미지 두 장을 받아 dense point map과 카메라 포즈를 동시에 냈다. 특징점 매칭도 RANSAC도 bundle adjustment 초기화도 거치지 않았다. MASt3R(2024)로 확장되면서 수십 장 재구성도 됐다. 전통적인 SfM 파이프라인의 각 모듈이 하나씩 대체되고 있다. COLMAP이 NeRF·3DGS의 입구였다면, DUSt3R 류는 그 입구마저 바꾸려 한다. 이 패러다임이 COLMAP을 실질적으로 밀어낼지, 특정 도메인에서만 이길지는 아직 모른다.

---

SfM이 정밀한 오프라인 재구성을 다듬는 동안, 다른 쪽에서는 전혀 다른 질문이 쌓이고 있었다. 사진이 아니라 움직이는 로봇이다. 이미지가 아직 수집되지 않았다. 포즈 추정을 지금 당장 내놓아야 한다. Randall Smith와 Peter Cheeseman이 1986년에 던진 질문 — 불확실한 공간관계를 어떻게 전파하는가 — 이 그 압박 아래서 SLAM이라는 별개의 분야로 자랐다.
