# Ch.3 — Structure from Motion: Longuet-Higgins에서 COLMAP까지

Harris와 Lowe가 이미지에서 추적하기 좋은 점을 고르는 방법을 다듬는 동안, 다른 연구자들은 같은 점이 찍힌 사진 두 장에서 그 점의 3D 위치와 카메라 움직임을 알아내는 방법을 연구했다. 특징을 *검출*하는 연구와 특징으로 *공간을 재구성*하는 연구는 같은 시기에 따로 발전했고, 2000년대 중반에 하나의 파이프라인으로 합쳐졌다.

1981년 서식스 대학교 실험심리학 연구실의 H.C. Longuet-Higgins는 *Nature*에 세 쪽짜리 논문 "[A Computer Algorithm for Reconstructing a Scene from Two Projections](https://cseweb.ucsd.edu/classes/fa01/cse291/hclh/SceneReconstruction.pdf)"를 실었다. 두 사진에 찍힌 같은 점 여덟 개의 좌표만 있으면 연립 일차방정식을 풀어 두 시점의 상대 방향과 장면 구조를 구할 수 있다는 내용이었다. 사진측량과 운동 지각(motion perception) 연구에는 이보다 앞선 결과가 있었다. 그래도 현대 컴퓨터 비전의 two-view geometry는 이 논문에서 시작한다고 보는 경우가 많다.

---

## 3.1 Essential Matrix와 8-point Algorithm

Longuet-Higgins의 출발점은 단순했다. 두 카메라로 같은 점을 찍으면 두 이미지 좌표 사이에 대수 관계가 생긴다. 카메라 내부 파라미터로 좌표를 정규화하면 이 관계를 3×3 행렬 하나로 쓸 수 있다. 논문은 이 행렬을 $Q$라고 썼고, 지금은 **essential matrix** $\mathbf{E}$라고 부른다.

대응점의 정규화 좌표를 $\mathbf{x}_1$, $\mathbf{x}_2$라 하면 다음 식이 성립한다:

$$\mathbf{x}_2^\top \mathbf{E} \mathbf{x}_1 = 0$$

$\mathbf{E}$는 두 카메라 사이의 회전 $\mathbf{R}$과 이동 $\mathbf{t}$로 $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$로 쓴다. 여기서 $[\mathbf{t}]_\times$는 $\mathbf{t}$의 skew-symmetric 행렬이다.

스케일을 빼면 essential matrix의 자유도는 5다. 대응점 5개로 푸는 비선형 5-point algorithm([Nistér 2004](http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/2004-PAMI-David%20Nister-An%20Efficient%20Solution%20to%20the%20Five-Point%20Relative%20Pose%20Problem.pdf))이 나오기 전에는 다른 방법을 주로 썼다. 행렬의 원소 9개 가운데 스케일 하나를 고정해 미지수 8개로 보고, rank-2 같은 제약은 나중에 맞춘 채 대응점 8개로 선형 연립방정식을 푼다. 이것이 **8-point algorithm**이다. 논문에 따르면 사진측량에서는 점 다섯 개로 두 시점의 관계를 정할 수 있었지만, 그러려면 3차 연립방정식 다섯 개를 반복 계산으로 풀어야 했다. Longuet-Higgins는 점을 여덟 개로 늘리는 대신 연립 일차방정식만 풀면 되게 했다. 구현이 간단했고 계산량도 작았다.

약점은 수치 안정성이었다. 이미지 좌표가 수백~수천 픽셀 단위이면 계수 행렬의 원소 크기가 서로 크게 달라져 SVD(특이값 분해) 결과가 불안정해진다.

> 🔗 **차용.** Hartley는 1997년 정규화된 8-point algorithm([In Defense of the Eight-Point Algorithm](https://www.cse.unr.edu/~bebis/CS485/Handouts/hartley.pdf))을 내놓았다. 이미지마다 점들의 평균 위치가 원점에 오도록 옮기고, 원점까지 평균 거리가 $\sqrt{2}$가 되도록 크기를 맞춘다. 이 좌표로 fundamental matrix를 추정한 뒤 원래 좌표계로 되돌린다. 앞에서 말한, 카메라 내부 파라미터로 좌표를 바꾸는 정규화와는 목적이 다르다. Hartley는 Longuet-Higgins의 기하를 그대로 두고 수치 조건만 고쳤다. 이 절차는 그 뒤 다중 시점 기하 교재와 구현에 표준으로 들어갔다.

Fundamental matrix $\mathbf{F}$는 essential matrix를 일반화한 행렬이다. 카메라 내부 파라미터 $\mathbf{K}$를 알지 못해도 $\mathbf{x}_2^\top \mathbf{F} \mathbf{x}_1 = 0$이 성립한다. 두 카메라의 내부 파라미터를 각각 $\mathbf{K}_1$, $\mathbf{K}_2$라 하면 관계는 $\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \mathbf{K}_1^{-1}$이다. 같은 카메라로 찍은 경우($\mathbf{K}_1 = \mathbf{K}_2 = \mathbf{K}$)에는 $\mathbf{F} = \mathbf{K}^{-\top} \mathbf{E} \mathbf{K}^{-1}$가 된다. SfM 파이프라인은 $\mathbf{K}$를 알면 $\mathbf{E}$를 바로 풀고, 모르면 $\mathbf{F}$를 먼저 추정한다.

---

## 3.2 Tomasi-Kanade Factorization

1981년 뒤 십 년 동안 SfM 연구는 주로 사진 두 장 사이의 기하를 다뤘다. 1992년 Carlo Tomasi(Cornell)와 Takeo Kanade(CMU)는 *IJCV*에 여러 프레임을 한꺼번에 처리하는 **[factorization method](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/TomasiC_Shape%20and%20motion%20from%20image%20streams%20under%20orthography.pdf)**를 발표했다.

프레임 $F$장에서 점 $P$개를 관측하면 이미지 좌표를 $2F \times P$ 행렬 $\mathbf{W}$로 쌓을 수 있다. 각 행에서 그 프레임의 점 좌표 평균을 빼 이동 성분을 없애면, 정사영(orthographic) 카메라 모델에서 $\mathbf{W}$의 rank는 3 이하가 된다. 논문은 이 사실을 rank theorem이라 불렀다. 이때 $\mathbf{W}$는 다음처럼 나뉜다:

$$\mathbf{W} = \mathbf{M} \mathbf{S}$$

$\mathbf{M}$은 $2F \times 3$ 모션 행렬, $\mathbf{S}$는 $3 \times P$ 구조 행렬이다. SVD로 $\mathbf{W}$의 특이값 가운데 큰 3개만 남기면 rank-3 분해를 얻는다. 그런데 $\mathbf{M}\mathbf{A}$와 $\mathbf{A}^{-1}\mathbf{S}$도 같은 $\mathbf{W}$를 만든다. 그래서 카메라 축을 나타내는 행들이 서로 직교하고 길이가 같아야 한다는 조건으로 가역 행렬 $\mathbf{A}$를 정한다. 논문은 이 조건을 metric constraints라 불렀고, 뒤의 문헌은 이 단계를 흔히 metric upgrade라 부른다.

SVD 한 번과 이 보정만으로 모든 프레임의 움직임과 모든 점의 3D 위치를 함께 구한다. 계산 비용은 대부분 $2F \times P$ 행렬의 SVD에서 나오고, 반복 비선형 최적화인 bundle adjustment보다 구조가 단순해 구현하기 쉬웠다.

factorization의 한계는 정사영·affine 가정이었다. Affine 카메라 모델은 원근 왜곡(perspective distortion)을 무시한다. 그래서 장면 안의 깊이 차이가 카메라까지 거리보다 충분히 작을 때, 예를 들어 멀리 있는 작은 물체를 찍을 때만 맞는다. 카메라에 가까운 장면, 화각이 넓은 렌즈, 앞뒤 깊이 차이가 큰 환경에서는 오차가 컸다. 1990년대 후반부터 여러 연구자가 원근 카메라 모델로 넓히려 했고, 이 흐름에서 컴퓨터 비전 연구자들은 사진측량의 bundle adjustment를 다시 찾아 쓰게 됐다.

---

## 3.3 Hartley & Zisserman의 교과서

2000년 Richard Hartley와 Andrew Zisserman의 교과서 *[Multiple View Geometry in Computer Vision](https://www.robots.ox.ac.uk/~vgg/hzbook/)*이 나왔다. 두 사람은 1981년부터 1990년대까지 여러 논문에 흩어져 있던 SfM 수학을 사영기하(projective geometry)로 정리했다. essential matrix, fundamental matrix, homography, camera calibration, bundle adjustment를 같은 표기와 같은 틀로 설명하자, 따로 다뤄지던 개념들이 같은 기하에서 나온다는 것이 분명해졌다.

이 책은 bundle adjustment를 비중 있게 다뤘다. Ch.1의 reprojection error 최소화 문제를 사영기하의 틀에서 다시 쓰고, 여기에 *robust cost function* $\rho$를 더했다. 이상치(outlier)가 섞인 실제 데이터에서도 최적화가 엉뚱한 해로 가지 않도록 Huber나 Cauchy 함수로 큰 오차의 영향을 줄인다. 최적화는 Levenberg-Marquardt로 풀고, Jacobian의 희소 구조로 계산량을 줄였다.

이 교과서는 그 뒤 SLAM·VO(visual odometry, 영상으로 카메라 이동을 추정하는 방법) 논문의 참고 문헌에 자주 올랐다.

---

## 3.4 Photo Tourism과 Bundler — 인터넷 규모 SfM

2006년 Noah Snavely, Steven Seitz, Richard Szeliski는 SIGGRAPH 논문 "[Photo Tourism](https://doi.org/10.1145/1179352.1141964)"을 발표했다. 이들은 Flickr에 올라온 관광지 사진(파리 노트르담 대성당, 로마 트레비 분수 등)을 모아 3D로 재구성했다.

수많은 사람이 제각기 찍어 올린 사진이라 카메라, 날씨, 구도가 저마다 달랐고 관계없는 사진도 섞여 있었다.

Snavely의 파이프라인은 먼저 SIFT로 특징을 찾아 이미지 쌍 사이의 대응점을 잇는다. 그다음 RANSAC으로 fundamental matrix를 추정해 기하적으로 맞지 않는 매칭을 버린다. 대응점이 많은 이미지 쌍에서 시작해 카메라를 하나씩 더해 가며(incremental SfM), 카메라를 더할 때마다 bundle adjustment로 모든 포즈와 점을 다시 최적화한다.

논문은 Flickr에서 내려받은 데이터셋으로 Notre Dame 대성당(2,635장 중 597장 등록), Trevi 분수(466장 중 360장), Yosemite Half Dome(1,882장 중 325장), Trafalgar Square(1,893장 중 278장)를 보고했다. 한 사람이 한 카메라로 찍은 Great Wall(120장 중 82장)과 Prague 구시가 광장(197장)도 실었다. 등록된 이미지의 평균 크기는 1,611×1,128 픽셀이었고, 평균 reprojection error는 약 1.5 픽셀이었다.

Snavely는 이 파이프라인을 Bundler라는 이름으로 공개했고, Bundler는 그 뒤 SfM 연구의 기본 도구가 됐다.

> 📜 **예언 vs 실제.** Snavely et al.(2006)은 논의 절에서 "Ultimately, we wish to scale up our reconstruction algorithm to handle millions of photographs"라고 썼다. 남은 과제로는 카메라가 늘수록 느려지는 계산(등록 순서를 바꿔 속도를 높이는 방안), 렌즈 왜곡 모델, 반복 구조가 있거나 특징이 약한 장면, 서로 연결되지 않은 여러 구조의 재구성을 꼽았다. 10년 뒤 COLMAP 논문(Schönberger & Frahm, CVPR 2016)은 로마 사진 75,000장 가운데 21,000장을 등록한 결과를 실었다. 수백만 장 규모는 같은 시기 다른 대규모 재구성 시스템이 다뤘다(Ch.1 §4).

<!-- DEMO: sfm_incremental.html -->

---

## 3.5 COLMAP — 공학적으로 다듬은 SfM

2016년 Johannes Schönberger와 Jan-Michael Frahm은 CVPR 논문 "[Structure-from-Motion Revisited](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)"를 발표했다. 두 사람은 공개 구현인 Bundler, 비공개 구현인 VisualSFM과 견주면서 incremental SfM의 단계를 하나씩 다시 설계했고, 결과를 COLMAP이라는 이름으로 공개했다.

카메라를 더하는 순서부터 바꿨다. 논문은 잘못 고른 이미지 하나가 카메라 등록 실패와 잘못된 삼각측량을 줄줄이 부를 수 있다고 봤다. COLMAP은 다음에 더할 이미지를 고를 때, 이미 3D 위치를 구한 점이 그 이미지에 몇 개 보이는지와 그 점들이 이미지 안에 얼마나 고르게 퍼져 있는지를 함께 점수로 매긴다. 보이는 점의 개수만 세는 방식, 보일 수 있는 점 가운데 실제로 보이는 점의 비율만 보는 방식과 비교한 실험에서, 결국 등록되는 이미지는 같았지만 이 점수로 순서를 정했을 때 카메라 위치 오차가 가장 작았다.

bundle adjustment를 돌리는 주기도 바꿨다. 카메라를 하나 더할 때마다 전체 bundle adjustment를 돌리면 비용이 크다. COLMAP은 이미지를 하나 등록할 때마다 그 이미지와 가장 많이 연결된 이미지들만 묶어 local bundle adjustment를 돌리고, 전체 bundle adjustment는 모델이 일정 비율만큼 커졌을 때만 돌렸다. 이 방식은 VisualSFM을 따른 것이다.

매칭을 기하적으로 검증하는 단계도 강화했다. COLMAP은 이미지 쌍마다 먼저 fundamental matrix를 추정해, inlier가 충분하면 그 쌍을 검증된 쌍으로 본다. 이어서 같은 쌍에 homography를 맞춰 inlier 수를 비교한다. Homography는 평면 장면이나 카메라가 제자리에서 회전만 한 경우를 나타내므로, homography inlier가 상대적으로 적으면 일반 장면으로 판단한다. 내부 파라미터를 아는 이미지는 essential matrix로 점을 삼각측량해 그 각도로 제자리 회전(panoramic)과 평면 장면을 가른다. 워터마크나 날짜 표시 때문에 서로 다른 장소의 사진이 이어진 쌍도 이 단계에서 걸러 낸다. 재구성은 제자리 회전이 아닌 쌍에서만 시작하고, 제자리 회전 쌍에서는 점을 삼각측량하지 않는다.

> 🔗 **차용.** COLMAP은 Bundler와 같은 incremental SfM이다. essential matrix 추정, triangulation, Levenberg-Marquardt 같은 핵심 수학은 Hartley & Zisserman 교과서에 있는 그대로다. COLMAP이 새로 정한 것은 이미지 순서, 검증, 최적화 주기 같은 단계별 판단이었다.

COLMAP이 널리 쓰인 데에는 성능 말고도 이유가 있었다. 코드가 잘 정리돼 있었고 문서가 충분했으며, CUDA 가속으로 큰 이미지 집합도 처리했다. 2020년 NeRF가 나온 뒤 많은 NeRF 학습 코드가 COLMAP이 낸 카메라 포즈와 sparse point cloud를 입력으로 받았고, 3D Gaussian Splatting의 대표 구현도 같은 전처리를 썼다. 그래서 COLMAP은 여러 3D 재구성 파이프라인이 공통으로 거치는 첫 단계가 됐다.

---

## 3.6 SfM과 SLAM이 나뉘다

SfM과 SLAM은 같은 수학을 쓰지만 처리 조건이 다른 문제를 푼다. 이 구분은 2000년대 초에 뚜렷해졌다.

SfM은 보통 이미지를 모두 모은 뒤 *오프라인*으로 처리한다. 시간 제약이 없으니 전체 데이터를 여러 번 다시 보면서 global bundle adjustment를 몇 번이고 돌리고, 카메라 포즈가 틀렸으면 돌아가 다시 계산한다.

움직이는 로봇에서 도는 SLAM에는 이런 여유가 없다. 센서 데이터가 실시간으로 들어오는 동안 로봇의 현재 위치를 그 자리에서 내놓아야 한다. 지난 데이터를 끝없이 다시 볼 수 없고, 지도가 커질수록 계산량도 커진다. 한 바퀴 돌아 처음 왔던 곳에 다시 오면, 그 사이 쌓인 위치 오차(drift)를 바로잡아야 한다. 이 보정은 루프 위의 여러 포즈를 한꺼번에 고친다.

불확실성을 다루는 방식도 달랐다. SLAM은 현재 포즈의 불확실성을 실시간으로 추적하면서 새 관측이 들어올 때마다 갱신하므로, EKF나 factor graph 같은 확률 표현이 필요하다. SfM은 최적화가 끝난 뒤에 covariance를 계산해도 된다.

Davison은 2003년 ICCV 논문 "[Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)"에서 이 차이를 직접 짚었다. 그는 성공한 SfM 방법들이 모두 영상 전체를 모아 한꺼번에 처리하는 오프라인 알고리즘이라고 지적했다. 그리고 카메라로 실시간 추정을 하려면 불확실성 전파를 중시하는 로봇공학의 SLAM 연구가 오프라인 SfM보다 더 가깝다고 주장했다. 결론에서는 자기 방법을 "sequential Simultaneous Localisation and Mapping or Structure from Motion"이라고 불렀다. 2000년대를 지나며 두 분야는 각자 문제를 따로 정의하는 별개의 연구 분야가 됐다.

> 🔗 **차용.** Nistér, Naroditsky, Bergen의 2004년 CVPR 논문 "Visual Odometry"는 카메라 자신의 움직임(ego-motion)을 실시간으로 추정하는 문제에 SfM 기법을 쓴 초기 연구로 널리 인용된다. 이 논문의 방법은 프레임 사이의 상대 포즈를 짧은 구간 안에서 차례로 푼다. 모든 데이터를 한꺼번에 최적화할 때보다 정확도는 조금 떨어지지만 결과가 빨리 나온다.

---

## 3.7 🧭 아직 열린 것

**움직이는 물체가 있는 장면의 SfM.** COLMAP을 비롯한 범용 SfM 파이프라인은 세계가 정지해 있다고 가정한다. 장면의 점이 움직이지 않는다고 보고 bundle adjustment를 풀기 때문에, 자동차나 보행자가 많은 장면에서는 움직이는 물체에서 나온 매칭이 최적화 결과를 틀어 놓는다. RANSAC이 일부를 걸러 내고, Dynamic SfM 연구는 segmentation이나 물체별 움직임을 모델에 넣는다.

**흐려지는 SfM과 SLAM의 경계.** 2023년 12월 Wang 등의 [DUSt3R](https://arxiv.org/abs/2312.14132)는 카메라 내부 파라미터도 포즈도 모르는 이미지 두 장을 Transformer 네트워크에 넣어, 픽셀마다 3D 좌표를 담은 pointmap을 바로 회귀했다. 카메라 포즈와 픽셀 대응은 이 pointmap에서 다시 구한다. 이미지가 셋 이상이면 쌍마다 얻은 pointmap을 한 좌표계로 맞추는 global alignment를 쓴다. 2024년 Leroy 등의 [MASt3R](https://arxiv.org/abs/2406.09756)는 이 네트워크에 dense local feature를 내는 출력을 더해 매칭 정확도를 높였다. 전통 SfM 파이프라인의 모듈이 하나씩 학습한 모델로 바뀌고 있고, NeRF·3DGS의 전처리를 COLMAP 대신 DUSt3R 계열 모델로 하는 연구도 나온다. 이 방식이 COLMAP을 실제로 대신하게 될지, 일부 분야에서만 쓰일지는 아직 모른다.

---

SfM 연구자들이 오프라인 재구성을 정밀하게 다듬는 동안, 움직이는 로봇은 이미지가 다 모이기 전에 자기 위치를 추정해야 했다. 이 요구에서 SLAM이라는 별개의 분야가 자랐고, 그 출발점은 Randall Smith와 Peter Cheeseman이 [1986년에 다룬 문제](https://people.csail.mit.edu/brooks/idocs/Smith_Cheeseman.pdf), 곧 불확실한 공간 관계를 어떻게 표현하고 전파할지였다.
