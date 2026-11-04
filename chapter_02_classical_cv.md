# Ch.2 — Classical CV 도구상자: Harris에서 SIFT까지, 그리고 ORB까지

Ch.1의 bundle adjustment는 카메라 자세와 3D 점을 함께 최적화한다. 이 최적화를 돌리려면 먼저 여러 이미지에서 서로 대응하는 점을 찾아야 한다. 측량에서는 사람이 현장에 표적을 세워 대응점을 만들었다. 컴퓨터 비전은 이 일을 알고리즘에 맡겨야 했다. 그래서 이미지에서 특징점을 찾는 문제(feature detection)와 찾은 점 주변을 비교할 수 있는 값으로 요약하는 문제(description)가 생겼다.

1970년대 후반 Hans Moravec은 Stanford Cart 프로젝트에서 카메라로 환경의 두드러진 점을 찾으려 했다. 그 작업은 1980년 Stanford 박사논문 ["Obstacle Avoidance and Navigation in the Real World by a Seeing Robot Rover"](https://frc.ri.cmu.edu/~hpm/project.archive/robot.papers/1975.cart/1980.html.thesis/index.html)로 정리됐다. Moravec은 패치를 조금씩 옮겨 볼 때 밝기가 얼마나 변하는지를 재서 추적할 점을 고르는 기준을 세웠다. 오늘날 특징 기반 SLAM의 front-end(이미지에서 관측을 뽑는 앞단)는 대부분 이 장에서 다루는 검출기와 서술자나 그 변형을 쓴다.

---

## 2.1 코너라는 개념: Moravec에서 Harris까지

창을 어느 방향으로 조금 옮기든 그 안의 영상이 크게 변하는 점을 "코너"라 부른다. Moravec(1977)의 기준은 단순했다. 창을 가로, 세로, 두 대각선, 이렇게 45도 간격의 네 방향으로 한 픽셀씩 옮겨 보고 원래 창과의 차이 제곱합(Sum of Squared Differences, SSD)을 잰다. 네 값 가운데 가장 작은 값이 충분히 크면 코너로 본다.

Harris와 Stephens는 1988년 Alvey Vision Conference에서 ["A Combined Corner and Edge Detector"](https://www.bmva-archive.org.uk/bmvc/1988/avc-88-023.html)를 발표했다. 이들은 Moravec의 기준이 45도 간격의 방향만 봐서 방향에 따라 응답이 달라진다고 지적하고, 이미지 미분으로 모든 방향의 작은 이동을 한꺼번에 다뤘다. 이미지 $I$에서 점 $(x,y)$ 주변 창 $W$를 이동량 $(\Delta x, \Delta y)$로 움직일 때 강도 변화를 근사하면:

$$M = \sum_{(x,y) \in W} \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}$$

$M$의 두 eigenvalue $\lambda_1, \lambda_2$로 점의 성격을 구분한다. 둘 다 크면 코너, 하나만 크면 엣지, 둘 다 작으면 평탄한 영역으로 본다. Harris는 $R = \det(M) - k \cdot \text{tr}(M)^2$라는 점수로 코너를 가려서 eigenvalue를 직접 구하지 않아도 되게 했다. 실제 구현에서는 $k$로 보통 0.04~0.06을 쓴다.

> 🔗 **차용.** Harris(1988)는 Stanford Cart 연구를 정리한 Moravec의 1980년 보고서(CMU-RI-TR-3)를 출발점으로 인용했다.

1994년 Jianbo Shi와 Carlo Tomasi는 ["Good Features to Track"](https://cecas.clemson.edu/~stb/klt/shi-tomasi-good-features-cvpr1994.pdf)(CVPR 1994)에서 좋은 특징을 추적기가 잘 따라갈 수 있는 창으로 정의했다. 두 eigenvalue 가운데 작은 값 $\min(\lambda_1, \lambda_2)$가 정해 둔 문턱보다 크면 그 창을 고른다. 같은 기준은 1991년 Tomasi와 Kanade의 보고서에도 이미 나온다. 이 기준을 Shi-Tomasi 코너 검출이라 부른다. OpenCV는 이 방법을 `goodFeaturesToTrack` 함수로 구현했고, 지금도 같은 이름으로 쓴다.

---

## 2.2 추적의 원형: Lucas-Kanade와 KLT

찾은 점이 다음 프레임에서 어디로 갔는지는 따로 구해야 한다. Bruce Lucas와 Takeo Kanade는 1981년 ["An Iterative Image Registration Technique"](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf)에서 프레임 사이의 픽셀 이동을 최소화 문제로 정식화했다. 이때 밝기 불변 가정(brightness constancy assumption)을 둔다.

밝기 불변 가정은 한 점의 밝기가 움직이기 전과 뒤에 같다는 가정이다.

$$I(x, y, t) = I(x + u, y + v, t + 1)$$

테일러 전개 후 선형화하면:

$$I_x u + I_y v + I_t = 0$$

방정식은 하나인데 미지수는 둘이다. Lucas-Kanade는 작은 창 안의 픽셀들이 모두 같은 $(u,v)$로 움직인다고 가정해서 방정식을 미지수보다 많이 만들고, 이 연립방정식을 최소자승으로 푼다.

$$\begin{pmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{pmatrix} \begin{pmatrix} u \\ v \end{pmatrix} = -\begin{pmatrix} \sum I_x I_t \\ \sum I_y I_t \end{pmatrix}$$

왼쪽 행렬은 Harris의 행렬 $M$과 같다. 코너 검출과 optical flow는 같은 행렬을 계산한다.

Tomasi와 Kanade는 1991년 tech report ["Detection and Tracking of Point Features"](https://cecas.clemson.edu/~stb/klt/tomasi-kanade-techreport-1991.pdf)에서 eigenvalue로 추적하기 좋은 창을 고르고 Newton-Raphson 반복으로 이동량을 다듬는 구현을 내놓았다. 2000년 Intel의 Jean-Yves Bouguet은 이미지 피라미드로 구현을 확장했다. 해상도를 단계별로 낮춘 이미지를 만들고 거친 해상도부터 차례로 맞춰 가므로(coarse-to-fine) 이동이 클 때도 수렴한다. 이 조합을 KLT(Kanade-Lucas-Tomasi) 추적기라 부른다. [VINS-Mono](https://arxiv.org/abs/1708.03852)(TRO 2018) 같은 실시간 VIO(visual-inertial odometry, 카메라와 IMU를 함께 쓰는 위치 추정)도 front-end에서 KLT로 특징을 추적한다.

---

## 2.3 SIFT — 불변성과 특허

KLT는 한 카메라가 조금씩 움직이며 찍은 연속 프레임에서 잘 동작한다. 다른 카메라로 다른 날 찍은 이미지에서는 같은 점이라도 주변 패치의 모양, 크기, 방향이 달라서 픽셀 값을 바로 비교하기 어렵다. 이럴 때는 점 주변을 시점이 바뀌어도 크게 변하지 않는 벡터로 요약해 비교한다. 이 벡터를 **서술자(descriptor)**라 한다.

UBC의 David Lowe는 1999년 ICCV에 "Object Recognition from Local Scale-Invariant Features"를 발표하고, 점마다 만든 160차원 벡터를 비교해 서로 다른 사진에서 같은 물체를 찾아 보였다. 5년 뒤인 2004년에는 IJCV에 서술자를 128차원으로 다듬은 완성판 ["Distinctive Image Features from Scale-Invariant Keypoints"](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf)를 실었다. 오늘날 SIFT(Scale-Invariant Feature Transform)를 인용할 때는 주로 이 논문을 쓴다. SIFT는 검출과 서술 두 단계로 이루어진다.

**검출 단계**: 여러 scale에서 DoG(Difference of Gaussians)를 계산하고, 그 극값을 keypoint로 고른다. DoG는 Laplacian of Gaussian의 근사다. $L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)$를 Gaussian 스무딩 이미지라 하면:

$$D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)$$

$k$는 이웃한 scale 사이의 비율로, Lowe는 이를 $2^{1/s}$로 정했다($s$는 octave 하나에 들어가는 scale 수). 여러 octave에서 극값을 찾으므로 영상 크기가 달라도 같은 점을 찾는다.

**서술자 단계**: keypoint 주변 $16\times16$ 표본 창을 $4\times4$ 블록으로 나누고, 블록마다 gradient 방향 히스토그램(8개 구간)을 만들어 이어 붙이면 128차원 벡터가 된다. 창을 keypoint의 주된 gradient 방향에 맞춰 돌린 다음 계산하므로 영상이 회전해도 서술자가 거의 같다.

Lowe(2004)는 SIFT keypoint가 회전과 크기 변화에 불변이고, affine 변형·노이즈·조명 변화에도 상당한 범위에서 견딘다고 보고했다. 그래서 SIFT는 KITTI 같은 SLAM 벤치마크가 나오기 전부터 시점과 크기가 다른 영상 사이의 대응점을 찾는 데 널리 쓰였다.

UBC는 Lowe를 발명자로 2000년 3월 SIFT 특허를 출원했고, 특허는 2004년 3월 등록됐다(US6711293B1, 우선일 1999년 3월 8일). 특허가 끝난 2020년 3월 전까지는 SIFT를 상업 제품에 쓰려면 라이선스가 필요했고, 그동안 SIFT를 대신할 방법을 찾는 연구가 이어졌다.

> 📜 **예언 vs 실제.** Lowe는 2004년 SIFT 논문의 "9 Conclusions"에서 물체 인식 말고도 keypoint를 쓸 수 있는 곳으로 "view matching for 3D reconstruction, motion tracking and segmentation, robot localization, image panorama assembly, epipolar calibration"을 꼽았다. 예언은 대부분 맞았다. 2000년대 후반의 SfM, SLAM, 파노라마 합성, 초기 영상 기반 로봇 위치 추정에는 SIFT가 쓰였다. 2012년 AlexNet 이후 물체 인식은 CNN으로 옮겨 갔고, SLAM에 쓰는 local descriptor도 SuperPoint·R2D2 같은 학습 서술자로 점차 바뀌었다.

---

## 2.4 SURF — 속도를 위해 근사하다

SIFT는 계산이 무거워 실시간 SLAM에 쓰기 어려웠다. SURF 논문의 측정에서는 800×640 이미지 한 장의 검출과 서술에 SIFT가 1,036 ms 걸렸다. ETH Zurich의 Herbert Bay는 KU Leuven의 Tinne Tuytelaars, ETH Zurich와 KU Leuven에 함께 적을 둔 Luc Van Gool과 함께 2006년 ECCV에 ["SURF: Speeded-Up Robust Features"](https://people.ee.ethz.ch/~surf/eccv06.pdf)를 발표하고 검출과 서술의 계산량을 함께 줄였다.

SURF는 DoG 대신 *Hessian 행렬의 행렬식*으로 keypoint를 찾는다. Gaussian 이차 미분은 integral image로 계산하는 box filter로 근사해서 빠르게 구한다. 서술자는 SIFT의 절반인 64차원이다. keypoint 주변을 $4\times4$ 하위 영역으로 나누고, 영역마다 Haar wavelet 응답 $d_x, d_y$로 $(\sum d_x,\, \sum d_y,\, \sum|d_x|,\, \sum|d_y|)$ 네 값을 구해 이어 붙이면 $4\times4\times4=64$차원이 된다. 128차원으로 늘린 SURF-128도 있지만 기본은 64차원이다.

같은 측정에서 SURF는 354 ms, 회전 불변성을 뺀 U-SURF는 255 ms가 걸려 SIFT보다 3~4배 빨랐다. 검출기만 보면 SURF의 Fast-Hessian이 DoG보다 3배 넘게 빨랐다. 정확도는 차원 수, 검출기와 서술 방식, 평가 조건에 따라 앞서기도 하고 뒤지기도 했다. SURF에도 특허가 걸려 있었다.

> 🔗 **차용.** Lowe(1999/2004)의 DoG scale-space → Bay(2006)의 Hessian integral image: 두 방법 모두 scale이 달라도 같은 점을 찾으려 했다. Bay는 integral image로 box filter를 계산해서 Hessian을 DoG보다 빠르게 근사했다.

---

## 2.5 ORB — 특허 없는 binary descriptor

2011년 Ethan Rublee, Vincent Rabaud, Kurt Konolige, Gary Bradski는 ICCV에 ["ORB: An Efficient Alternative to SIFT or SURF"](https://www.gwylab.com/download/ORB_2012.pdf)를 발표했다. 네 저자는 모두 ROS를 개발하던 Willow Garage 소속이었다.

ORB는 이미 있던 검출기와 서술자를 고쳐서 묶었다.

**검출**: [FAST](https://www.edwardrosten.com/work/rosten_2006_machine.pdf)(Features from Accelerated Segment Test, Rosten & Drummond 2006). FAST는 한 픽셀 둘레의 원 위에 있는 16개 점을 살펴서, 가운데보다 충분히 밝거나 어두운 점이 연속으로 길게 이어지면 코너로 본다. 비교 연산만 하므로 SIFT의 DoG보다 훨씬 빠르다. ORB는 FAST가 찾은 점에 Harris 점수를 매겨 응답이 강한 점만 남긴다.

**서술자**: [BRIEF](https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf)(Binary Robust Independent Elementary Features, Calonder et al. 2010). BRIEF는 keypoint 주변 패치에서 무작위로 고른 점 쌍의 밝기를 비교해 비트열을 만든다. 기본 길이는 256비트다. 두 서술자는 Hamming 거리로 비교한다. 두 비트열을 XOR한 뒤 1인 비트 수를 세면(popcount) 거리가 나오므로 계산이 아주 빠르다.

BRIEF는 영상이 몇 도만 회전해도 매칭 성능이 크게 떨어졌다. Rublee와 동료들은 패치의 밝기 중심(intensity centroid)으로 방향을 구하고, 비교할 점 쌍의 위치를 그 방향만큼 돌려서 BRIEF를 계산했다(steered BRIEF). 그런데 이렇게 돌리면 비트들이 서로 비슷해져 변별력이 떨어졌다. 그래서 학습 데이터에서 분산이 크고 서로 상관이 낮은 점 쌍 256개를 골라 썼고, 이것을 **rBRIEF(rotation-aware BRIEF)**라 불렀다.

$$\theta = \text{atan2}(m_{01},\, m_{10}), \quad m_{pq} = \sum_{x,y} x^p y^q I(x,y)$$

논문은 ORB가 SIFT보다 약 100배 빠르면서 여러 상황에서 비슷한 성능을 낸다고 보고했다. ORB는 특허 부담이 없었고 OpenCV에 들어갔다. [ORB-SLAM](https://arxiv.org/abs/1502.00956)(Mur-Artal et al., TRO 2015)은 이름대로 ORB를 썼고, 2021년의 ORB-SLAM3까지 front-end의 ORB를 바꾸지 않았다.

> 🔗 **차용.** Calonder et al.(2010)의 BRIEF → Rublee et al.(2011)의 ORB: binary descriptor에 밝기 중심으로 구한 방향을 더하고 비교할 점 쌍을 학습으로 골라 회전에 강하게 만들었다.

---

## 2.6 학습 기반 descriptor

사람이 설계한 규칙 대신 데이터로 학습한 규칙을 쓰면 더 나을까? 2016년 Yi et al.이 발표한 [LIFT](https://arxiv.org/abs/1603.09114)(Learned Invariant Feature Transform, ECCV 2016)는 검출, 방향 추정, 서술자 계산을 모두 CNN으로 바꾸려 했다. 단계마다 따로 학습한 네트워크 셋을 차례로 이었다.

2018년 DeTone et al.의 [SuperPoint](https://arxiv.org/abs/1712.07629)(CVPRW 2018)는 keypoint 검출과 256차원 서술자를 한 네트워크로 함께 학습했다. 먼저 합성 도형 데이터로 학습하고, 실제 이미지에 homography 변환을 여러 번 걸어 얻은 검출 결과를 정답처럼 써서 다시 학습했다(homographic adaptation). SuperPoint는 그 뒤 SLAM 연구에서 널리 시험된 학습 특징 가운데 하나가 됐다. [AnyLoc](https://arxiv.org/abs/2308.00688)(Keetha et al., RA-L 2024)처럼 장소 인식에 DINOv2 같은 foundation model 특징을 쓰는 연구도 나왔다.

2026년에도 사람이 설계한 descriptor는 쓰인다. 임베디드 장치에서는 ORB가 SuperPoint보다 빠르다. 학습 서술자는 학습하지 않은 종류의 이미지에서 성능이 들쭉날쭉한데, ORB는 어떤 이미지에서 어떻게 동작할지 예측하기 쉽다.

---

## 2.7 🧭 아직 열린 것

**학습 기반 descriptor의 일반화 한계.** SuperPoint, R2D2, DISK 같은 학습 서술자는 학습한 것과 비슷한 이미지에서는 전통 방법보다 나은 경우가 많지만, 수중·열화상·저조도처럼 처음 보는 환경에서는 결과가 고르지 않다. 어느 쪽이 나은지는 2026년에도 환경마다 판단이 갈린다.

**Wide-baseline 매칭의 실패 모드.** Harris나 ORB로 매칭하면 시점이 크게 바뀔 때 성능이 떨어진다. 얼마나 떨어지는지는 장면, 회전축, 매칭 조건에 따라 다르다. ASIFT, MSER 같은 affine-covariant detector가 일부를 보완했지만 문제는 남아 있다. [DUSt3R](https://arxiv.org/abs/2312.14132)(Wang et al., CVPR 2024)는 두 이미지에서 대응점 매칭 단계 없이 pointmap을 직접 회귀한다. 이 방식이 descriptor를 어디까지 대신할지는 아직 알 수 없다.

---

이 장의 기법들은 이미지 한 장에서 점을 찾거나 두 장 사이에서 점을 잇는다. 이미지 수십, 수백 장을 기하적으로 어긋나지 않게 한꺼번에 연결하는 방법은 Ch.3의 SfM에서 다룬다.

---

*참고 문헌*

- Harris, C. & Stephens, M. (1988). A Combined Corner and Edge Detector. *Proc. Alvey Vision Conference*.
- Lucas, B. D. & Kanade, T. (1981). An Iterative Image Registration Technique with an Application to Stereo Vision. *IJCAI*.
- Shi, J. & Tomasi, C. (1994). Good Features to Track. *CVPR*.
- [Lowe, D. G. (2004). Distinctive Image Features from Scale-Invariant Keypoints.](https://doi.org/10.1023/B:VISI.0000029664.99615.94) *IJCV 60(2)*.
- Bay, H., Tuytelaars, T. & Van Gool, L. (2006). SURF: Speeded-Up Robust Features. *ECCV*.
- Calonder, M. et al. (2010). BRIEF: Binary Robust Independent Elementary Features. *ECCV*.
- [Rublee, E. et al. (2011). ORB: An Efficient Alternative to SIFT or SURF.](https://doi.org/10.1109/ICCV.2011.6126544) *ICCV*.
- DeTone, D., Malisiewicz, T. & Rabinovich, A. (2018). SuperPoint: Self-Supervised Interest Point Detection and Description. *CVPRW*. [arXiv:1712.07629](https://arxiv.org/abs/1712.07629)
