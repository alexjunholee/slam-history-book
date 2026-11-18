# 2장 고전 특징점: Harris에서 ORB까지

Bundle adjustment가 카메라 자세와 3D 점 좌표를 한꺼번에 깎아내리려면(→ 1장), 먼저 여러 이미지에서 같은 물리적 지점을 가리키는 대응점 쌍이 손에 들어와야 한다. 측량학에서는 사람이 직접 현장에 표적을 세워 대응점을 확보했지만, 움직이는 로봇을 다루는 컴퓨터 비전은 그 역할을 온전히 알고리즘에 맡겨야 했다. 결국 두 가지 질문이 피할 수 없는 과제로 떠올랐다. 이미지 속 어디에서 추적하기 좋은 점을 뽑을 것인가(feature detection), 그리고 그 점 주변의 생김새를 시점이 달라져도 비교할 수 있는 숫자 묶음으로 어떻게 요약할 것인가(description)라는 질문이다.

그 실마리는 1970년대 후반 Stanford Cart 프로젝트에서 나왔다. 자율 주행 로버를 만들던 Hans Moravec은 1980년 Stanford 박사논문 ["Obstacle Avoidance and Navigation in the Real World by a Seeing Robot Rover"](https://frc.ri.cmu.edu/~hpm/project.archive/robot.papers/1975.cart/1980.html.thesis/index.html)에서 작은 이미지 패치를 조금씩 밀어 볼 때 밝기가 얼마나 크게 요동치는지로 추적할 만한 점을 가려내는 기준을 세웠다. 반세기가 지난 오늘날에도 특징 기반 SLAM의 front-end는 대부분 Moravec이 던진 이 직관의 직계 후손들을 쓰고 있다.

---

## 2.1 코너라는 개념: Moravec에서 Harris까지

작은 창(window)을 어느 방향으로 조금만 밀어도 그 안의 밝기 패턴이 크게 달라지는 지점을 컴퓨터 비전에서는 "코너(corner)"라고 부른다. 평탄한 벽면은 어느 쪽으로 밀어도 비슷하고, 직선 경계(edge)는 선을 따라 밀면 변화가 없지만, 모서리나 뾰족한 질감은 사방 어느 쪽으로 움직여도 즉각 차이가 드러나기 때문이다. Moravec(1977)의 판별법은 소박했다. 창을 가로, 세로, 두 대각선—즉 45도 간격의 네 방향—으로 한 픽셀씩 옮겨 보며 원래 창과의 차이 제곱합(Sum of Squared Differences, SSD)을 구한 뒤, 네 값 중 최솟값이 충분히 크면 코너로 판정했다.

그러나 45도 간격만 살피는 방식은 카메라가 조금만 기울어져도 응답이 출렁이는 한계가 있었다. 1988년 Alvey Vision Conference에서 Chris Harris와 Mike Stephens는 ["A Combined Corner and Edge Detector"](https://www.bmva-archive.org.uk/bmvc/1988/avc-88-023.html)를 내놓으며 이 불연속성을 이미지 미분으로 해결했다. 점 $(x,y)$ 주변의 창 $W$를 임의의 미소 변위 $(\Delta x, \Delta y)$로 움직일 때 밝기 변화량을 테일러 근사로 펼치면, 모든 방향의 정보가 하나의 $2\times 2$ 행렬로 모인다.

$$M = \sum_{(x,y) \in W} \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}$$

이제 행렬 $M$의 두 eigenvalue $\lambda_1, \lambda_2$를 들여다보면 점의 기하적 성격이 그대로 드러난다. 둘 다 크면 어느 방향으로 움직여도 밝기가 크게 변하는 코너이고, 하나만 크면 한쪽 방향으로만 변하는 엣지이며, 둘 다 작으면 밋밋한 평면이다. Harris는 당시 계산 부담이 컸던 eigenvalue 분해를 우회하려고 $R = \det(M) - k \cdot \text{tr}(M)^2$라는 간단한 판별식을 고안했다(실제 구현에서는 보통 $k$로 0.04~0.06을 쓴다).

1994년 CVPR에 발표된 Jianbo Shi와 Carlo Tomasi의 ["Good Features to Track"](https://cecas.clemson.edu/~stb/klt/shi-tomasi-good-features-cvpr1994.pdf)는 한 걸음 더 나아갔다. 이들은 "좋은 특징"이란 단순히 눈에 띄는 점이 아니라 *추적기가 끝까지 놓치지 않고 따라갈 수 있는 창*이라고 재정의하고, 두 eigenvalue 가운데 작은 값인 $\min(\lambda_1, \lambda_2)$가 문턱값을 넘으면 코너로 뽑는 기준을 제시했다(같은 기준은 1991년 Tomasi와 Kanade의 보고서에도 이미 등장한다). 오늘날 Shi-Tomasi 코너 검출이라 불리는 이 방법은 OpenCV의 `goodFeaturesToTrack` 함수 이름 속에 지금도 살아 숨 쉬고 있다.

> 🔗 **차용.** Harris(1988)는 Stanford Cart 연구를 집대성한 Moravec의 1980년 보고서(CMU-RI-TR-3)를 출발점으로 직접 인용하며, 4방향 이산 차분을 연속적인 미분 행렬로 일반화했다.

---

## 2.2 추적의 원형: Lucas-Kanade와 KLT

좋은 점을 찾았다면 다음 프레임에서 그 점이 어디로 이동했는지 따라가야 한다. 1981년 Bruce Lucas와 Takeo Kanade는 ["An Iterative Image Registration Technique"](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf)에서 연속 프레임 사이의 픽셀 이동을 최적화 문제로 정식화했다. 이들이 놓은 전제는 단 하나, 짧은 순간 동안 움직인 점은 이동 전후의 밝기가 변하지 않는다는 **밝기 불변 가정(brightness constancy assumption)**이었다.

$$I(x, y, t) = I(x + u, y + v, t + 1)$$

작은 이동 $(u, v)$에 대해 이 식을 1차 테일러 전개로 선형화하면 유명한 optical flow 구속 방정식을 얻는다.

$$I_x u + I_y v + I_t = 0$$

식 하나에 미지수는 $u, v$ 둘이라 한 픽셀만 봐서는 해가 정해지지 않는다. Lucas와 Kanade는 작은 창 안의 이웃 픽셀들이 모두 같은 속도 $(u, v)$로 함께 움직인다고 가정해 방정식 수를 늘렸고, 이를 최소자승 문제로 묶어 풀었다.

$$\begin{pmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{pmatrix} \begin{pmatrix} u \\ v \end{pmatrix} = -\begin{pmatrix} \sum I_x I_t \\ \sum I_y I_t \end{pmatrix}$$

여기서 눈여겨볼 대목은 왼쪽 계수 행렬이다. 이 행렬은 2.1절에서 본 Harris의 행렬 $M$과 토씨 하나 틀리지 않고 같다. 코너를 고르는 수학과 optical flow로 코너를 쫓는 수학이 사실상 같은 행렬의 안정성(가역성) 위에서 맞물려 돌아간다는 뜻이다.

1991년 Tomasi와 Kanade는 기술 보고서 ["Detection and Tracking of Point Features"](https://cecas.clemson.edu/~stb/klt/tomasi-kanade-techreport-1991.pdf)에서 eigenvalue 기준으로 추적하기 좋은 창을 선별하고 Newton-Raphson 반복으로 이동량을 정밀하게 깎아내는 구현을 정립했다. 이어 2000년 Intel의 Jean-Yves Bouguet이 해상도를 단계별로 낮춘 이미지 피라미드를 도입해 거친 해상도부터 굵직한 이동을 맞춘 뒤 고해상도로 좁혀 들어가는(coarse-to-fine) 구조를 얹으면서, 큰 움직임에서도 잘 수렴하는 오늘날의 **KLT(Kanade-Lucas-Tomasi) 추적기**가 완성됐다. 이 1981년산 아이디어는 [VINS-Mono](https://arxiv.org/abs/1708.03852)(TRO 2018) 같은 현대 실시간 VIO(visual-inertial odometry)의 front-end에서도 여전히 현역으로 뛴다.

---

## 2.3 SIFT — 불변성과 특허

KLT는 카메라가 조금씩 연속적으로 움직이는 영상에서는 발군이지만, 다른 카메라로 다른 날, 다른 거리와 각도에서 찍은 사진 두 장을 갖다 대면 맥을 못 춘다. 같은 지점이라도 주변 패치의 크기와 회전, 명암이 달라져 픽셀 밝기를 직접 견줄 수 없기 때문이다. 이 장벽을 넘으려면 시점이 바뀌어도 값이 크게 흔들리지 않는 수치적 지문, 즉 **서술자(descriptor)**가 필요했다.

1999년 ICCV에서 UBC의 David Lowe는 "Object Recognition from Local Scale-Invariant Features"를 발표해 점마다 뽑아낸 160차원 벡터만으로 서로 다른 사진 속 같은 물체를 찾아냈다. 5년 뒤인 2004년 *IJCV*에 실린 완성판 ["Distinctive Image Features from Scale-Invariant Keypoints"](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf)는 서술자를 128차원으로 다듬어 컴퓨터 비전의 교과서적 표준이 됐으며, 오늘날 **SIFT(Scale-Invariant Feature Transform)**를 말할 때 대부분 이 논문을 가리킨다. SIFT는 검출과 서술이라는 두 톱니바퀴로 맞물려 돌아간다.

**검출 단계**에서는 이미지를 여러 스케일로 흐려 가며 인접 스케일 간의 차이, 즉 DoG(Difference of Gaussians)를 계산해 그 극값을 keypoint로 삼는다. DoG는 계산량이 큰 Laplacian of Gaussian을 빠르게 근사한 것이다. Gaussian 스무딩 이미지를 $L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)$라 할 때, 두 스케일 사이의 차이는 다음과 같다.

$$D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)$$

이웃한 scale의 비율 $k$는 한 octave에 들어가는 scale 수 $s$에 따라 $2^{1/s}$로 정해진다. 여러 octave에 걸쳐 공간과 스케일 축 모두에서 극값을 찾기 때문에, 카메라가 멀어지든 가까워지든 같은 3D 지점을 일관되게 붙잡을 수 있다.

**서술자 단계**는 회전 불변성을 부여하는 과정이다. keypoint의 주된 gradient 방향을 먼저 계산해 그 방향으로 기준축을 돌린 뒤, 주변 $16\times 16$ 표본 창을 $4\times 4$ 블록으로 쪼갠다. 각 블록 안에서 gradient 방향을 8개 구간으로 묶어 히스토그램을 만들고 이를 한 줄로 이어 붙이면 128차원($4\times 4\times 8$) 벡터가 탄생한다. 방향을 정렬한 덕분에 이미지가 통째로 돌아가도 서술자 벡터는 거의 변하지 않는다.

Lowe(2004)가 보인 것처럼 SIFT는 회전과 크기 변화에 불변일 뿐 아니라 affine 변형, 노이즈, 조명 변화에도 상당한 탄력을 발휘했다. 덕분에 SIFT는 KITTI 같은 SLAM 벤치마크가 등장하기 한참 전부터 서로 다른 시점의 사진들을 엮는 SfM과 장소 인식의 사실상 표준으로 군림했다.

다만 SIFT에는 기술 외적인 족쇄가 하나 채워져 있었다. UBC가 Lowe를 발명자로 2000년 3월 특허를 출원해 2004년 3월 등록을 마친 것이다(US6711293B1, 우선일 1999년 3월 8일). 특허가 만료된 2020년 3월까지 상업 제품에 SIFT를 넣으려면 라이선스 비용을 물어야 했고, 이 상업적 장벽은 거꾸로 SIFT를 대체할 가볍고 자유로운 서술자를 찾아 나서는 연구 경쟁에 불을 붙였다.

> 📜 **예언 vs 실제.** Lowe는 2004년 논문의 "9 Conclusions"에서 SIFT가 물체 인식을 넘어 "view matching for 3D reconstruction, motion tracking and segmentation, robot localization, image panorama assembly, epipolar calibration"에 쓰일 것이라 내다봤다. 그 예언은 놀라울 만큼 적중해, 2000년대 후반의 SfM과 파노라마 합성, 초기 시각 로봇 위치 추정은 거의 SIFT의 독무대였다. 다만 2012년 AlexNet 이후 물체 인식의 주도권은 CNN으로 넘어갔고, SLAM의 국소 서술자 역시 점차 SuperPoint·R2D2 같은 학습 모델의 도전을 받게 된다.

---

## 2.4 SURF — 속도를 위해 근사하다

SIFT는 정확했지만 실시간 로봇에 올리기에는 너무 무거웠다. 후속 논문인 SURF의 벤치마크에 따르면 800×640 이미지 한 장에서 특징을 뽑고 서술자를 만드는 데만 SIFT는 무려 1,036 ms가 걸렸다. 1초에 한 장꼴이니 초당 30프레임으로 움직여야 하는 실시간 SLAM에는 언감생심이었다. 2006년 ECCV에서 ETH Zurich의 Herbert Bay, KU Leuven의 Tinne Tuytelaars, 그리고 두 대학에 모두 적을 둔 Luc Van Gool이 발표한 ["SURF: Speeded-Up Robust Features"](https://people.ee.ethz.ch/~surf/eccv06.pdf)는 검출과 서술 양쪽에서 계산량을 과감히 덜어낸 해답이었다.

SURF는 SIFT의 DoG 피라미드 대신 *Hessian 행렬의 행렬식*으로 특징점을 찾았다. 핵심 비결은 Gaussian 이차 미분을 box filter로 과감히 근사하고, 이를 **integral image(적분 영상)**로 처리해 필터 크기와 무관하게 상수 시간에 계산한 데 있다. 서술자 역시 SIFT의 절반인 64차원으로 줄였다. keypoint 주변을 $4\times 4$ 하위 영역으로 나눈 뒤, 영역마다 Haar wavelet 응답 $d_x, d_y$를 뽑아 $(\sum d_x,\, \sum d_y,\, \sum|d_x|,\, \sum|d_y|)$ 네 값만 추려 붙인 결과다(필요하면 128차원 SURF-128로 늘릴 수도 있었다).

속도 차이는 극적이었다. 같은 800×640 이미지 측정에서 SURF는 354 ms, 회전 불변성을 끈 U-SURF는 255 ms를 기록해 SIFT보다 3~4배 빨랐고, 검출기만 떼어 놓으면 SURF의 Fast-Hessian이 DoG보다 3배 넘게 빨랐다. 정확도는 평가 조건에 따라 SIFT와 엎치락뒤치락했다. 하지만 SURF 역시 특허로 묶여 있었다는 점에서는 SIFT와 처지가 같았다.

> 🔗 **차용.** Lowe(1999/2004)의 DoG scale-space → Bay(2006)의 Hessian integral image: 두 기법 모두 크기 변화에 흔들리지 않는 특징을 노렸지만, Bay는 적분 영상 위의 box filter 근사로 Hessian 계산을 갈아치워 속도를 끌어올렸다.

---

## 2.5 ORB — 특허 없는 binary 서술자

실시간 SLAM이 바라는 조건은 까다로웠다. SIFT보다 수십 배 빠르면서, 회전과 크기 변화에도 견뎌야 하고, 무엇보다 특허 눈치를 보지 않고 누구나 코드에 집어넣을 수 있어야 했다. 2011년 ICCV에서 ROS를 개발하던 Willow Garage의 Ethan Rublee, Vincent Rabaud, Kurt Konolige, Gary Bradski가 발표한 ["ORB: An Efficient Alternative to SIFT or SURF"](https://www.gwylab.com/download/ORB_2012.pdf)는 바로 그 빈자리를 정확히 메웠다.

ORB의 전략은 무에서 유를 창조하는 대신, 이미 검증된 빠른 검출기와 가벼운 binary 서술자를 영리하게 고쳐 한데 묶는 것이었다.

**검출**에는 Rosten과 Drummond(2006)의 [FAST](https://www.edwardrosten.com/work/rosten_2006_machine.pdf)(Features from Accelerated Segment Test)를 가져왔다. 중심 픽셀을 둘러싼 반지름 원 위의 16개 픽셀을 훑어 중심보다 일정 문턱 이상 밝거나 어두운 점이 연속으로 길게 이어지면 코너로 찍는 방식이다. 미분도 행렬식도 없이 단순 픽셀 밝기 비교만 수행하므로 SIFT의 DoG와는 비교할 수 없이 빠르다. ORB는 여기에 FAST가 엣지 위에서도 반응하는 약점을 보완하려고 Harris 코너 점수를 매겨 강한 점만 추려 냈다.

**서술자**는 Calonder et al.(2010)의 [BRIEF](https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf)(Binary Robust Independent Elementary Features)를 바탕에 두었다. keypoint 주변 패치에서 미리 정해 둔 점 쌍 256쌍의 밝기를 비교해, 한쪽이 밝으면 1, 어두우면 0을 적어 256비트 문자열을 만든다. 실수형 벡터의 유클리드 거리 대신 두 비트열을 XOR한 뒤 1인 비트 개수만 세는(popcount) Hamming 거리를 쓰니, 현대 CPU 명령어 한두 줄로 순식간에 매칭이 끝난다.

문제는 원래의 BRIEF가 이미지가 몇 도만 돌아가도 매칭률이 수직 낙하한다는 점이었다. Rublee와 동료들은 패치의 밝기 중심(intensity centroid)으로 주된 방향각 $\theta$를 구하고, 비교할 점 쌍의 좌표를 그 각도만큼 회전시켜 계산하는 **steered BRIEF**를 도입했다.

$$\theta = \text{atan2}(m_{01},\, m_{10}), \quad m_{pq} = \sum_{x,y} x^p y^q I(x,y)$$

그런데 점 쌍을 회전시키자 이번에는 비트들 사이의 상관관계가 높아져 변별력이 떨어지는 부작용이 나타났다. 연구진은 학습 데이터를 동원해 분산이 크고 서로 상관관계가 낮은 256개 점 쌍만 탐욕적으로 골라내 이 문제를 잡았고, 이렇게 탄생한 서술자에 **rBRIEF(rotation-aware BRIEF)**라는 이름을 붙였다.

논문에 따르면 ORB는 SIFT보다 약 100배 빠르면서도 여러 벤치마크에서 대등한 매칭 성능을 보였다. 게다가 특허 부담이 전혀 없었던 덕에 OpenCV에 기본 탑재되며 삽시간에 퍼져 나갔다. 2015년 발표된 [ORB-SLAM](https://arxiv.org/abs/1502.00956)(Mur-Artal et al., TRO 2015)은 이름에까지 ORB를 내걸었고, 2021년의 ORB-SLAM3에 이르기까지 front-end의 핵심 엔진을 바꾸지 않았다.

> 🔗 **차용.** Calonder et al.(2010)의 BRIEF → Rublee et al.(2011)의 ORB: 단순한 binary 비교 서술자에 intensity centroid 방향 보정과 학습 기반 점 쌍 선택을 얹어, 속도는 유지한 채 회전 취약점을 메웠다.

---

## 2.6 학습 기반 서술자

사람이 미분과 히스토그램, 비트 비교 규칙을 손으로 짜는 대신 데이터에서 직접 특징 추출기를 학습시키면 더 강건한 서술자가 나오지 않을까? 2016년 ECCV에 발표된 Yi et al.의 [LIFT](https://arxiv.org/abs/1603.09114)(Learned Invariant Feature Transform)는 검출, 방향 추정, 서술자 계산이라는 전통 파이프라인의 세 단계를 각각 CNN으로 학습시켜 하나로 이은 첫 시도였다.

2018년 CVPRW에서 DeTone et al.이 내놓은 [SuperPoint](https://arxiv.org/abs/1712.07629)는 한 걸음 더 나아가 keypoint 검출과 256차원 서술자 생성을 단일 네트워크 안에서 동시에 학습했다. 먼저 합성 기하 도형으로 코너의 감각을 익힌 뒤, 실제 이미지에 다양한 homography 변환을 걸어 얻은 예측을 자기 지도 정답처럼 삼는 *homographic adaptation* 전략이 주효했다. 이후 SuperPoint는 SLAM 연구에서 학습 기반 특징점의 표준 시험대로 자리 잡았고, 최근에는 [AnyLoc](https://arxiv.org/abs/2308.00688)(Keetha et al., RA-L 2024)처럼 DINOv2 같은 대형 foundation model의 표현을 장소 인식에 끌어다 쓰는 흐름도 두드러진다.

그럼에도 2026년 현재, 사람이 설계한 서술자는 여전히 현장을 지키고 있다. 연산 자원이 빠듯한 임베디드 보드 위에서 ORB는 SuperPoint보다 훨씬 가볍고 빠르다. 무엇보다 학습 서술자는 훈련 분포를 벗어난 낯선 환경에서 성능이 예측 불가능하게 출렁이는 반면, ORB는 어떤 조건에서 강하고 어디서 무너지는지 엔지니어가 사전에 가늠하고 통제하기 쉽기 때문이다.

---

## 아직 열린 문제

**학습 기반 서술자의 일반화 한계.** SuperPoint, R2D2, DISK 같은 학습 서술자는 학습 데이터와 닮은 도심·실내 영상에서는 고전 기법을 앞서는 경우가 많다. 하지만 수중, 열화상, 극심한 저조도처럼 학습 분포 바깥의 장면으로 들어가면 결과가 들쭉날쭉하다. 고전 서술자와 학습 서술자 중 어느 쪽을 택할지는 2026년에도 여전히 환경과 하드웨어 제약에 따라 판단이 갈린다.

**Wide-baseline 매칭의 실패 모드.** Harris나 ORB는 카메라 시점이 크게 벌어지면(wide-baseline) 급격히 무력해진다. ASIFT나 MSER 같은 affine-covariant 검출기가 일부 구간을 메워 주긴 했지만 근본적 해결에는 이르지 못했다. 최근 [DUSt3R](https://arxiv.org/abs/2312.14132)(Wang et al., CVPR 2024)는 두 이미지에서 명시적인 대응점 매칭을 건너뛰고 픽셀별 3D 좌표(pointmap)를 곧바로 회귀하는 다른 길을 열어젖혔다. 이 접근이 지난 수십 년간 다듬어 온 특징점 서술자를 어디까지 대체할지는 아직 지켜볼 일이다.

---

지금까지 살펴본 기법들은 이미지 한 장에서 쓸 만한 점을 뽑거나 두 장 사이에서 짝을 짓는 기술이다. 그렇다면 이렇게 얻은 대응점들을 바탕으로 수십, 수백 장의 사진을 기하적 모순 없이 한 공간에 세우는 일은 어떻게 가능할까? 이제 Structure from Motion의 세계로 들어간다(→ 3장).

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
