# Ch.2 — Classical CV 도구상자: Harris에서 SIFT까지, 그리고 ORB까지

Ch.1의 번들조정은 카메라 자세와 3D 점을 동시에 최적화하는 backend 문제를 다루었다. 그러나 그 최적화가 작동하려면 먼저 이미지에서 "대응하는 점"을 찾아야 한다. 측량사는 야지에서 직접 타깃을 세웠고, 컴퓨터 비전은 그 역할을 알고리즘에 맡겨야 했다. 이것이 feature detection·description의 문제다.

1977년 Hans Moravec은 Stanford Cart 프로젝트에서 스테레오 카메라로 "환경에서 두드러진 점"을 찾으려 했다. 텍스처가 풍부한 모서리가 추적하기 좋다는 직관은 있었지만, 수학적 정의는 없었다. 11년 후 Chris Harris와 Mike Stephens가 그 직관을 autocorrelation matrix의 eigenvalue로 공식화했다. Lucas와 Kanade는 그보다 7년 앞서 픽셀 추적의 틀을 세웠다. Lowe는 두 개념을 흡수해 scale과 rotation에 불변인 서술자를 만들었다. Rublee는 특허 없이 더 빠르게 같은 일을 했다. SLAM의 front-end는 이 계보 위에 앉아 있다.

---

## 2.1 코너라는 개념: Moravec에서 Harris까지

카메라가 조금 움직였을 때 영상 패치가 크게 변하는 점을 "코너"라 부른다. Moravec(1977)의 기준은 단순했다. 인접 픽셀과의 Sum of Squared Differences(SSD)가 상하좌우 모든 방향에서 크면 코너로 간주한다.

Harris와 Stephens는 1988년 BMVC 논문 "A Combined Corner and Edge Detector"에서 이를 연속 미분으로 대체했다. 이미지 $I$에서 점 $(x,y)$ 주변 창 $W$를 이동량 $(\Delta x, \Delta y)$로 움직일 때 강도 변화를 근사하면:

$$M = \sum_{(x,y) \in W} \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}$$

$M$의 두 eigenvalue $\lambda_1, \lambda_2$로 점의 성격을 구분한다. 둘 다 크면 코너, 하나만 크면 엣지, 둘 다 작으면 평탄한 영역. Harris는 행렬식을 직접 계산하지 않고 $R = \det(M) - k \cdot \text{tr}(M)^2$ 점수를 사용해 eigenvalue 분해를 피했다. $k$는 보통 0.04–0.06.

> 🔗 **차용.** Harris(1988)의 autocorrelation matrix 아이디어는 Moravec(1977)의 SSD 기반 코너 탐색을 연속 미분으로 정제한 것이다. 개념의 원형은 Stanford Cart 보고서에 있었다.

1994년 Jianbo Shi와 Carlo Tomasi는 "Good Features to Track"(CVPR 1994)에서 Harris 점수 대신 $\min(\lambda_1, \lambda_2)$를 직접 사용하는 것이 optical flow 추적에 더 안정적임을 보였다. 이 기준이 Shi-Tomasi 코너 검출로 불리며 OpenCV의 `goodFeaturesToTrack` 함수로 구현되었다. 30년이 지난 오늘도 그 함수는 그대로다.

---

## 2.2 추적의 원형: Lucas-Kanade와 KLT

Harris의 행렬 $M$은 점을 찾는다. 찾은 점을 다음 프레임에서 다시 찾는 것은 별개 문제다. 코너를 찾는 것과 코너를 추적하는 것은 다른 문제다. Bruce Lucas와 Takeo Kanade는 1981년 "An Iterative Image Registration Technique"에서 프레임 간 픽셀 이동을 밝기 불변 가정(brightness constancy assumption) 아래 최소화 문제로 정식화했다.

밝기 불변 가정: 픽셀 $(x,y)$의 강도는 움직임 전후로 같다.

$$I(x, y, t) = I(x + u, y + v, t + 1)$$

테일러 전개 후 선형화하면:

$$I_x u + I_y v + I_t = 0$$

이 방정식 하나에 미지수가 둘이다. Lucas-Kanade는 $3\times3$ 또는 $5\times5$ 창 안의 픽셀들이 같은 $(u,v)$로 움직인다는 가정을 추가해 overdetermined 시스템을 만들고 최소자승으로 푼다.

$$\begin{pmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{pmatrix} \begin{pmatrix} u \\ v \end{pmatrix} = -\begin{pmatrix} \sum I_x I_t \\ \sum I_y I_t \end{pmatrix}$$

왼쪽 행렬이 Harris의 구조 행렬 $M$과 동일하다. 코너 검출과 optical flow가 같은 수학 위에 있다는 뜻이다.

Tomasi는 1991년 tech report "Detection and Tracking of Point Features"에서 이 방법을 이미지 pyramid 구조로 확장해 큰 이동에도 수렴하도록 했다. KLT(Kanade-Lucas-Tomasi) 추적기라 불리는 이 구현이 VINS-Mono(2018)의 front-end에서 여전히 동작한다. 1981년의 최소자승 추적기가 38년 뒤 스마트폰 드론의 VIO에서 돌아가는 셈이다.

> 🔗 **차용.** Lucas-Kanade(1981) → KLT tracker → Qin et al. VINS-Mono(2018): 38년 전 optical flow가 실시간 VIO의 feature tracking backbone으로 그대로 살아있다.

---

## 2.3 SIFT: 불변성의 완성과 특허의 그림자

KLT는 같은 카메라가 조금씩 이동하는 상황에 맞다. 다른 카메라로, 다른 날 찍은 이미지에서 같은 점을 연결하는 문제는 다른 차원이다. 시점이 달라지면 같은 점이 패치 모양과 크기, 방향까지 달라져 단순 픽셀 비교가 통하지 않는다. 이 문제가 **서술자(descriptor)**의 문제다.

David Lowe(UBC)는 1999년 ICCV에서 아이디어를 공개하고, 2004년 IJCV에 확장 논문 "Distinctive Image Features from Scale-Invariant Keypoints"를 발표했다. SIFT(Scale-Invariant Feature Transform)는 두 단계로 구성된다.

**검출 단계**: DoG(Difference of Gaussians)를 여러 scale에서 계산해 local extremum을 keypoint로 선택한다. DoG는 Laplacian of Gaussian의 근사다. $L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)$를 Gaussian 스무딩 이미지라 하면:

$$D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)$$

여기서 $k$는 인접 scale 간 비율(보통 $2^{1/s}$, $s$는 octave당 스케일 수). 여러 octave에 걸쳐 극값을 찾으면 scale 변화에도 같은 점을 탐지할 수 있다.

**서술자 단계**: keypoint 주변 $16\times16$ 창을 $4\times4$ 블록으로 나누고 각 블록의 gradient 방향 히스토그램(8빈)을 연결해 128차원 벡터를 만든다. keypoint의 dominant gradient 방향을 기준으로 회전시키므로 회전 불변성도 확보한다.

결과는 scale, rotation, 부분적인 affine 변형에 강인한 128차원 서술자였다. KITTI 이전 시대, SLAM 벤치마크가 없던 시절에도 연구자들이 SIFT를 쓸 수밖에 없었던 이유다.

Lowe는 2000년 SIFT를 특허 출원해 2002년 등록했다(US6711293B1). 이 특허는 상업용 사용에 비용을 부과했고, 2020년 3월 만료 전까지 SIFT를 대체하려는 시도의 동기 중 하나가 되었다.

> 📜 **예언 vs 실제.** Lowe는 2004년 논문 §8에서 "향후 연구 방향으로 비디오 추적과 혼잡한 장면에서의 물체 인식으로의 확장"을 제안했다. 실제 전개는 달랐다. 비디오 추적은 KLT 기반 hybrid로 부분 실현되었으나, 물체 인식은 CNN이 2012년 이후 완전히 대체했다. SIFT의 128차원 서술자가 지향하던 장기 대응 문제는 deep feature(SuperPoint, R2D2)가 인수했다. `[기술변화]`

---

## 2.4 SURF: 속도를 위한 타협

SIFT의 128차원 서술자는 정확하지만 느렸다. 당시 데스크톱 CPU에서 이미지 한 장당 수백 밀리초. 실시간 SLAM에는 쓸 수 없었다. Herbert Bay(ETH Zurich)는 2006년 ECCV에 "SURF: Speeded-Up Robust Features"를 발표했다. 핵심 아이디어는 두 가지다.

DoG 대신 *Hessian 행렬의 행렬식*으로 keypoint를 탐지한다. integral image를 이용한 box filter로 Gaussian 이차 미분을 근사해 계산 속도를 높인다. 서술자는 64차원으로 SIFT의 절반. keypoint 주변을 $4\times4$ 하위 영역으로 나누고, 각 영역에서 Haar wavelet 응답 $d_x, d_y$의 합 $(\sum d_x,\, \sum d_y,\, \sum|d_x|,\, \sum|d_y|)$ 4값을 연결해 $4\times4\times4=64$차원을 구성한다. 128차원 확장(SURF-128)도 존재하나 기본값은 64차원이다.

SURF는 SIFT보다 3–7배 빨랐다. 그러나 128차원 vs 64차원의 정확도 차이가 있었고, 특허 문제도 Bay가 피하지 못했다(ETH Zurich 특허). SIFT와 SURF는 서로 다른 이유에서 대체되었다. 전자는 속도, 후자는 정확도와 특허. 두 문제를 동시에 푼 것이 ORB였다.

> 🔗 **차용.** Lowe(1999/2004)의 DoG scale-space → Bay(2006)의 Hessian integral image: scale-invariance를 얻는 두 가지 답. DoG는 이론적으로 우아하고, Hessian 근사는 공학적으로 빠르다.

---

## 2.5 ORB: binary descriptor와 특허 없는 세계

2011년 Ethan Rublee(Willow Garage), Vincent Rabaud, Kurt Konolige, Gary Bradski는 ICCV에 "ORB: An Efficient Alternative to SIFT or SURF"를 발표했다. 제목이 직접적이다. Willow Garage는 ROS의 산실이기도 했다. 로보틱스 연구자가 쓸 수 있는 feature를 만들겠다는 동기가 제목에 그대로 담겼다.

ORB는 두 기존 기법을 조합하고 개선했다.

**검출**: FAST(Features from Accelerated Segment Test, Rosten & Drummond 2006). 픽셀 주변 16개 점을 순환하며 충분히 밝거나 어두운 연속 호가 있으면 코너로 판정한다. SIFT의 DoG보다 10배 이상 빠르다. ORB는 FAST에 Harris 점수를 추가해 응답이 강한 것만 남긴다.

**서술자**: BRIEF(Binary Robust Independent Elementary Features, Calonder et al. 2010). keypoint 주변 패치에서 무작위로 선택한 점 쌍의 밝기를 비교해 비트열을 만든다. 256비트가 기본. 유클리드 거리 대신 Hamming 거리로 매칭하므로 XOR 연산 하나로 비교 가능하다.

BRIEF의 약점은 회전 불변성 부재였다. Rublee는 FAST 코너의 intensity centroid 방향으로 패치를 회전 보정해 **rBRIEF(rotated BRIEF)**를 만들었다. 이것이 ORB 서술자의 핵심이다.

$$\theta = \text{atan2}(m_{01},\, m_{10}), \quad m_{pq} = \sum_{x,y} x^p y^q I(x,y)$$

결과: SIFT보다 100배 빠른 계산 속도, 특허 없음, OpenCV에 즉시 통합. ORB-SLAM(Mur-Artal et al. 2015)은 이름 그대로 ORB를 기반으로 했고, 이후 삼부작까지 이어졌다. ORB-SLAM3의 2021년 발표 시점에서도 front-end는 ORB였다. 10년이 지나도 바뀌지 않은 선택이다.

> 🔗 **차용.** Calonder et al.(2010)의 BRIEF → Rublee et al.(2011)의 ORB: binary descriptor에 intensity centroid 기반 방향 추정을 추가해 회전 불변성을 확보했다.

---

## 2.6 학습 기반 descriptor의 등장

ORB가 실용적 정점이라면, 그 뒤의 질문은 자연스럽다. 손으로 설계한 규칙이 아닌 학습된 규칙이 더 나은가. 2016년 Yi et al.의 LIFT(Learned Invariant Feature Transform, ECCV 2016)는 검출·방향 추정·서술자 세 단계를 CNN으로 대체하려 했다. 단계별로 따로 학습한 세 네트워크를 파이프라인으로 연결하는 구조였다.

2018년 DeTone et al.의 [SuperPoint](https://arxiv.org/abs/1712.07629)(CVPRW 2018)는 homographic adaptation이라는 자기지도 학습법으로 keypoint 검출과 256차원 서술자를 동시에 학습했다. 합성 데이터로 사전 학습 후 실제 이미지에 적응. SLAM 커뮤니티에서 처음으로 주목받은 learned descriptor였다.

그러나 2026년 기준으로도 전통 descriptor가 사라지지 않았다. ORB는 임베디드 디바이스에서 SuperPoint보다 빠르고, 도메인 밖 이미지에서 일반화가 불안정한 learned descriptor보다 예측 가능한 동작을 보인다. AnyLoc(Keetha et al. 2023)처럼 DINOv2 기반 feature가 장소 인식에 도입되었지만, ORB-SLAM3는 2021년 발표 이후 여전히 ORB를 쓴다. Moravec의 1977년 직관이 2020년대 로봇에서 살아있는 이유다.

---

## 2.7 🧭 아직 열린 것

**학습 기반 descriptor의 일반화 한계.** SuperPoint, R2D2, DISK 등 learned descriptor는 학습 도메인에서 전통 방법을 능가하지만 새로운 환경(underwater, thermal, low-light)에서는 일관성이 없다. 어느 쪽이 낫다는 합의가 없다. 이 질문은 2026년에도 공개된 채로 남아 있다.

**Wide-baseline 매칭의 실패 모드.** Harris나 ORB 기반 매칭은 카메라 시점 변화가 45도를 넘으면 급격히 성능이 떨어진다. Affine-covariant detector(ASIFT, MSER)가 일부 보완했지만, 완전한 해법은 없다. DUSt3R(Wang et al. 2023)가 matching 자체를 회피하는 방향으로 돌파구를 열었지만, 이것이 descriptor 문제의 종말인지 우회인지는 아직 판단하기 이르다.

---

Harris의 직관과 Lowe의 불변성이 기반을 놓았고, Rublee의 속도 최적화가 그것을 현장으로 끌어냈다. 도구상자가 완성됐다. 이 기법들은 각자 이미지 한 장 혹은 두 장 사이에서 일하도록 설계됐다. 수십, 수백 장의 이미지를 동시에 기하적으로 일관되게 연결하려면 다른 층이 필요했다.

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

TODO: Lowe 2004 §8 Future Work 원문 인용 확인 (정확한 절 번호와 인용구 검증 필요)
TODO: Rublee 2011 §6 "replacing SIFT" 원문 표현 확인
