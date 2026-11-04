# Ch.1 — 사진측량과 bundle adjustment: Triggs 이전 100년

오늘날 SLAM 최적화 backend의 수학은 측량학에서 나왔다. 19세기 후반 독일의 Albrecht Meydenbauer는 건축물을 사진으로 측량하는 방법을 체계화했고, 20세기 초 Carl Pulfrich는 두 시점에서 찍은 사진으로 거리를 재는 광학 기기를 만들었다. 1958년 미국의 Duane C. Brown은 이 측량 문제를 수치 최소자승 문제로 정식화했다. 1999년 Bill Triggs, Philip McLauchlan, Richard Hartley, Andrew Fitzgibbon은 그때까지 사진측량 문헌에 쌓인 bundle adjustment 이론을 컴퓨터 비전 연구자가 읽을 수 있는 말로 정리했다.

---

## 1. 20세기 초 유리판과 Stereophotogrammetry

1901년 [Carl Pulfrich](https://en.wikipedia.org/wiki/Carl_Pulfrich)는 함부르크 자연과학자 회의에서 Zeiss 광학연구소가 만든 **입체 측량기(stereocomparator)**를 발표했다. 1899년 뮌헨에서 입체 거리계 시제품을 먼저 공개한 뒤였다. 이 장치로 두 시점에서 찍은 사진 건판을 놓고 같은 점의 좌표 차이를 읽으면 거리를 계산할 수 있었다. 두 시점 사이에서 같은 점이 어긋나 보이는 정도, 곧 시차(parallax)는 깊이에 반비례한다. 삼각법 자체는 고대 그리스부터 쓰였고, Pulfrich의 장치는 여기에 광학 기기의 정밀도를 더했다.

그보다 한 세대 앞서 [Albrecht Meydenbauer](https://www.uni-marburg.de/de/fotomarburg/histfoto/gliederung/messbilder/messbildverfahren)는 건축물 보존에 쓰는 **건축 사진측량(architectural photogrammetry)**을 체계화했다. 1858년 베츨라 대성당 외벽을 재다가 떨어질 뻔한 뒤, 그는 사람이 직접 올라가 재지 않고 사진으로 측량하는 방법을 궁리했다. 1885년에는 프로이센 왕립 사진측량국(Königlich Preussische Messbild-Anstalt)을 세웠다.

사진으로 측량하는 방법은 20세기 항공 측량으로 이어졌다. 비행기에서 겹치게 찍은 사진 여러 장과 지상 기준점 몇 개로 각 사진의 촬영 위치와 지상 점의 좌표를 구하는 방법을 aerotriangulation(항공 삼각측량)이라 한다. 계산은 사람이 손과 기계식 계산기로 했다.

> 🔗 **차용.** 현대 SLAM의 스테레오 깊이 추정은 Pulfrich의 stereocomparator와 원리가 같다. 두 카메라 사이의 거리(baseline)와 시차로 깊이를 구한다. Pulfrich는 유리 건판에서 좌표를 읽었고, 지금은 그 자리를 이미지 센서의 픽셀 좌표가 대신한다.

---

## 2. 1958년 Brown과 수치 bundle adjustment

[Duane C. Brown](https://digital.hagley.org/08206139_solution)은 RCA 소속으로 플로리다 Patrick 공군기지의 미 공군 미사일 시험장(Air Force Missile Test Center) 측량 업무를 맡은 엔지니어였다. 그가 다룬 문제는 여러 촬영 지점의 카메라 자세와 여러 지상 점의 좌표를 한꺼번에 추정하는 일이었다. Triggs et al.(1999)의 부록은 기본적인 bundle 방법을 Brown과 동료들이 1957~1959년 미 공군을 위해 개발했다고 적었다.

1958년 보고서 "A Solution to the General Problem of Multiple Station Analytical Stereotriangulation"(RCA-MTP Data Reduction Technical Report No. 43, AFMTC-TR-58-8)은 **bundle adjustment**를 수치 계산 문제로 정식화한 초기 문헌이다. 같은 시기 해석적 사진측량을 연구한 Helmut Schmid도 bundle adjustment를 처음 만든 사람으로 함께 거론된다. 이 1958년 방법은 카메라가 이미 교정돼 있다고 보고 카메라 자세와 점 좌표만 최적화했다.

**Reprojection error**는 카메라 $i$에서 관측한 2D 이미지 좌표 $x_{ij} \in \mathbb{R}^2$와, 3D 점 $X_j \in \mathbb{R}^3$을 내부 행렬 $K_i$와 외부 행렬 $[R_i | t_i]$로 투영한 예측 좌표 $\pi(K_i, R_i, t_i, X_j)$의 차이다. bundle adjustment는 이 차이의 제곱합을 최소화한다:

$$E = \sum_{i,j} \| x_{ij} - \pi(K_i, R_i, t_i, X_j) \|^2$$

"Bundle"이라는 이름은 각 카메라 중심에서 3D 점들로 뻗어 나가는 광선 다발(bundle of rays)에서 왔다. 이 광선들이 3차원 점에서 만나도록 카메라 자세와 점 위치를 함께 조정한다. 위 식은 오늘날 쓰는 형태여서 내부 행렬 $K_i$까지 변수로 두지만, Brown의 1958년 방법은 $K_i$를 알려진 값으로 놓았다. Brown의 작업은 군사 측량에서 시작했고, bundle adjustment는 그 뒤 사진측량 문헌 안에서 계속 다듬어졌다.

> 🔗 **번역.** Triggs et al.(1999)은 초록에서 이 논문이 사진측량의 bundle adjustment 이론과 방법을 "컴퓨터 비전 분야에서 이를 구현하려는 사람들"을 위해 정리한 글이라고 밝혔다. 두 분야는 같은 최적화 문제를 서로 다른 용어와 관행으로 다루고 있었고, 이 논문이 사진측량에 쌓인 결과를 컴퓨터 비전에 소개했다. 군사 보안 때문에 컴퓨터 비전이 이 방법을 따로 다시 발견해야 했다는 이야기에는 직접 근거가 없다.

---

## 3. Levenberg와 Marquardt — 비선형 최적화

Reprojection error 최소화는 비선형 최소자승 문제다. 닫힌 꼴의 해가 없어서 반복 수치 최적화로 푼다. 이 반복 계산을 안정시키는 방법은 측량학 바깥에서 나왔다.

1944년 [Kenneth Levenberg](https://cs.uwaterloo.ca/~y328yu/classics/levenberg.pdf)는 Gauss-Newton과 steepest descent 사이를 댐핑 파라미터 $\lambda$로 조절하는 방법을 발표했다. $\lambda$가 크면 steepest descent에 가까워져 안전하게 수렴하고, 작으면 Gauss-Newton처럼 빠르게 수렴한다. 기본 형태에서는 선형화한 정규방정식의 행렬 $\mathbf{J}^{\top}\mathbf{J}$에 $\lambda \mathbf{I}$를 더해 수치 안정성을 높인다. 1963년 [Donald Marquardt](https://epubs.siam.org/doi/10.1137/0111030)는 같은 아이디어를 따로 찾아내 더 명시적으로 정식화했다. 그 뒤 이 방법은 **Levenberg-Marquardt(LM) 알고리즘**이라는 이름으로 자리 잡았다.

---

## 4. 1999년 Triggs et al. — 사진측량의 결과를 정리하다

1999년 ICCV와 함께 열린 Vision Algorithms Workshop에서 Bill Triggs, Philip McLauchlan, Richard Hartley, Andrew Fitzgibbon은 ["Bundle Adjustment — A Modern Synthesis"](https://link.springer.com/chapter/10.1007/3-540-44480-7_21)를 발표했다.

이 논문은 측량학과 항공 사진측량 문헌에 흩어져 있던 bundle adjustment(BA) 이론을 컴퓨터 비전 용어로 다시 써서 한데 모았다. 논문은 BA 문제의 희소 구조를 자세히 다뤘다. 한 점은 일부 카메라에만 보이므로 정규방정식의 행렬이 희소 블록 구조를 이룬다. 점 변수를 먼저 소거하고(Schur complement) 카메라 변수만 남은 작은 연립방정식을 풀면 계산이 훨씬 빨라진다. 논문에 따르면 Brown의 1958년 방법이 이미 이렇게 점 변수를 소거한 뒤 남은 카메라 연립방정식을 가우스 소거법으로 풀었다. 논문은 기준 좌표계를 임의로 잡을 수 있어서 생기는 자유도(gauge freedom)도 따로 다루었다.

7년 뒤 Noah Snavely, Steven Seitz, Richard Szeliski의 [Photo Tourism(SIGGRAPH 2006)](https://phototour.cs.washington.edu/Photo_Tourism.pdf)은 인터넷에서 내려받은 사진으로 노트르담 대성당, 트레비 분수 같은 랜드마크를 자동으로 재구성했다. 노트르담 대성당은 사진 2,635장 가운데 597장, 트레비 분수는 466장 가운데 360장의 카메라 위치를 찾았다. 그로부터 10년 뒤 Johannes Schönberger와 Jan-Michael Frahm의 [COLMAP(CVPR 2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)은 사진을 한 장씩 더해 가며 재구성하는 incremental SfM을 더 견고하게 만들어 오픈소스로 공개했다. 논문은 입력 사진 7만 5천 장 가운데 2만 1천 장을 등록한 로마 재구성을 실었다. 그 무렵 연구 논문에서는 사진 수백만 장, 많게는 1억 장을 다루는 재구성 시스템도 나와 있었다.

---

## 5. Reprojection error가 지금 형태로 굳기까지

20세기 초 항공 삼각측량에서는 오차를 지상 좌표계의 거리 차이로 쟀다. 3D 공간에서 직접 비교하므로 렌즈나 캘리브레이션에서 생긴 오차가 지상 좌표 잔차에 섞여 따로 드러나지 않았다.

Brown은 1958년 보고서에서 비교하는 자리를 이미지 평면으로 옮겼다. 3D 점을 이미지에 투영한 위치와 실제로 관측한 이미지 좌표를 맞추는 방식이다. Triggs et al.(1999)의 부록에 따르면 카메라 내부 파라미터를 BA 안에서 함께 추정하는 self-calibration은 1964년 무렵 논의되기 시작해 1968년에 구현됐고, 렌즈 왜곡을 나타내는 카메라 모델은 1970년대 초에 크게 다듬어졌다. 이 발전도 상당 부분 Brown과 동료들이 이끌었다.

이미지 노이즈를 등방성 가우시안으로 놓으면 reprojection error 최소화는 최대우도 추정과 같아진다. Triggs et al.(1999)은 이 공식을 컴퓨터 비전 연구자에게 익숙한 표기로 정리했다.

> 🔗 **차용.** SLAM에서 visual landmark의 관측 모델 $z = \pi(K, T, p) + \epsilon$은 Brown(1958)의 reprojection 공식을 이어받았다. Gauss-Newton으로 이 오차를 최소화하는 SLAM backend는 1958년 항공 삼각측량 solver와 수학 구조가 같다.

---

## 6. 2026년의 SLAM backend

ORB-SLAM3는 g2o로 SE(3) 자세와 3D landmark 위치를 함께 최적화한다. LIO-SAM은 GTSAM의 factor graph로 비선형 최적화를 푼다. DROID-SLAM은 GRU(gated recurrent unit, 순환 신경망의 한 종류) 기반 업데이트 연산자로 optical flow 보정량을 구하고, 그 뒤에 미분 가능한 dense bundle adjustment 층을 둔다.

1999년의 행렬 표기는 Lie group과 factor graph로 바뀌었고, 사람이 설계하던 계산 일부는 신경망이 맡게 됐다. 지금도 backend는 관측 잔차를 최소화한다. 시각 BA는 여러 시점의 reprojection error로 카메라 자세와 지도를 추정하고, LiDAR 시스템은 점과 점 사이 거리나 점과 평면 사이 거리처럼 센서에 맞는 잔차를 쓴다.

그래서 SLAM 연구자는 사진측량에서 쌓인 최적화 이론과 검증 결과를 가져다 쓸 수 있다. 그 결과는 정적인 세계, 점 특징, 가우시안 노이즈라는 BA의 전제로 얻은 것이어서, 실제 환경이 이 전제와 다르면 그대로 쓸 수 없다.

---

> 📜 **예언 vs 실제.** Triggs et al.(1999)은 마지막 절 "Summary and Recommendations"에서 문제 구조를 활용하는 일이 아주 중요하다고 쓰고, 배치(batch) 문제에는 몇 가지 예외를 두고 희소 분해를 쓰는 Gauss-Newton 방법을 권했다. 이후 SfM은 이 방향으로 커졌다. 규모를 키우는 데에는 BA를 사진을 더할 때마다 부분적으로 돌리는 방법, vocabulary tree로 닮은 이미지끼리만 짝지어 매칭하는 방법 같은 엔지니어링이 함께 들어갔다.

---

## 🧭 아직 열린 것

**비선형 BA의 전역 최적해 보장.** LM 알고리즘은 국소 최솟값(local minimum)에 수렴한다. 초기값이 나쁘면 틀린 구조로 수렴한다. 초기값을 구하는 방법으로는 epipolar geometry 추정, PnP(Perspective-n-Point, 3D 점과 그 영상 좌표로 카메라 자세를 구하는 문제), 5-point algorithm 같은 기하 solver가 나왔다. 이런 기하 solver는 RANSAC처럼 이상치를 걸러 내는 강건 추정 절차 안에서 돌고, 그렇게 얻은 해를 반복 최적화로 다듬는다. 대규모 환경에서 전역 최적해를 보장하려는 convex relaxation 연구가 있고(Ch.6b 참조), 이를 실시간 SLAM에 필요한 속도와 규모로 돌리는 일은 남은 과제다.

**사진측량의 정밀도와 Visual SLAM의 차이.** 항공 사진측량의 정확도는 촬영 조건과 산출물이 요구하는 규격에 맞춰 평가한다. 교정된 카메라와 품질 좋은 GCP(지상 기준점)를 쓰고, 최적화는 오프라인에서 한다. 실시간 Visual SLAM은 같은 수식을 쓰지만 GPS가 없고, 카메라 해상도가 낮고, 추정을 바로 내놓아야 하는 조건에서 돈다.

---

BA를 돌리려면 먼저 최적화에 넣을 대응점을 찾아야 한다. Ch.2는 Harris corner와 optical flow로 이미지에서 이 대응점을 찾은 과정을 따라간다.
