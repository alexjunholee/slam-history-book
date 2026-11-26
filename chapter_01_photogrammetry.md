# 1장 사진측량과 번들 조정

오늘날 SLAM 최적화 backend를 떠받치는 수학은 로봇공학이 아니라 측량학에서 태어났다. 19세기 후반 독일의 Albrecht Meydenbauer가 건축물을 사진으로 재는 방법을 체계화한 데 이어, 20세기 초 Carl Pulfrich는 두 시점에서 찍은 사진으로 거리를 읽어 내는 광학 기기를 내놓았다. 반세기가 흐른 1958년, 미국의 Duane C. Brown은 이 측량 문제를 수치 최소자승 문제로 정식화했다. 그리고 1999년 Bill Triggs, Philip McLauchlan, Richard Hartley, Andrew Fitzgibbon이 사진측량 문헌에 축적돼 있던 bundle adjustment 이론을 컴퓨터 비전 연구자의 언어로 번역해 내면서, 두 분야의 역사가 한데 만나게 된다.

---

## 1.1 20세기 초 유리판과 Stereophotogrammetry

1901년 함부르크 자연과학자 회의에서 [Carl Pulfrich](https://en.wikipedia.org/wiki/Carl_Pulfrich)는 Zeiss 광학연구소가 제작한 **입체 측량기(stereocomparator)**를 세상에 공개했다. 1899년 뮌헨에서 입체 거리계 시제품을 먼저 선보인 지 2년 만이었다. 원리는 명쾌했다. 서로 다른 두 위치에서 찍은 사진 건판을 나란히 놓고 같은 지점의 좌표 차이를 읽으면 거리를 계산할 수 있다. 두 시점 사이에서 같은 점이 어긋나 보이는 정도, 즉 시차(parallax)가 깊이에 반비례한다는 성질을 이용한 것이다. 삼각측량의 원리 자체는 고대 그리스 시대부터 알려져 있었지만, Pulfrich의 기기는 거기에 광학 계측의 정밀도를 실어 주었다.

그보다 한 세대 앞서 [Albrecht Meydenbauer](https://www.uni-marburg.de/de/fotomarburg/histfoto/gliederung/messbilder/messbildverfahren)는 문화재 보존을 위한 **건축 사진측량(architectural photogrammetry)**의 기틀을 다졌다. 1858년 베츨라 대성당 외벽을 실측하다 추락할 뻔한 아찔한 사고를 겪은 뒤, 그는 사람이 직접 줄자를 들고 기어오르는 대신 사진으로 건물을 재는 길을 찾았다. 그 집념은 1885년 프로이센 왕립 사진측량국(Königlich Preussische Messbild-Anstalt)의 설립으로 결실을 맺었다.

사진으로 공간을 재는 기술은 20세기에 들어 하늘로 올라갔다. 비행기에서 서로 겹치게 찍은 여러 장의 항공 사진과 지상의 기준점 몇 개를 엮어 촬영 위치와 지상 점의 좌표를 한꺼번에 구하는 기법을 aerotriangulation(항공 삼각측량)이라 불렀다. 물론 당시 계산을 수행하는 주체는 컴퓨터가 아니라 손과 기계식 계산기를 붙든 사람이었다.

> 🔗 **차용.** 현대 SLAM의 스테레오 깊이 추정은 Pulfrich의 stereocomparator와 같은 기하 원리 위에 서 있다. 두 카메라 사이의 거리(baseline)와 시차로 깊이를 역산한다는 점은 변함이 없다. 다만 Pulfrich가 유리 건판 위에서 눈으로 읽어 내던 좌표를 오늘날에는 이미지 센서의 픽셀 좌표가 대신할 뿐이다.

---

## 1.2 1958년 Brown과 수치 bundle adjustment

[Duane C. Brown](https://digital.hagley.org/08206139_solution)은 RCA 소속 엔지니어로, 플로리다 Patrick 공군기지의 미 공군 미사일 시험장(Air Force Missile Test Center)에서 측량 업무를 맡고 있었다. 그에게 떨어진 과제는 여러 관측 지점에서 찍은 카메라 자세와 여러 지상 점의 3차원 좌표를 동시에 추정하는 일이었다. Triggs et al.(1999)의 부록에 따르면, Brown과 동료들은 바로 이 미 공군 프로젝트를 위해 1957년부터 1959년 사이에 기본적인 bundle 방법의 뼈대를 세웠다.

1958년에 나온 보고서 "A Solution to the General Problem of Multiple Station Analytical Stereotriangulation"(RCA-MTP Data Reduction Technical Report No. 43, AFMTC-TR-58-8)은 **bundle adjustment**를 수치 계산 문제로 정식화한 기념비적 문헌이다. 비슷한 시기 해석적 사진측량을 연구하던 Helmut Schmid 역시 bundle adjustment의 공동 창시자로 나란히 꼽힌다. 다만 이 1958년 초기 정식화에서는 카메라 내부가 이미 교정돼 있다고 가정하고, 카메라 자세와 3차원 점 좌표만을 최적화 변수로 삼았다.

핵심 목표는 **reprojection error**를 줄이는 데 있다. 카메라 $i$가 실제로 관측한 2D 이미지 좌표는 $x_{ij} \in \mathbb{R}^2$이고, 3D 점 $X_j \in \mathbb{R}^3$을 카메라 내부 행렬 $K_i$와 외부 행렬 $[R_i | t_i]$로 이미지 평면에 투영한 예측 좌표는 $\pi(K_i, R_i, t_i, X_j)$이다. 이 두 좌표 사이의 거리를 재고, 모든 관측에 걸쳐 그 거리의 제곱합을 최소화한다.

$$E = \sum_{i,j} \| x_{ij} - \pi(K_i, R_i, t_i, X_j) \|^2$$

"Bundle"이라는 이름은 각 카메라 중심에서 3차원 점을 향해 뻗어 나가는 광선 다발(bundle of rays)에서 유래했다. 이 광선 다발들이 공간 속 3차원 점에서 오차 없이 만나도록 카메라 자세와 점 위치를 한꺼번에 비틀어 맞추는 작업이라는 뜻이다. 위 식은 오늘날 통용되는 일반화된 형태라 내부 행렬 $K_i$까지 표기에 담았지만, Brown의 1958년 보고서에서는 $K_i$를 고정된 기지값으로 두었다. 군사 측량 현장에서 태어난 bundle adjustment는 이후 수십 년 동안 사진측량 문헌 안에서 꾸준히 다듬어졌다.

> 🔗 **번역.** Triggs et al.(1999)은 초록에서 자신들의 논문이 사진측량의 bundle adjustment 이론과 기법을 "컴퓨터 비전 분야에서 이를 구현하려는 사람들"에게 건네기 위해 쓴 글임을 분명히 했다. 두 분야는 본질적으로 같은 최적화 문제를 각자의 용어와 관행 속에서 따로 풀고 있었고, 이 논문이 사진측량의 축적된 성과를 컴퓨터 비전 진영으로 번역해 온 셈이다. 군사 보안 때문에 컴퓨터 비전 연구자들이 이 방법을 처음부터 다시 발명해야 했다는 통설에는 직접적인 근거가 없다.

---

## 1.3 Levenberg와 Marquardt — 비선형 최적화

Brown이 세운 reprojection error 최소화는 비선형 최소자승 문제다. 투영 함수 $\pi$가 비선형이기 때문에 닫힌 꼴의 해가 존재하지 않으며, 결국 초기 추정치에서 출발해 해를 조금씩 고쳐 나가는 반복 수치 최적화에 의존해야 한다. 흥미롭게도 이 반복 계산이 발산하지 않도록 붙잡아 주는 수치 엔진은 측량학 바깥에서 먼저 만들어졌다.

1944년 [Kenneth Levenberg](https://cs.uwaterloo.ca/~y328yu/classics/levenberg.pdf)는 댐핑 파라미터 $\lambda$ 하나로 Gauss-Newton 방법과 steepest descent 방법 사이를 매끄럽게 오가는 기법을 발표했다. $\lambda$를 키우면 steepest descent에 가까워져 느리더라도 안전하게 내려가고, $\lambda$를 줄이면 Gauss-Newton처럼 해를 향해 성큼성큼 수렴한다. 기본 형태에서는 선형화한 정규방정식의 행렬 $\mathbf{J}^{\top}\mathbf{J}$ 대각 성분에 $\lambda \mathbf{I}$를 더해 행렬이 특이해지는 일을 막고 수치 안정성을 확보한다. 1963년 [Donald Marquardt](https://epubs.siam.org/doi/10.1137/0111030)가 같은 원리를 독자적으로 재발견해 한층 엄밀하게 정식화하면서, 이 방법은 오늘날까지 비선형 최소자승의 표준인 **Levenberg-Marquardt(LM) 알고리즘**으로 굳어졌다.

---

## 1.4 1999년 Triggs et al. — 사진측량의 결과를 정리하다

사진측량의 bundle adjustment와 수치 최적화의 LM 알고리즘이 컴퓨터 비전의 주류 문제와 정면으로 만난 결정적 계기는 1999년 ICCV와 함께 열린 Vision Algorithms Workshop이었다. 이 자리에서 Bill Triggs, Philip McLauchlan, Richard Hartley, Andrew Fitzgibbon이 발표한 ["Bundle Adjustment — A Modern Synthesis"](https://link.springer.com/chapter/10.1007/3-540-44480-7_21)는 측량학과 항공 사진측량 문헌 곳곳에 흩어져 있던 bundle adjustment(BA) 이론을 컴퓨터 비전의 언어로 집대성했다.

이 논문은 BA 문제가 지닌 희소 구조(sparsity)의 중요성을 낱낱이 파헤쳤다. 장면 속 한 3D 점은 전체 카메라 중 일부에만 찍히기 때문에, 선형화한 정규방정식의 행렬은 빈 칸이 대부분인 희소 블록 구조를 띤다. 여기서 점 변수를 먼저 대수적으로 소거하고(Schur complement) 카메라 변수만 남긴 작은 연립방정식을 풀면 계산량이 극적으로 줄어든다. 놀랍게도 논문에 따르면, Brown은 이미 1958년에 점 변수를 소거한 뒤 남은 카메라 방정식을 가우스 소거법으로 푸는 이 구조를 쓰고 있었다. 논문은 여기에 더해 기준 좌표계를 임의로 잡을 수 있기 때문에 생기는 자유도, 즉 gauge freedom 문제까지 체계적으로 정리했다.

이 이론적 번역은 곧 거대한 공학적 도약으로 이어졌다. 발표 7년 뒤인 2006년, Noah Snavely, Steven Seitz, Richard Szeliski는 [Photo Tourism(SIGGRAPH 2006)](https://phototour.cs.washington.edu/Photo_Tourism.pdf)을 통해 인터넷에서 긁어모은 불특정 다수의 사진만으로 노트르담 대성당이나 트레비 분수 같은 랜드마크를 자동 복원해 보였다. 다시 10년이 흐른 2016년, Johannes Schönberger와 Jan-Michael Frahm은 사진을 한 장씩 덧붙여 가며 재구성하는 incremental SfM을 한층 견고하게 다듬어 [COLMAP(CVPR 2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)을 공개했다. 비슷한 시기 대규모 연구 시스템들은 수백만 장에서 많게는 1억 장에 이르는 사진까지 한꺼번에 다루는 단계에 이르렀다.

---

## 1.5 Reprojection error가 지금 형태로 굳기까지

오늘날 우리에게 너무도 당연해 보이는 reprojection error 역시 처음부터 그 모습은 아니었다. 20세기 초 항공 삼각측량에서는 오차를 3D 지상 좌표계의 거리 차이로 재는 일이 흔했다. 그런데 3차원 공간에서 직접 거리를 비교하면 렌즈 왜곡이나 카메라 내부 교정에서 비롯된 오차가 지상 좌표 잔차 속에 뒤섞여, 어디서 문제가 생겼는지 가려내기가 어려웠다.

Brown은 1958년 보고서에서 이 비교의 기준을 3D 지상 좌표에서 2D 이미지 평면으로 옮겼다. 3차원 점을 이미지에 투영한 예측 위치와 실제 사진에서 측정한 픽셀 좌표를 같은 평면 위에서 맞대어 보자는 발상이었다. Triggs et al.(1999)의 부록에 따르면, 카메라 내부 파라미터까지 BA 안에서 함께 추정하는 self-calibration은 1964년 무렵 논의가 시작돼 1968년에 실제 구현으로 이어졌고, 렌즈 왜곡을 정교하게 묘사하는 카메라 모델 역시 1970년대 초에 크게 발전했다. 이 일련의 진전을 이끈 주역도 상당 부분 Brown과 동료들이었다.

여기에 통계적 해석이 더해지면서 공식은 완벽히 자리를 잡았다. 이미지 측정 노이즈가 등방성 가우시안 분포를 따른다고 가정하면, reprojection error 제곱합을 최소화하는 일은 정확히 최대우도 추정(MLE)과 일치한다. Triggs et al.(1999)이 이 공식을 컴퓨터 비전 연구자에게 익숙한 표기로 정리해 내면서, reprojection error는 다중 시점 기하와 SLAM의 공통 언어로 굳어졌다.

> 🔗 **차용.** 현대 SLAM에서 visual landmark의 관측 모델 $z = \pi(K, T, p) + \epsilon$은 Brown(1958)의 reprojection 공식을 그대로 물려받았다. Gauss-Newton으로 이 관측 잔차를 줄여 나가는 SLAM backend는 1958년 미 공군 기지의 항공 삼각측량 solver와 수학적으로 같은 골격을 공유한다.

---

## 1.6 오늘의 SLAM backend

반세기가 지난 지금도 그 뼈대는 건재하다. ORB-SLAM3는 g2o를 이용해 SE(3) 카메라 자세와 3D landmark 좌표를 한꺼번에 최적화하고, LIO-SAM은 GTSAM의 factor graph 위에서 비선형 최적화를 푼다. 딥러닝 시대의 대표 주자인 DROID-SLAM조차 GRU(gated recurrent unit) 기반 업데이트 연산자로 optical flow 보정량을 예측한 뒤, 맨 마지막에는 미분 가능한 dense bundle adjustment 층을 거쳐 기하학적 정합성을 맞춘다.

1999년의 투박한 행렬 표기는 Lie group과 factor graph라는 세련된 언어로 갈아입었고, 사람이 손으로 짜던 계산 일부는 신경망이 대신 맡게 됐다. 그럼에도 backend가 하는 본질적인 일은 여전히 관측 잔차를 최소화하는 것이다. 시각 BA는 여러 시점에 걸친 reprojection error로 자세와 지도를 깎아내고, LiDAR 시스템은 점과 점, 또는 점과 평면 사이의 거리처럼 센서의 물리적 특성에 맞는 잔차를 세울 뿐이다.

이 덕분에 SLAM 연구자는 사진측량 분야가 수십 년에 걸쳐 검증해 온 최적화 이론을 고스란히 가져다 쓸 수 있다. 다만 그 수학은 정적인 세계, 점 형태의 특징, 가우시안 노이즈라는 BA의 고전적 전제 위에서 완성된 것이므로, 현실의 동적 환경이 그 전제를 벗어나는 순간 한계도 함께 드러난다.

---

> 📜 **예언 vs 실제.** Triggs et al.(1999)은 마지막 절 "Summary and Recommendations"에서 문제의 희소 구조를 활용하는 일이 무엇보다 중요하다고 강조하며, 몇몇 예외적인 batch 문제를 제외하면 희소 분해에 기반한 Gauss-Newton 계열 방법을 권했다. 이후 SfM의 역사는 정확히 그 예언을 따라 거대해졌다. 물론 사진 수백만 장 규모로 덩치를 키우는 과정에서는 사진을 하나씩 추가할 때마다 BA를 국소적으로만 돌리거나, vocabulary tree로 닮은 이미지끼리만 추려 매칭하는 정교한 공학적 장치가 더해져야 했다.

---

## 아직 열린 문제

**비선형 BA의 전역 최적해 보장.** LM 알고리즘은 본질적으로 국소 최솟값(local minimum)을 찾는 방법이라 초기값이 참값에서 멀면 엉뚱한 3차원 구조에 갇힌다. 이 함정을 피하려고 epipolar geometry 추정, PnP(Perspective-n-Point, 3D 점과 영상 좌표로 카메라 자세를 복원하는 문제), 5-point algorithm 같은 기하 solver가 개발됐다. 실전에서는 이런 solver를 RANSAC 같은 강건 추정 틀 안에서 돌려 초기값을 얻고, 그 결과를 반복 최적화로 다듬는다. 대규모 환경에서 전역 최적해 자체를 수학적으로 보장하려는 convex relaxation 연구도 활발하지만(→ 7장), 실시간 SLAM이 요구하는 속도와 규모를 충족하며 이를 풀어내는 일은 여전히 과제로 남아 있다.

**사진측량의 정밀도와 Visual SLAM의 차이.** 항공 사진측량의 정확도는 엄격히 통제된 촬영 조건과 산출물 규격에 맞춰 관리된다. 잘 교정된 카메라와 정밀한 GCP(지상 기준점)를 동원하고, 최적화는 시간 제약 없는 오프라인에서 돌린다. 반면 실시간 Visual SLAM은 똑같은 수식을 들고나와 GPS도, 고해상도 계측 카메라도, 사후 계산의 여유도 없는 현장에서 매 프레임 답을 내놓아야 한다.

---

하지만 아무리 정교한 BA 공식이 준비돼 있어도, 식에 집어넣을 대응점 $(x_{ij})$가 없다면 최적화는 한 걸음도 내디딜 수 없다. 측량 현장에서 사람이 세우던 표적을 카메라 영상 속에서 스스로 찾아내고 짝짓는 일, 곧 특징점 검출과 서술자의 역사로 넘어가 보자(→ 2장).
