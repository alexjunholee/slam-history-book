# Ch.1 — 사진측량과 bundle adjustment: Triggs 이전 100년

오늘날 SLAM 최적화 backend의 뼈대는 독일 측량학에서 태어났다. 20세기 초 Carl Pulfrich가 유리판 위에서 두 시점의 삼각측량을 손으로 계산하던 방법론은, Albrecht Meydenbauer의 사진측량 체계와 결합해 하나의 측량 전통을 형성했다. 그 전통이 1958년 Duane C. Brown의 수치 공식화를 거쳐, 1999년 Bill Triggs, Philip McLauchlan, Richard Hartley, Andrew Fitzgibbon의 손에서 컴퓨터 비전 언어로 번역되었다. Bundle adjustment의 1999년 종합은 100년 된 측량 유산을 컴퓨터 비전 커뮤니티가 쓸 수 있는 언어로 옮긴 작업이었다. Triggs et al.(1999)은 Pulfrich의 기하학에서 시차 원리를, Brown(1958)의 군사 측량에서 reprojection 정식화를 물려받았다. solver 골격은 Levenberg-Marquardt가 제공했다.

---

## 1. 20세기 초 유리판과 Stereophotogrammetry

1901년 [Carl Pulfrich](https://en.wikipedia.org/wiki/Carl_Pulfrich)는 함부르크 자연과학자 회의에서 Zeiss 광학연구소가 제작한 **입체 측량기(stereocomparator)**를 발표했다 (1899년 뮌헨에서 입체 거리계 시제품을 먼저 공개한 뒤의 정식 공개). 두 카메라 시점에서 같은 점을 찍고, 유리판 위의 좌표 차이를 읽어 거리를 산출하는 장치였다. 원리는 단순했다: 두 시점의 시차(parallax)가 깊이와 역비례한다. 수학은 그리스 시대의 삼각법이었고, 새로운 것은 광학 기기의 정밀도였다.

한 세대 앞선 흐름으로, [Albrecht Meydenbauer](https://de.wikipedia.org/wiki/Albrecht_Meydenbauer)는 건축물 보존을 위한 **건축 사진측량(architectural photogrammetry)**을 체계화했다. 1858년 그는 베츨라 대성당 외벽을 측량하다 추락사고를 겪은 뒤, 사진으로 대신할 수 있다는 생각을 품었다. 1885년 그는 프로이센 왕립 사진측량국(Königlich Preussische Messbild-Anstalt)을 설립했다.

이 두 흐름이 합쳐진 전통이 20세기 항공 측량으로 이어졌다. 비행기 위에서 지형을 찍고, 두 시점 사진으로 3차원 지도를 만드는 aerotriangulation이다. 수동 계산기의 시대였다.

> 🔗 **차용.** 현대 SLAM의 스테레오 깊이 추정은 Pulfrich의 stereocomparator와 같은 원리다. 두 카메라 간 baseline과 시차로 깊이를 구한다. 125년 전 유리판이 픽셀 배열로 바뀌었을 뿐이다.

---

## 2. 1958년 Brown과 수치 bundle adjustment

Pulfrich와 Meydenbauer가 광학 기기로 해결한 문제를, Brown은 수식으로 옮겼다.

[Duane C. Brown](https://digital.hagley.org/08206139_solution)은 미국 공군 탄도미사일 개발 체계의 측량 엔지니어였다. 위성 궤도와 지상 좌표를 함께 추정하는 문제, 즉 다수의 카메라 시점과 다수의 지상 제어점을 동시에 최적화하는 문제를 다루었다.

1958년 보고서 "A Solution to the General Problem of Multiple Station Analytical Stereotriangulation"(RCA-MTP Data Reduction Technical Report No. 43, AFMTC-TR-58-8)에서 Brown은 **bundle adjustment**를 수치적으로 공식화한 초기 문헌 중 하나를 남겼다 (같은 시기 Helmut Schmid도 공동 발명자로 함께 거론된다).

핵심은 **reprojection error**다. 카메라 $i$에서 관측된 2D 이미지 좌표 $x_{ij} \in \mathbb{R}^2$와, 3D 점 $X_j \in \mathbb{R}^3$을 내부 행렬 $K_i$·외부 행렬 $[R_i | t_i]$로 투영한 예측 좌표 $\pi(K_i, R_i, t_i, X_j)$의 차이를 최소화한다:

$$E = \sum_{i,j} \| x_{ij} - \pi(K_i, R_i, t_i, X_j) \|^2$$

"Bundle"이라는 이름은 각 카메라 중심에서 관측된 3D 점들로 뻗어 나가는 광선 다발(bundle of rays)에서 왔다. 그 광선들이 3차원 점에서 교차하도록 카메라 자세와 점 위치를 동시에 조정한다. 군사·첩보 응용에서 출발한 기법이 학계에 흡수되기까지는 40년이 걸렸다.

> 🔗 **차용.** 위성 geolocation 분야의 bundle 기법은 1990년대 이후 컴퓨터 비전 커뮤니티에 유입되었다. 군사 보안 분류(classified)로 묶인 기간 동안 학계는 같은 문제를 독립적으로 재발견했다. Triggs 1999는 그 두 흐름의 합류점이다.

---

## 3. Levenberg와 Marquardt — 비선형 최적화의 선구자

Brown이 최소화해야 할 목적함수를 손에 쥐었다면, 그것을 실제로 푸는 도구는 전혀 다른 곳에서 왔다.

Reprojection error 최소화는 비선형 최소제곱 문제다. 해석적 해가 없으므로 반복 수치 최적화가 필요하다.

1944년 [Kenneth Levenberg](https://cs.uwaterloo.ca/~y328yu/classics/levenberg.pdf)는 Gauss-Newton과 steepest descent를 댐핑 파라미터 $\lambda$로 보간하는 방법을 발표했다. $\lambda$가 클수록 steepest descent에 가까워져 안전하게 수렴하고, 작을수록 Gauss-Newton의 빠른 수렴을 활용한다. 이 전략은 목적함수에 $\lambda \mathbf{I}$를 더한 수식으로 표현되어 수치 안정성을 높였다. 컴퓨터 비전보다 20년 앞선 시점이었다. 1963년 [Donald Marquardt](https://epubs.siam.org/doi/10.1137/0111030)는 같은 아이디어를 독립적으로 재발견해 더 명시적으로 공식화했다. **Levenberg-Marquardt(LM) 알고리즘**이라는 이름으로 굳어졌다.

LM 알고리즘이 컴퓨터 비전에서 BA의 표준 solver가 되기까지 약 35년이 더 걸렸다. 분야 간 벽이 그 시간을 만들었다.

---

## 4. 1999년 Triggs et al. — 100년 유산 통합

Levenberg-Marquardt가 수치 도구를 준비해 둔 지 35년 뒤, 컴퓨터 비전은 마침내 그 도구를 가져갔다.

1999년 Vision Algorithms Workshop에서 Bill Triggs, Philip McLauchlan, Richard Hartley, Andrew Fitzgibbon은 ["Bundle Adjustment — A Modern Synthesis"](https://link.springer.com/chapter/10.1007/3-540-44480-7_21)를 발표했다.

이 논문이 한 일은, 20세기 측량학과 항공 사진측량에 흩어져 있던 BA 이론을 컴퓨터 비전 커뮤니티의 언어로 번역해 종합하는 것이었다. Triggs et al.이 기여한 것은 두 가지다. 첫째, sparse BA의 구조적 성질을 명시했다. Hessian 행렬의 희소 블록 구조(Schur complement trick)를 이용하면 카메라-점 결합 최적화를 훨씬 효율적으로 수행할 수 있다. 둘째, gauge freedom(기준틀의 임의성)을 명시적으로 다루었다.

이 논문이 나오고 7년 후, Noah Snavely의 [Photo Tourism(2006)](https://phototour.cs.washington.edu/Photo_Tourism.pdf)은 인터넷에 흩어진 사진 수백 장에서 노트르담·트레비 분수 같은 유명 랜드마크를 자동 재구성했다. 그로부터 10년 후 Johannes Schönberger의 [COLMAP(2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)은 수만~수십만 장 규모의 robust incremental SfM을 오픈소스로 공개하며, 이미 백만 장대까지 가 있던 연구 흐름을 누구나 재현할 수 있는 도구로 가져왔다. Triggs의 언어가 없었다면 그 경로는 훨씬 느렸을 것이다.

---

## 5. Reprojection Error — 개념의 형성

Triggs et al.이 어떤 오차 함수를 최소화하는지 서술했다면, 그 함수 자체가 어떻게 현재 형태로 정착했는지는 별도로 추적할 만하다.

이 오차 함수가 지금 형태로 자리 잡기까지 두 번의 전환이 있었다.

20세기 초 항공 삼각측량사들은 오차를 "지상 좌표계에서의 거리 차이"로 쟀다. 3D 공간에서 직접 비교하는 방식이어서, 카메라 렌즈가 틀어졌거나 캘리브레이션이 나빠도 그 오차는 지상 좌표 잔차에 녹아 보이지 않았다.

Brown은 1958년 보고서에서 비교 대상을 이미지 면으로 옮겼다. "3D 점을 이미지로 투영한 위치"와 "실제 이미지 관측"을 픽셀 단위로 맞추는 방식이다. 이렇게 하면 캘리브레이션 오차, 렌즈 왜곡, 외부 파라미터 오차가 하나의 잔차에 함께 드러난다. 통계적으로도 더 깔끔하다. 카메라 이미지 노이즈는 픽셀 단위의 등방성 가우시안으로 모델링할 수 있고, 그러면 reprojection error 최소화는 최대우도 추정과 같아진다.

Triggs et al.(1999)은 그 공식을 컴퓨터 비전 교과서 언어로 다듬어 표준화했다. 이 reprojection error minimization이 2026년 기준 factor graph 기반 SLAM backend의 핵심 측정 함수(measurement function)다.

> 🔗 **차용.** SLAM에서 visual landmark의 관측 모델 $z = \pi(K, T, p) + \epsilon$은 Brown(1958)의 reprojection 공식을 직접 계승한다. Gauss-Newton으로 이를 최소화하는 SLAM backend는 1958년 항공 삼각측량 solver와 수학적으로 동일한 구조를 가진다.

---

## 6. SLAM Backend의 뼈대 — 2026년까지

"SLAM"이라는 약어 자체는 1995년 [Durrant-Whyte·Leonard의 survey](https://ieeexplore.ieee.org/document/476131)에서 표준 용어로 정립됐지만, 그 backend의 수학은 이 챕터가 추적해 온 1958년 Brown의 reprojection 공식을 거의 그대로 물려받는다. 오늘날 SLAM 최적화 backend를 보자. ORB-SLAM3는 g2o를 통해 SE(3) 자세와 3D landmark 위치를 동시 최적화한다. LIO-SAM은 GTSAM의 factor graph 위에서 LM 알고리즘을 돌린다. DROID-SLAM은 GRU-based optical flow로 업데이트 방향을 구하지만, 최종 bundle adjustment 레이어는 여전히 Schur complement trick을 쓴다.

Lie group과 factor graph가 1999년의 행렬 표기를 대체했고, 신경망이 기술자 계산을 넘겨받았지만, 연산의 본질은 그대로다. 다수의 시점에서 관측된 점들의 reprojection error를 최소화해 카메라 자세와 맵을 동시에 추정한다. Pulfrich의 유리판이 픽셀 배열로 바뀌고, 손 계산이 GPU로 바뀌었을 뿐이다.

이 연속성은 분야의 강점이자 취약점이다. 강점: 100년의 수렴성 증명과 실용 검증이 무료로 따라온다. 취약점: BA의 전제(static world, point feature, Gaussian noise)가 현실 환경과 어긋날 때 대안이 없다.

---

> 📜 **예언 vs 실제.** Triggs et al.(1999)은 대규모 BA — 수천 대 카메라, 수백만 점 규모 — 로의 확장을 주요 도전으로 꼽은 것으로 널리 읽힌다. 그 방향성은 이후 20년에 걸쳐 달성되었다. 2006년 Snavely의 Photo Tourism이 인터넷 사진 수백 장으로 랜드마크를 재구성했고, 2016년 COLMAP은 그 흐름의 robust incremental SfM 구현체를 표준화했다. 다만 Triggs가 상상한 "직접 확장"이 아니었다. incremental BA와 visibility graph pruning 위에 vocabulary tree 루프 클로저가 얹힌, 엔지니어링 층의 결과였다. `[적중]`

---

## 🧭 아직 열린 것

**비선형 BA의 global optimum 보장.** LM 알고리즘은 국소 최솟값(local minimum)에 수렴한다. 초기값이 나쁘면 틀린 구조에 수렴한다. 초기화를 위한 방법들, 즉 5-point algorithm, PnP, epipolar geometry 추정이 차례로 등장했지만 이것들 역시 내부적으로 RANSAC과 반복 최적화에 의존한다. 대규모 환경에서 전역 최적을 보장하는 convex relaxation 기반 접근들이 연구되고 있으나, 실시간 SLAM 수준의 속도와 규모에서는 아직 실용화되지 않았다.

**사진측량 수준 정밀도와 Visual SLAM의 간극.** 항공 사진측량은 서브픽셀(0.1픽셀 이하) 정확도를 표준으로 요구한다. 교정된 카메라와 고품질 GCP(지상 기준점)가 있고, 최적화는 오프라인에서 수행한다. 실시간 Visual SLAM은 같은 수식 구조를 쓰면서도 GPS 없는 환경과 저해상도 카메라, 그리고 즉각적 추정이라는 제약 아래서 동작한다. 측량 분야의 정확도 기준(RMSE < 5 cm at 500 m 거리)에 Visual SLAM이 체계적으로 도달하는 환경은 제한적이며, 두 분야의 정확도 기준을 단일 프레임워크로 통합하는 시도는 진행 중이다.

---

BA의 전제(static world, point feature, Gaussian noise)가 무너지기 시작하는 것은 카메라가 이동하는 물체를 만났을 때다. 측량사는 다리를 측량하지 로봇 축구 경기장을 측량하지 않았다. 그 균열은 Ch.2에서 시작된다: 컴퓨터 비전이 단순한 특징점 매칭 너머로 움직이던 시기, Harris corner와 optical flow가 이 유산을 실시간으로 이어받으려 한 첫 번째 시도들이다.
