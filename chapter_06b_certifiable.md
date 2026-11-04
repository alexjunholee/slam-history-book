# Ch.6b — Certifiable SLAM: 해가 전역 최적인지 증명하기

Lu-Milios에서 g2o, GTSAM까지 이어진 연구에는 풀리지 않은 문제가 하나 있었다. 포즈 그래프 최적화(PGO)는 비볼록 문제라서 Gauss-Newton이나 LM이 내놓는 해가 지역 최솟값일 수 있다. 실무자들은 "odometry 초기값이 있으면 대체로 잘 풀린다"는 경험칙에 기댔다. 그러나 backend가 엉뚱한 해에 수렴하고도 아무 실패 신호 없이 멈추는 일이 있었다. 2015년 Georgia Tech에 있던 Luca Carlone은 Dellaert, MIT의 Rosen, Leonard 등과 함께 해를 수학적으로 검증하는 연구를 시작했다. Lagrangian duality를 쓴 이 연구는 Briales와 Gonzalez-Jimenez의 Cartan-Sync(2017), Rosen의 SE-Sync(2019), Yang과 Carlone의 TEASER, Papalia의 CORA로 이어졌다. 여기에 쓰인 도구는 모두 SLAM 바깥에서 왔다. Shor relaxation은 operations research에서, Burer-Monteiro factorization은 수리 최적화에서, Riemannian optimization은 미분기하에서, Kirchhoff의 Matrix-Tree 정리는 그래프 이론에서 왔다.

---

## 6b.1 지역 최솟값 문제

Ch.6 §6.7은 그래프 SLAM backend의 첫 문제로 초기값 의존성을 꼽았다. 비용 함수는 회전 변수 $\boldsymbol{R}_i \in \mathrm{SO}(3)$ 때문에 비볼록이다. 그래서 초기 추정이 참값에서 멀면 Gauss-Newton은 잘못된 해로 수렴한다. *The SLAM Handbook* 6장의 그림 6.1은 여러 층짜리 주차장을 돈 로봇의 실제 포즈 그래프로 이 문제를 보여 준다. SE-Sync가 구한 전역 최적해에서는 층이 제대로 쌓여 있다. 무작위 초기값에서 출발한 국소 최적화는 이와 전혀 다른 모양의 지역 최솟값에 멈춘다.

2000년대 후반까지 연구자들은 odometry로 좋은 초기값을 얻고, frontend에서 loop closure를 검증하고 outlier를 철저히 걸러 내는 식으로 대응했다. 둘 다 효과가 있었지만, 수렴한 값이 전역 최솟값인지 판정할 방법은 여전히 없었다. Huang과 Dissanayake는 2010년 무렵, 초기값이 좋아도 데이터가 모호하면 최적화기가 틀린 답에서 멈출 수 있다고 지적했다. PGO의 여러 특수한 경우가 NP-hard라는 것도 알려졌다. 그런데도 실제로는 g2o가 대개 잘 풀었다. 2010년대 중반 backend 이론 연구자들은 최악의 경우를 다루는 이론과 실제 결과가 왜 이렇게 다른지 파고들었다.

Gauss-Newton이 보장하는 것은 국소 조건뿐이다. 이상적인 국소 최솟값에서는 기울기가 0이고 Hessian이 양의 준정부호다. 실제 solver는 정지 기준이나 수치 문제 때문에 그 조건에 닿기 전에 멈추기도 한다. backend가 "수렴했다"고 알려 오면 사용자는 실패를 알아차리기 어렵다.

> 🔗 **차용.** Huber, Cauchy 같은 robust kernel은 robust statistics에서 왔다. [Black & Rangarajan (1996)](https://doi.org/10.1007/BF00131148)은 이런 robust 비용을 outlier 가중치 문제로 바꿔 쓸 수 있음을 보였고, 이 대응 관계는 뒤에 SLAM과 점군 정합에서 GNC(graduated non-convexity, 볼록한 근사에서 시작해 점점 원래 비용으로 옮겨 가는 방법)를 쓸 때 바탕이 됐다.

---

## 6b.2 Shor relaxation

PGO가 비볼록인 것은 회전 제약 $\boldsymbol{R}_i \in \mathrm{SO}(d)$ 때문이다. 직교성 조건 $\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}$은 이차식이다. 3차원에서 $\det(\boldsymbol{R})=+1$은 삼차식이지만, 열벡터가 오른손 좌표계를 이룬다는 조건을 외적 관계로 쓰면 이차 제약이 된다. 목적함수도 이차이므로 PGO는 **QCQP**(Quadratically Constrained Quadratic Program, 이차 제약 이차 계획)로 쓸 수 있다. QCQP에는 Naum Shor가 1987년에 발표한 convex relaxation이 있었다.

Shor는 항등식 $\boldsymbol{x}^\top \boldsymbol{M}\boldsymbol{x} = \mathrm{tr}(\boldsymbol{M}\boldsymbol{x}\boldsymbol{x}^\top)$을 이용해 새 변수 $\boldsymbol{X} \triangleq \boldsymbol{x}\boldsymbol{x}^\top$을 도입했다(lifting). 그러면 원래 QCQP는 "$\boldsymbol{X} \succeq 0$이고 rank가 1"이라는 조건 아래에서 선형 목적함수를 최소화하는 문제가 된다. 여기서 rank-1 제약을 버리면 볼록한 **SDP**(semidefinite program, 반정부호 계획)가 된다. 변수 수는 $n$에서 $n(n+1)/2$로 늘지만 문제는 볼록해진다.

$$d^* = \min_{\boldsymbol{X}\in\mathbb{S}^n} \mathrm{tr}(\boldsymbol{C}\boldsymbol{X}) \;\; \text{s.t.} \;\; \mathrm{tr}(\boldsymbol{A}_i\boldsymbol{X})=b_i,\; \boldsymbol{X}\succeq 0.$$

이 relaxation이 쓸모 있는 까닭은 부등식 $d^* \le p^*$에 있다. SDP의 최솟값은 원래 QCQP 최솟값의 하한이다. 후보해 $\hat{\boldsymbol{x}}$가 있으면 $f(\hat{\boldsymbol{x}}) - d^*$가 그 후보가 최적에서 떨어진 정도의 상한이 된다. "certifiable"이라는 이름은 여기서 나왔다. 전역 최적해를 직접 구하지 못해도, 손에 든 해가 최적에서 얼마나 떨어져 있는지 상한은 구할 수 있다. SDP 해 $\boldsymbol{X}^*$의 rank가 1이면 $\boldsymbol{X}^* = \boldsymbol{x}^*\boldsymbol{x}^{*\top}$에서 얻은 $\boldsymbol{x}^*$가 원래 QCQP의 전역 최적해다. SLAM에서 이런 경우가 얼마나 자주 생기는지가 그 뒤 논문들의 주제가 됐다.

이 연구는 Carlone이 2015년 ICRA와 IROS에서 발표한 두 논문, [Carlone & Dellaert 2015 "Duality-based verification techniques for 2D SLAM"](https://doi.org/10.1109/ICRA.2015.7139835)과 [Carlone et al. 2015 "Lagrangian duality in 3D SLAM"](https://arxiv.org/abs/1506.00746)에서 시작했다. 두 논문은 후보해가 전역 최적인지 Lagrangian dual로 검증하는 방법을 내놓았고, 실험에서 duality gap이 0인 경우가 많음을 보였다. 언제 gap이 0인지는 답하지 못했다.

2017년에는 [Briales와 Gonzalez-Jimenez의 Cartan-Sync (RA-L 2017)](https://doi.org/10.1109/LRA.2017.2718661)가 pose synchronization 문제로 같은 연구를 이었다. 수학 쪽에는 Boumal, Absil, Sepulchre 등의 Riemannian optimization이, 최적화 쪽에는 2003년에 나온 Burer-Monteiro의 low-rank SDP factorization이 이미 있었다. SE-Sync는 이 도구들을 한 알고리즘으로 엮었다.

---

## 6b.3 SE-Sync

[Rosen, Carlone, Bandeira, Leonard의 SE-Sync (IJRR 2019)](https://arxiv.org/abs/1612.07386)는 certifiable SLAM의 대표 논문이다. Rosen은 MIT에서 John Leonard의 지도로 박사학위를 받았다. Leonard는 Ch.4에서 Durrant-Whyte와 함께 EKF-SLAM 틀을 만든 연구자로 나왔다. 공저자 Afonso Bandeira는 SDP와 synchronization 문제를 연구하는 수학자다. 이 논문은 Shor relaxation, translation 소거, Burer-Monteiro low-rank 매개화, Boumal의 Riemannian Staircase를 PGO 하나에 함께 적용했다.

SE-Sync는 다음 순서로 문제를 바꾼다. 첫째, 회전을 고정하면 translation이 선형 최소자승 문제가 된다. SE-Sync는 이 점을 이용해 $\boldsymbol{t}$를 닫힌 형태로 소거한다. graph SLAM 연구자들이 오래전부터 알던 사실이지만, SE-Sync는 이를 convex relaxation의 첫 단계로 삼았다. 둘째, 남은 회전만의 문제 $\min_{\boldsymbol{R}\in\mathrm{SO}(d)^n} \mathrm{tr}(\tilde{\boldsymbol{Q}}\boldsymbol{R}^\top\boldsymbol{R})$에 Shor relaxation을 적용해 SDP로 바꾼다. 셋째, $dn \times dn$ 크기의 이 SDP를 interior-point method로 그대로 풀면 포즈가 수천 개만 돼도 감당할 수 없다. 그래서 Burer-Monteiro 재매개화 $\boldsymbol{Z} = \boldsymbol{Y}^\top \boldsymbol{Y}$로 Stiefel manifold 위의 저차원 비제약 문제로 바꾼다.

이 방법은 먼저 **exact recovery**에 기댄다. 측정 노이즈가 어떤 상수 $\beta$보다 작으면 SDP relaxation의 해 $\boldsymbol{Z}^*$는 유일하고 rank가 $d$이며, 원래 MLE의 전역 최솟값을 정확히 되찾는다. 일반적인 벡터 QCQP에서는 rank-1이 조건이지만, 여기서는 $\boldsymbol{Z}=\boldsymbol{R}^\top\boldsymbol{R}$이고 $\boldsymbol{R}\in\mathbb{R}^{d\times dn}$이므로 정확한 해의 rank가 $d$다. 다만 $\beta$는 참값에 따라 달라서 미리 알 수 없다. 여기에 Boumal 등의 결과가 더해진다. Stiefel manifold 위에서 찾은 2차 임계점이 rank-deficient하면 그 점이 곧 전역 최솟값이라는 것이다. Riemannian Staircase는 이 두 결과를 이용한다. 작은 rank에서 시작해 2차 임계점을 찾고 rank-deficiency를 검사한 다음, 조건이 맞지 않으면 rank를 하나 올린다. rank가 $dn + 1$에 닿으면 모든 $\boldsymbol{Y}$가 row rank-deficient가 되므로 유한한 단계 안에 반드시 멈춘다. Handbook 6장에 따르면 실제로는 대개 한두 단계면 충분하다.

SE-Sync 논문은 합성 데이터(sphere, torus, grid)와 실측 데이터(garage, cubicle, rim)에서 실험했다. 모든 데이터에서 SE-Sync와 Gauss-Newton은 같은 전역 최적해에 도달했고, SE-Sync가 3.3배(garage)에서 15.7배(rim) 빨랐다. SE-Sync는 해와 함께 사후(a posteriori) certificate도 낸다. g2o나 GTSAM은 해만 돌려준다. certificate로 확인되면 그 해가 전역 최적임이 증명된다. 확인되지 않으면 전역 최적인지 판정할 수 없다. Lu-Milios 이후 20여 년 만에 backend가 해와 함께 전역 최적성의 증명을 내놓게 됐다.

> 📜 **예언 vs 실제.** Rosen 등은 SE-Sync 논문 결론에서 측정 일부에 outlier가 섞인 경우에도 강건하고 통계적으로 효율적인 추정으로 넓히는 일을 내다봤다. 더 넓은 polynomial optimization 문제에도 구조를 이용한 SDP relaxation으로 certifiably correct 알고리즘을 설계할 수 있다고 보았다. 이 예상은 상당 부분 맞았다. Yang과 Carlone의 TEASER 계열, 2023년 Holmes와 Barfoot의 landmark SLAM 확장, 2024년 Papalia 등의 CORA(거리 측정 확장)가 뒤따랐다. visual SLAM의 perspective projection은 유리함수라서 polynomial optimization에 그대로 넣기 어렵다. Han과 Yang의 [XM](https://arxiv.org/abs/2502.04640)(RSS 2025)은 학습한 depth로 2D keypoint를 3D로 올려, 목적함수에서 projection을 없앤 scaled bundle adjustment를 만들었다. 이 문제를 SfM 파이프라인에서 SDP relaxation으로 풀어 전역 최적성 certificate도 냈다.

> 🔗 **차용.** SE-Sync의 핵심인 Burer-Monteiro factorization은 [Burer & Monteiro (2003)](https://link.springer.com/article/10.1007/s10107-002-0352-8)의 low-rank SDP 해법이다. [Boumal, Voroninski, Bandeira (2016)](https://arxiv.org/abs/1606.04970)는 이 방법에 Riemannian optimization 관점을 적용해 2차 임계점이 전역 최적이 되는 조건을 보였고, Rosen이 이 결과를 SLAM에 가져왔다.

---

## 6b.4 Graph Laplacian과 Fisher Information

전역 최적해를 찾았더라도, 그 추정이 참값과 얼마나 가까운지는 따로 물어야 한다. Cramér-Rao Lower Bound와 Fisher Information Matrix(FIM)가 이 질문을 다룬다. 회전을 고정한 단순한 PGO 모델에서는 FIM이 그래프의 가중 reduced Laplacian과 단위행렬의 Kronecker product로 정확히 나온다.

$$\mathcal{I} = \boldsymbol{J}^\top \boldsymbol{\Sigma}^{-1} \boldsymbol{J} = \boldsymbol{L}_w \otimes \boldsymbol{I}_3.$$

그래서 그래프 구조만 알면 실제 측정 없이도 추정 정확도를 근사할 수 있다. Kirchhoff의 Matrix-Tree 정리에 따르면 reduced Laplacian의 행렬식은 가중 spanning tree 수와 같다. 이 값은 D-optimality(정보 행렬의 행렬식을 기준으로 삼는 실험 설계 기준)와 대응한다. 대수적 연결도(algebraic connectivity, Fiedler value)는 E-optimality(가장 나쁜 방향의 분산을 기준으로 삼는 기준)와 대응한다. 1847년 Kirchhoff가 전기 회로망을 분석하려고 증명한 정리가 측정 선택과 active SLAM(정보를 많이 얻도록 로봇의 움직임을 고르는 SLAM)의 이론 근거가 됐다. active SLAM에서 FIM을 최대화하는 문제는 Laplacian 스펙트럼을 다루는 문제로 바뀐다.

이 연결은 [Kasra Khosoussi, Shoudong Huang, Gamini Dissanayake 등의 2014년 이후 연구](https://doi.org/10.1177/0278364918823086)에서 정리됐다. Khosoussi는 University of Technology Sydney에서 Huang과 Dissanayake의 지도로 2017년 박사학위를 받았고, 그 뒤 MIT에서 박사후연구원과 연구원으로 일했다. 3D PGO로 일반화하면 Laplacian과 SE(3) adjoint representation이 Kronecker product로 결합해, 그래프 연결 정보와 기하 정보를 따로 다룰 수 있게 된다. 3D PGO에서 Laplacian은 FIM보다 차원이 6배 작다. 그래서 측정을 고를 때 FIM 대신 Laplacian으로 근사하면 계산이 가볍고, 이 결과는 loop closure 선택의 수학적 근거가 된다.

Ch.4 §4.5에서 본 EKF-SLAM의 inconsistency 문제도 같은 질문과 이어진다. Julier와 Uhlmann이 2001년 지적한 EKF의 과신(over-confidence)을 CRLB로 다시 보면, 근사 선형화가 Fisher information을 실제보다 크게 잡는 문제다. Handbook 6장은 6.1절에서 certifiable solver를, 6.2절에서 최적해의 정확도를 다룬다. 전역 최솟값을 찾는 문제와 그 정확도를 따지는 문제를 함께 봐야 하기 때문이다.

> 🔗 **차용.** [Kirchhoff의 Matrix-Tree 정리(1847)](https://en.wikipedia.org/wiki/Kirchhoff%27s_theorem)는 전기 회로망 분석 도구로 만들어져 조합론과 실험 설계 분야에서 쓰였다. 2010년대 Khosoussi 등의 연구로 SLAM의 측정 선택과 active SLAM에도 쓰이게 됐다.

---

## 6b.5 확장과 한계: TEASER, CORA, Lasserre relaxation

SE-Sync가 나온 뒤 연구는 outlier에 강한 certifiable estimator와, 거리, landmark, 비등방 잡음(anisotropic noise) 같은 측정 모델로 넓어졌다.

먼저 outlier 문제다. Ch.6 §6.7에서 본 것처럼, loop closure 검증을 통과한 잘못된 매칭이 그래프에 섞인다. outlier 비율이 어느 수준을 넘으면 Huber나 Cauchy kernel을 써도 최적화가 실패한다. certifiable 연구는 2017년 무렵부터 이 문제를 다뤘다.

대표적인 예가 [Yang, Shi, Carlone의 TEASER (TRO 2021)](https://arxiv.org/abs/2001.07715)다. TEASER는 3D 점군 정합 벤치마크에서 scale을 알 때 대응 가운데 outlier가 99%를 넘어도 견뎠다. 대응이 틀려도 비용이 일정 값 이상 커지지 않는 truncated least squares(TLS) 비용으로 문제를 다시 쓰고, scale, rotation, translation 추정을 분리해 차례로 푼다. scale과 translation은 adaptive voting으로 다항 시간 안에 풀고, rotation은 SDP로 relaxation해 certificate를 낸다. 뒤이은 [Yang & Carlone (TPAMI 2022)](https://arxiv.org/abs/2109.03349)는 TLS를 비롯한 robust 비용을 polynomial optimization 문제로 다시 쓰고, Lasserre 계층보다 작은 sparse SDP relaxation을 제안했다.

거리 측정을 함께 쓰는 range-aided SLAM은 [Papalia 등의 CORA (TRO 2024)](https://arxiv.org/abs/2302.11614)가 다뤘다. 거리 측정 항 $(\|\boldsymbol{t}_j - \boldsymbol{t}_i\| - \tilde r_{ij})^2$에는 노름(제곱근)이 들어 있어 그대로는 이차식이 아니다. CORA는 방향을 나타내는 보조 단위벡터 $\boldsymbol{b}_{ij} \in S^{d-1}$를 따로 변수로 두어 이 문제를 QCQP로 바꿨다. Handbook 6장에 따르면 이 relaxation은 앞의 문제들과 달리 노이즈가 작아도 언제나 exact하지는 않다. 로봇 여러 대가 서로 거리만 재고 상대 포즈를 재지 않으면 대개 exact하지 않았다. 그래도 여러 실제 SLAM 문제에서는 exact했고, 그래프의 연결 정도가 exact 여부에 크게 영향을 주었다.

Landmark 쪽에서는 [Holmes & Barfoot (RA-L 2023)](https://doi.org/10.1109/LRA.2023.3238173)이 Schur complement로 landmark를 미리 소거해 SE-Sync에 바로 넣을 수 있는 형태로 만들었다. Holmes, Khosoussi, Rosen은 Handbook 6장의 공저자다.

한계도 드러났다. anisotropic noise와 truncated-quadratic outlier까지 다루려고 POP(Polynomial Optimization Problem)로 일반화하면 Lasserre moment relaxation이 필요하다. moment relaxation이 만드는 SDP에는 중복 제약이 많아 **degenerate**해진다. 그러면 Riemannian Staircase의 수렴 보장에 필요한 constraint qualification이 대개 성립하지 않고, dual 해가 무수히 많아 구현도 어려워진다. Handbook은 Riemannian Staircase를 이런 SDP에 쓸 수 없다고 정리한다. 2022년 Yang 등은 이런 relaxation을 푸는 전용 solver를 내놓았지만, 아직 국소 solver보다 상당히 느리다. Visual SLAM과 VIO에서는 perspective projection과 IMU preintegration을 이 정식화에 넣는 문제도 남아 있다.

> 📜 **예언 vs 실제.** Carlone 등은 2015년 IROS 논문 결론에 "Many theoretical questions remain open, e.g., under which conditions the duality gap is zero?"라고 적었다. 10년이 지난 지금 답은 일부만 나왔다. SE-Sync의 exact recovery 정리가 "노이즈가 $\beta$ 이하"라는 충분조건을 주었지만, 실제 SLAM 인스턴스에서 $\beta$를 미리 계산하는 방법은 없다. 2026년에도 tightness가 언제 깨지는지 미리(a priori) 알려 주는 조건 대신, 문제마다 사후 certificate를 계산해 확인한다.

---

## 🧭 아직 열린 것

**Tightness가 깨지는 경계.** SE-Sync의 exact recovery 정리는 "노이즈 $\beta$ 이하"라는 충분조건이다. 실제 인스턴스에서 $\beta$를 미리 계산하는 일은 남은 과제다.

**Visual SLAM, VIO에 넣기.** perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$는 유리함수라 polynomial optimization에 넣으려면 변형이 필요하다. 분모를 곱해 다항식으로 바꿔도 feature마다 변수와 보조 제약이 늘어서, ORB-SLAM3처럼 map point가 수천 개면 SDP가 너무 커져 실시간으로 풀 수 없다. Forster의 2015년 IMU preintegration도 exponential map과 bias drift가 얽혀 POP로 쓰기 어렵다. 2026년에도 Ch.7, Ch.8, Ch.13에서 다루는 주요 visual SLAM, VIO 시스템은 certifiable 보장을 받지 못한다.

**Online certification과 규모.** SE-Sync는 배치 방법이다. iSAM2가 배치 SAM을 증분화했듯 certifiable solver도 증분화하려는 연구가 있다. 측정이 들어올 때마다 certificate를 갱신하는 방법으로 warm-start, rank 증분, 부분 certificate 합성이 연구되고 있다. 도시 규모 그래프에서 moment relaxation solver가 느린 것도 여전히 문제다.

**Outlier가 다수인 경우.** TEASER처럼 outlier가 대다수인 정합에서 잘 작동하는 추정기도 있지만, 그 보장은 특정 문제와 오염 조건에서만 성립한다. 일반적인 SLAM 문제로 넓히는 일은 아직 남아 있다. 그럴듯한 해가 여러 개일 때는 list-decodable regression 같은 다중 가설 방법도 연구된다. 2024년에는 Cheng, Shi, Carlone의 후속 작업이 나왔다.

---

Rosen, Khosoussi, Holmes, Dissanayake, Barfoot, Carlone이 함께 쓴 *The SLAM Handbook* 6장은 이 주제를 149~181쪽(2026년 4월 21일 공개본 기준)에 걸쳐 다룬다. 같은 10년 동안 Ch.12, Ch.13, Ch.16에서 다루는 학습 기반 SLAM은 신경망이 해를 직접 예측하는 쪽으로 나아갔다.

Ch.19 §19.4는 이 장에서 풀리지 않은 문제들을 학습 기반 시스템의 불확실성 문제와 함께 다시 다룬다. Ch.7은 backend를 주어진 것으로 두고, frontend를 다룬다.
