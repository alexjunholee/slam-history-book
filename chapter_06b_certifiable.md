# Ch.6b — Certifiable SLAM: 지역 최솟값을 넘어서

Ch.6이 기록한 Lu-Milios에서 g2o·GTSAM에 이르는 계보는 한 가지를 남겨두었다. 포즈 그래프 최적화는 비볼록 문제다. Gauss-Newton·LM이 내놓는 해는 지역 최솟값일 수 있다. 실무자들은 "odometry 초기값이 있으면 대체로 잘 풀린다"는 민속적 관찰로 지내왔지만, 어느 현장에서는 백엔드가 엉뚱한 지점에서 수렴했는데도 경고음은 울리지 않았다. 2015년 MIT의 Luca Carlone이 그 민속을 수학으로 대체하기 시작했다. Carlone의 Lagrangian duality 시도에서 2019년 Rosen의 SE-Sync로, 이어 Briales-Gonzalez-Jimenez의 Cartan-Sync, Yang-Carlone의 TEASER, Papalia의 CORA까지 — 이 계보는 SLAM 백엔드를 "경험적으로 잘 풀리는 비볼록 최적화"에서 "전역 최적성이 증명 가능한 convex surrogate"로 다시 쓴다. 도구는 모두 SLAM 바깥에서 왔다. 오퍼레이션스 리서치의 Shor relaxation, 수학 최적화의 Burer-Monteiro factorization, 미분기하의 Riemannian optimization, 그래프 이론의 Kirchhoff Matrix-Tree. 10년간 이것들을 한 테이블에 모은 사람들의 이름이 이 챕터의 본문이다.

---

## 6b.1 지역 최솟값이라는 오래된 불안

Ch.6 §6.7은 그래프 SLAM 백엔드의 첫 문제로 초기값 의존성을 꼽았다. 비용 함수가 회전 변수 $\boldsymbol{R}_i \in \mathrm{SO}(3)$ 위에서 비볼록이기 때문에, 초기 추정이 참값에서 멀면 Gauss-Newton은 엉뚱한 분지로 빨려 들어간다. Handbook §6.1의 parking garage 예시가 증상을 극명하게 보여준다. 같은 입력에서 무작위 초기화 네 번 중 하나만 SE-Sync가 도달한 전역 최솟값에 붙고, 나머지 셋은 육안으로도 바닥이 접힌 지역 최솟값에 안착한다.

2000년대 후반까지 커뮤니티의 대응은 두 갈래였다. odometry를 신뢰해 초기값 품질을 확보하거나, 루프 클로저 검증과 아웃라이어 제거를 전단에서 철저히 하거나. 둘 다 유효했지만, 수렴한 값이 진짜 최솟값인지 판정하는 도구는 아니었다. Huang과 Dissanayake가 2010년 무렵 짚은 문제는 단순했다. 초기값이 아무리 좋아도 데이터 자체가 모호하면 최적화기는 틀린 답에 가서 멈출 수 있다. PGO가 NP-hard라는 것도 그 무렵 정식화됐다. 그런데도 현장에서는 g2o가 대체로 잘 풀렸다. 이 간극 — 이론은 최악을 말하는데 실무는 평균을 보는 간극 — 이 2010년대 중반 백엔드 이론 연구자들이 파고든 자리다. Gauss-Newton이 멈춘 모든 지점은 *국소적으로는* 최적이다. 기울기가 0이고 헤시안도 양정치다. 그런데도 답은 전혀 다르다. 백엔드가 "수렴했다"고 신호를 보내는 순간이 실패가 가장 눈에 띄지 않는 순간이기도 하다.

> 🔗 **차용.** Ch.6의 robust kernel(Huber, Cauchy)과 이 챕터의 GNC는 [Black & Rangarajan (1996)](https://cs.brown.edu/people/mjblack/Papers/ijcv1996.pdf)의 robust statistics·이중성 정리를 공유한 뿌리에서 갈라졌다. 한쪽은 비용 가중으로 아웃라이어 영향을 줄였고, 반대쪽은 같은 원리를 비볼록성 회피에 전용했다.

---

## 6b.2 Shor relaxation — 바깥에서 들어온 무기

PGO의 비볼록성은 회전 제약 $\boldsymbol{R}_i \in \mathrm{SO}(d)$에서 온다. 이 제약은 사실 직교성 조건 $\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}$과 $\det(\boldsymbol{R}) = +1$로, 이차 방정식으로 쓸 수 있다. 목적함수도 이차다. 결국 PGO는 **QCQP**(Quadratically Constrained Quadratic Program)로 정확하게 떨어진다. 그리고 QCQP에는 1987년 이후 오퍼레이션스 리서치 분야에서 검증된 convex relaxation 도구가 있었다. [Naum Shor의 1987 relaxation](https://link.springer.com/article/10.1007/BF01582220)이다.

Shor의 아이디어는 $\boldsymbol{x}^\top \boldsymbol{M}\boldsymbol{x} = \mathrm{tr}(\boldsymbol{M}\boldsymbol{x}\boldsymbol{x}^\top)$ 항등식으로 리프팅 변수 $\boldsymbol{X} \triangleq \boldsymbol{x}\boldsymbol{x}^\top$을 도입해 원 QCQP를 "$\boldsymbol{X} \succeq 0$이면서 rank-1" 위의 선형 목적 문제로 바꾸고, rank-1 제약을 버려 볼록한 **SDP**를 얻는 거래다. 탐색 공간이 $n$에서 $n(n+1)/2$로 늘지만 볼록성을 얻는다.

$$d^* = \min_{\boldsymbol{X}\in\mathbb{S}^n} \mathrm{tr}(\boldsymbol{C}\boldsymbol{X}) \;\; \text{s.t.} \;\; \mathrm{tr}(\boldsymbol{A}_i\boldsymbol{X})=b_i,\; \boldsymbol{X}\succeq 0.$$

쓸모는 이중성 부등식 $d^* \le p^*$에 있다. SDP 최솟값은 원 QCQP 최솟값의 아래쪽 경계다. 후보해 $\hat{\boldsymbol{x}}$가 있을 때 $f(\hat{\boldsymbol{x}}) - d^*$가 그 후보의 최적성 간극의 상한이 된다. 여기서 "certifiable"이라는 이름이 나온다. 전역적으로 못 풀어도, 가진 해가 얼마나 나쁜지의 상한은 풀 수 있다. SDP 해 $\boldsymbol{X}^*$가 rank-1로 떨어지면 $\boldsymbol{X}^* = \boldsymbol{x}^*\boldsymbol{x}^{*\top}$에서 $\boldsymbol{x}^*$가 원 QCQP의 전역 최솟값이다. 이 "favorable situation"이 SLAM에서 얼마나 자주 일어나는지가 이후 논문들의 주제가 된다.

이 계보의 출발점은 Carlone이 2015년 IROS와 ICRA에서 발표한 두 편의 논문, [Carlone et al. 2015 "Lagrangian duality in 3D SLAM"](https://arxiv.org/abs/1506.00746)과 [Carlone & Dellaert 2015 "Planar pose graph optimization"](https://doi.org/10.1109/ICRA.2015.7139264)이다. 2D PGO에서 duality gap이 대개 0임을 경험적으로 보였고, 3D로 확장 가능함을 시사했다. Carlone은 2014년 TRO 서베이에서 g2o·GTSAM 초기화 기법을 정리한 직후였고, odometry와 루프 클로저가 충돌할 때 최적화가 자주 틀린 지점에서 멈추는 것을 본 뒤였다. 2015년 논문은 "duality gap이 보통 0"임을 보고할 뿐, 언제 성립하는지의 닫힌 조건은 주지 못했다.

같은 시기 [Briales & Gonzalez-Jimenez (2017)](https://arxiv.org/abs/1702.03235)의 Cartan-Sync가 SO(3) synchronization으로 같은 프로그램을 밀었다. 수학 쪽에서는 Boumal·Absil·Sepulchre가 Riemannian optimization을, 최적화 쪽에서는 Burer-Monteiro의 low-rank SDP factorization이 2003년부터 자리잡고 있었다. 흩어진 재료들이 2019년 한 편의 논문에서 조립된다.

---

## 6b.3 SE-Sync — Rosen 2019가 조립한 것

[Rosen, Carlone, Bandeira, Leonard의 SE-Sync (IJRR 2019)](https://arxiv.org/abs/1612.07386)는 certifiable SLAM의 캐논이다. Rosen은 MIT에서 John Leonard의 박사과정을 마쳤고, Leonard는 Ch.4의 Durrant-Whyte와 함께 1990년대 초 "SLAM"이라는 이름을 자리잡게 한 MIT 연구자였다. 공저자 Afonso Bandeira는 SDP·synchronization 수학 쪽 전문가로 rank-deficient 2차 임계점의 전역성 증명을 맡았다. 로보틱스·SLAM·수학 최적화·응용수학 네 계보의 배경이 이 논문이 무엇을 조립했는지를 말해준다. 이 논문이 한 일은 조립이었다. Shor relaxation, translation elimination, Burer-Monteiro low-rank parameterization, Boumal의 Riemannian staircase — 각각 다른 계보에서 10여 년씩 숙성된 재료들을 PGO라는 한 문제 위에서 맞물리게 했다.

조립의 순서는 세 단계다. 첫째, 회전 고정 시 translation이 선형 최소자승이 된다는 관찰에서 $\boldsymbol{t}$를 닫힌 형태로 소거한다(Problem 6.2). Ch.6의 graph SLAM 계보가 오래전부터 알던 사실을 Carlone이 2014년 TRO 서베이에서 명시했고, Rosen이 convex relaxation의 첫 단계로 집어넣었다. 둘째, 남은 rotation-only 문제 $\min_{\boldsymbol{R}\in\mathrm{SO}(d)^n} \mathrm{tr}(\tilde{\boldsymbol{Q}}\boldsymbol{R}^\top\boldsymbol{R})$에 Shor relaxation을 적용해 SDP로 리프팅한다(Problem 6.3). 셋째, $dn \times dn$ 차원 SDP는 그대로 풀면 interior-point method가 수천 포즈에서 무너지므로 Burer-Monteiro 재파라미터화 $\boldsymbol{Z} = \boldsymbol{Y}^\top \boldsymbol{Y}$로 Stiefel manifold 위의 저차원 비제약 문제로 바꾼다(Problem 6.4).

두 정리가 이 조립을 정당화한다. Theorem 6.1 **exact recovery**: 측정 노이즈가 어떤 상수 $\beta$보다 작으면 SDP relaxation의 유일 해가 원 MLE의 전역 최솟값을 rank-1로 품는다. "어떤 노이즈까지 버티는가"에 대한 첫 정량적 답이었다. 다만 $\beta$는 ground-truth에 의존해 사전에는 모른다. Theorem 6.2는 Boumal et al.의 결과로, Stiefel manifold 위에서 찾은 2차 임계점이 rank-deficient하면 곧 전역 최솟값임을 보장한다. 이 두 정리가 Riemannian Staircase를 가능케 한다. rank를 작게 두고 시작해 2차 임계점을 찾고 rank-deficiency를 검사, 안 맞으면 rank를 하나 올린다. rank가 $dn + 1$에 닿으면 모든 $\boldsymbol{Y}$가 row rank-deficient가 되므로 유한 단계 내 반드시 멈춘다. 실무 데이터셋에서는 보통 한 계단이면 끝난다.

sphere·torus·garage 벤치마크에서 SE-Sync는 g2o·GTSAM 수준 속도로 수렴하며 a posteriori certificate를 함께 냈다. g2o·GTSAM은 빨랐지만 답을 언제 믿을지 침묵했고, Rosen의 알고리즘은 끝에 suboptimality bound를 하나 더 토해낸다. 이 bound가 0이면 해는 증명 가능하게 전역 최적이다. Lu-Milios 이후 20년 만에 백엔드가 "이 해가 진짜 최솟값인가"에 '예/아니오'를 찍을 수 있게 됐다.

> 📜 **예언 vs 실제.** Rosen은 IJRR 2019 논문 §8.2에서 "우리가 보인 algebraic simplification은 anisotropic noise·outlier·다양한 센서 모달리티로 확장될 수 있을 것"이라 적었다. 그 예언은 부분적으로 적중했다. 2023년 Holmes-Barfoot의 landmark-SLAM 확장, 2024년 Papalia의 CORA 범위 측정 확장, Yang-Carlone의 TEASER 계열이 실제 뒤따랐다. 그러나 "visual SLAM의 perspective projection까지 SE-Sync가 덮는다"는 가장 야심찬 확장은 2026년에도 오지 않았다. Projection이 rational function이라 polynomial optimization으로 편입되기 어렵다는 구조적 장벽이 드러났다. `[기술변화]`

> 🔗 **차용.** SE-Sync의 심장에 있는 Burer-Monteiro factorization은 [Burer & Monteiro (2003)](https://link.springer.com/article/10.1007/s10107-002-0352-8)의 low-rank SDP 해법이다. 그 위에 [Boumal-Voroninski-Bandeira (2016)](https://arxiv.org/abs/1605.08101)이 Riemannian 언어로 2차 임계점의 전역성을 보였고, Rosen이 SLAM 맥락에 가져왔다. 순수 수학에서 로봇 백엔드까지 16년이다.

---

## 6b.4 Graph Laplacian과 Fisher Information의 뜻밖의 등가

§6.2는 다른 질문을 던진다. 전역 최솟값이라고 해서 그 추정이 참값과 얼마나 가까운가? 답은 Cramér-Rao Lower Bound와 Fisher Information Matrix다. 회전을 고정한 단순 PGO 모델에서 Rosen-Khosoussi-Barfoot의 결과는 놀랍게도 FIM이 그래프의 weighted reduced Laplacian의 Kronecker product로 정확히 떨어진다는 것이다.

$$\mathcal{I} = \boldsymbol{J}^\top \boldsymbol{\Sigma}^{-1} \boldsymbol{J} = \boldsymbol{L}_w \otimes \boldsymbol{I}_3.$$

그래프 구조만 알면 실제 측정 없이도 추정 정확도의 근사를 얻는다는 뜻이다. Kirchhoff의 Matrix-Tree Theorem에 따라 reduced Laplacian의 determinant는 가중 spanning tree 수와 같고, 이것이 D-optimality(정보 행렬 행렬식)에 대응한다. 알제브라 연결성(Fiedler value)은 E-optimality(최악 분산)에 대응한다. 1847년 Kirchhoff가 전기 회로망을 위해 증명한 정리가 180년 뒤 측정 선택·active SLAM의 이론 기반이 된 셈이다. active SLAM에서 "FIM을 최대화"는 Laplacian 스펙트럼 조작으로 환산된다.

[Kasra Khosoussi와 Timothy Barfoot의 2014년 이후 작업](https://arxiv.org/abs/1709.08601)이 이 연결을 정립했다. Khosoussi는 Sydney에서 Dissanayake·Huang 지도로 박사과정을 밟았고, 이후 MIT와 Toronto를 거쳤다. 3D PGO로 일반화된 형태에서는 Laplacian과 SE(3) adjoint representation의 Kronecker 결합이 등장해 위상·기하 정보를 분리해 다루게 한다. "측정 선택 기준"을 FIM 전체 대신 6배 작은 Laplacian으로 근사 가능하다는 것이 Ch.6이 자리만 두고 지나간 "루프 클로저 선택"의 수학적 근거가 된다.

Ch.4 §4.8이 짚은 EKF-SLAM의 consistency 문제도 이와 맞닿는다. Julier-Uhlmann이 2001년 지적한 EKF의 over-confidence는 CRLB로 재해석하면 근사 선형화가 Fisher information을 과대 추정한다는 말이다. Handbook §6.2가 FIM 챕터를 convex relaxation 옆에 붙여둔 까닭이다. 전역 최솟값과 그 정확도는 쌍으로 다뤄야 한다.

> 🔗 **차용.** [Kirchhoff의 Matrix-Tree Theorem(1847)](https://en.wikipedia.org/wiki/Kirchhoff%27s_theorem)은 전기 회로망 분석 도구로 태어나 조합론을 거쳐 측정 설계 문헌으로 이식됐고, 2010년대 Khosoussi를 통해 SLAM active perception의 언어가 됐다. 한 정리의 180년 이주 경로다.

---

## 6b.5 확장과 한계 — TEASER, CORA, 그리고 Lasserre의 벽

SE-Sync가 나온 뒤 전선은 두 방향으로 넓어졌다. 첫째, 아웃라이어에 강건한 certifiable estimator. 둘째, range·landmark·anisotropic noise 같은 확장된 측정 모델.

아웃라이어 쪽이 먼저 압박이었다. Ch.6 §6.7이 짚었듯 루프 클로저 검증이 완벽하지 않으면 오매칭이 섞이고, Huber·Cauchy 커널로도 일정 비율 이상의 아웃라이어 앞에서는 최적화가 무너진다. 2017년 무렵 certifiable 계보가 이에 답해야 한다는 압박이 분명해졌다.

대표는 [Yang, Shi, Carlone의 TEASER (TRO 2020)](https://arxiv.org/abs/2001.07715)다. 3D 점군 등록에서 99% 아웃라이어에서도 전역 최적 해를 찾는다. truncated least squares 비용을 GNC 래퍼에서 풀되 회전 부분 문제에 SDP relaxation을 붙여 certificate를 함께 낸다. 비결은 스케일·translation·rotation을 각각 certifiable subproblem으로 쪼개 단계마다 전역 최적 보장과 함께 넘기는 데 있었다. 이어진 [Yang & Carlone (2022)](https://arxiv.org/abs/2109.03349)는 이를 Lasserre moment relaxation으로 일반화해 "certifiably robust estimation"이라 명명했다.

Range-aided SLAM은 [Papalia et al. CORA (2024)](https://arxiv.org/abs/2403.09295)의 자리다. 범위 측정 $(\|\boldsymbol{t}_j - \boldsymbol{t}_i\| - \tilde r_{ij})^2$는 그대로면 quartic이라 QCQP에서 벗어나는데, Papalia는 보조 단위벡터 $\boldsymbol{b}_{ij} \in S^{d-1}$로 bearing lifting해 다시 집어넣었다. CORA는 단일 로봇에서는 tight한 relaxation이 멀티로봇에서는 일반적으로 exact하지 않음을 보여 "언제 Shor가 통하는가"의 범위를 좁혔다.

Landmark 쪽에서는 [Holmes & Barfoot (2023)](https://arxiv.org/abs/2308.05631)이 Schur complement로 landmark를 미리 소거해 SE-Sync가 그대로 받아먹는 형태로 만들었다. Holmes·Khosoussi·Rosen이 Handbook Ch.6을 공저한 것은 이 계보가 2025년 한 테이블에 모였다는 증거다.

그러나 벽도 드러났다. anisotropic noise와 truncated-quadratic outlier를 POP(Polynomial Optimization Problem)로 일반화하면 Lasserre moment relaxation이 필요한데, 유도된 SDP가 **degenerate**해 constraint qualification이 실패하고 Riemannian Staircase의 수렴 조건이 깨진다. Yang의 2022년 sparse monomial basis 같은 우회가 있지만 전용 solver는 일반 local solver보다 느리다. 속도와 증명 가능성을 동시에 쥐는 알고리즘은 아직 없다. Visual SLAM·VIO는 더 깊은 장벽 — perspective projection·IMU preintegration의 구조적 비호환 — 앞에 있는데 🧭에서 다룬다.

> 📜 **예언 vs 실제.** Carlone이 2015년 ICRA에서 "Lagrangian dual이 tight한 인스턴스가 왜 대부분인지 이론적 해명이 필요하다"고 적었다. 10년이 지났고, 답은 부분적으로만 나왔다. Rosen-Carlone-Bandeira-Leonard의 exact recovery 정리가 "노이즈가 $\beta$ 이하"라는 충분조건을 주었지만, 실제 SLAM 인스턴스에서 $\beta$를 사전에 계산하는 방법은 없다. tightness가 언제 깨지는지에 대한 **사전**(a priori) 조건은 2026년 기준 여전히 per-instance certificate로 대체되어 있다. `[진행형]`

---

## 🧭 아직 열린 것

**Tightness가 깨지는 경계.** SE-Sync의 exact recovery 정리는 "노이즈 $\beta$ 이하"라는 충분조건만 주고, 실제 인스턴스에서 $\beta$ 계산법은 없다. 사전에 tight 여부를 판정할 수 있어야 알고리즘 설계가 진전된다. 무거운 아웃라이어나 극단적 희소 그래프에서 relaxation의 실패 양상에 대한 계통적 연구는 아직 초기 단계다.

**Visual SLAM·VIO와의 통합.** perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$는 rational function이라 polynomial optimization에 그대로 들어오지 않는다. 분모를 곱해 polynomial로 바꿔도 feature마다 새 변수·보조 제약이 추가되어 ORB-SLAM3의 수천 맵포인트에서 SDP 크기는 실시간 영역 밖이다. Forster의 2015년 IMU preintegration도 exponential map과 bias drift가 얽혀 POP 편입이 어렵다. 2026년 기준 Ch.7·Ch.8·Ch.13의 visual/VIO 주류는 certifiable guarantee 바깥에 있다. 계보가 풀지 못한 가장 큰 자리다.

**Online certification과 스케일.** SE-Sync는 배치다. 새 측정마다 SDP를 다시 풀어 certificate를 갱신하는 증분 certifiable SLAM은 아직 성숙하지 않았다. iSAM2가 배치 SAM에 풀어낸 증분화를 certifiable 쪽에서 반복해야 하는 셈이다. warm-start, rank 증분, 부분 certificate 합성 모두 열린 연구고, 도시 규모 그래프에서 moment relaxation solver의 속도도 여전히 문제다.

**Outlier-majority.** 현재의 certifiable robust estimator는 "소수 아웃라이어" 가정 위에 선다. 다수가 오염된 상황에서는 list-decodable regression 같은 다중 가설 certification이 필요하나 통계학 쪽에서도 시작 단계다. 2024년 Cheng·Shi·Carlone의 후속 작업이 있었지만 TEASER 같은 표준 도구는 없다.

---

이 챕터 전체가 Ch.6이 한 줄로 지나간 "지역 최솟값 수렴"(§6.7)에 대한 각주다. 민속적 관찰은 10년의 이론 프로그램으로 대체됐다. Carlone-Khosoussi-Rosen-Holmes-Barfoot-Dissanayake가 공저한 *The SLAM Handbook* Ch.6이 34페이지로 이 주제를 다룬 것 자체가 계보의 현재 무게다. 같은 10년 동안 Ch.12·Ch.13·Ch.16의 학습 기반 SLAM은 다른 경로로 나아갔다. 한쪽은 해의 전역성을 증명하는 쪽, 다른 쪽은 신경망이 해를 직접 예측하는 쪽. 두 계보가 만날지, 분야를 둘로 나누어 지낼지는 2026년에도 답이 없다. Ch.19에서 이 챕터의 🧭 항목들이 "백엔드 이론의 공란"으로 수확된다.
