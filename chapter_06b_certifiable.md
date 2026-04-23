# Ch.6b — Certifiable SLAM: 지역 최솟값을 넘어서

Ch.6이 기록한 Lu-Milios에서 g2o·GTSAM에 이르는 계보는 한 가지를 남겨두었다. 포즈 그래프 최적화는 비볼록 문제다. Gauss-Newton과 Levenberg-Marquardt가 내놓는 해는 지역 최솟값일 수 있다. 초기값이 나쁘거나 루프 클로저가 어긋나면 수렴한 값이 진짜 최솟값인지 아무도 알 수 없다. 실무자들은 "odometry 초기값이 있으면 대체로 잘 풀린다"는 민속적 관찰로 지내왔다. 어떤 데이터셋에서는 이 관찰이 들어맞았고, 어떤 현장에서는 백엔드가 엉뚱한 지점에서 수렴했지만 경고음은 울리지 않았다. 2015년 MIT의 Luca Carlone이 그 민속을 수학으로 대체하기 시작했다. 스캔 얼라인먼트의 전역 일관성을 최초로 확보한 Lu-Milios의 해법 (Ch.6 §6.1)이 여전히 지역해일 수 있다는 역설, 이 간극을 메우는 일이 이어지는 10년의 과제가 된다. 2015년 Carlone의 Lagrangian duality 시도에서 2019년 David Rosen의 SE-Sync로, 그 뒤 Briales-Gonzalez-Jimenez의 Cartan-Sync, Yang-Carlone의 TEASER, Papalia의 CORA까지 — 이 계보는 SLAM 백엔드를 "경험적으로 잘 풀리는 비볼록 최적화"에서 "전역 최적성이 증명 가능한 convex surrogate"로 다시 쓴다. 그 재작성의 도구 상자에 담긴 연장들은 모두 SLAM 바깥에서 왔다. 오퍼레이션스 리서치의 Shor relaxation, 수학 최적화의 Burer-Monteiro factorization, 미분기하의 Riemannian optimization, 그래프 이론의 Kirchhoff Matrix-Tree. 10년간 이것들을 한 테이블 위에 모은 사람들의 이름이 이 챕터의 본문이다.

---

## 6b.1 지역 최솟값이라는 오래된 불안

Ch.6 §6.7은 그래프 SLAM 백엔드의 첫 문제로 초기값 의존성을 꼽았다. 포즈 그래프 최적화의 비용 함수는 회전 변수 $\boldsymbol{R}_i \in \mathrm{SO}(3)$ 위에서 정의되는 비볼록 함수이기 때문에, 초기 추정이 참값에서 멀면 Gauss-Newton은 엉뚱한 분지로 빨려 들어간다. Handbook 6.1 절의 parking garage 데이터셋 예시는 이 증상을 극명하게 보여준다. 같은 입력에서 무작위 초기화를 네 번 하면, 그중 하나는 SE-Sync가 도달한 전역 최솟값에 붙지만 나머지 셋은 뒤틀린 지역 최솟값에 안착한다. 육안으로 봐도 주차장 바닥이 접혀 있다.

2000년대 후반까지 커뮤니티의 대응은 두 갈래였다. 하나는 odometry를 신뢰해서 초기값 품질을 확보하는 쪽. 또 하나는 루프 클로저 검증과 아웃라이어 제거를 전단에서 철저히 하는 쪽. 둘 다 유효했지만, 수렴한 값이 진짜 최솟값인지 판정하는 도구는 아니었다. Huang과 Dissanayake가 2010년 무렵 짚은 문제는 단순했다. 초기값이 아무리 좋아도 데이터 자체가 모호하면 최적화기는 틀린 답에 가서 멈출 수 있다. PGO가 NP-hard라는 것도 그 무렵 정식화됐다. 그런데도 현장에서는 g2o가 대체로 잘 풀렸다. 이 간극 — 이론은 최악을 말하는데 실무는 평균을 보는 간극 — 이 2010년대 중반 백엔드 이론 연구자들이 파고든 자리다.

지역 최솟값 문제가 걸러지지 못하면 어떤 일이 일어나는지는 Handbook Ch.6이 garage 데이터셋의 네 컷 시각화로 명시한다. 같은 PGO 인스턴스에 무작위 초기값을 준 네 번의 실행 중 하나만 바닥이 평평한 주차장 구조를 복원하고, 나머지 셋은 건물 한 면이 옆으로 접혀 있거나, 층 전체가 비틀어져 있거나, 루프가 제자리로 닫히지 않은 채 멈춰 있다. Gauss-Newton이 멈춘 모든 지점은 *국소적으로는* 최적이다. 비용 함수의 기울기가 0이고 헤시안도 양정치다. 그런데 답은 전혀 다르다. 백엔드가 "수렴했다"고 신호를 보내는 순간은 실패가 가장 눈에 띄지 않는 순간이기도 하다. 이 시각화가 Rosen-Carlone의 2019년 논문 서두에 놓인 이유는, 이것이 이론을 위한 문제 제기가 아니라 10년 넘은 현장 불안의 그림 버전이기 때문이다.

> 🔗 **차용.** Ch.6이 다룬 robust kernel(Huber, Cauchy) 계보와 이 챕터의 GNC(Graduated Non-Convexity)는 동일한 뿌리를 공유한다. [Black & Rangarajan (1996)](https://cs.brown.edu/people/mjblack/Papers/ijcv1996.pdf)의 robust statistics와 비용 함수의 이중성 정리가 그 뿌리다. 한쪽에서는 비용 가중을 바꿔 아웃라이어 영향을 줄였고, 반대쪽에서는 같은 원리를 비볼록성 회피에 전용했다.

---

## 6b.2 Shor relaxation — 바깥에서 들어온 무기

PGO의 비볼록성은 회전 제약 $\boldsymbol{R}_i \in \mathrm{SO}(d)$에서 온다. 이 제약은 사실 직교성 조건 $\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}$과 $\det(\boldsymbol{R}) = +1$로, 이차 방정식으로 쓸 수 있다. 목적함수도 이차다. 결국 PGO는 **QCQP**(Quadratically Constrained Quadratic Program)로 정확하게 떨어진다. 그리고 QCQP에는 1987년 이후 오퍼레이션스 리서치 분야에서 검증된 convex relaxation 도구가 있었다. [Naum Shor의 1987 relaxation](https://link.springer.com/article/10.1007/BF01582220)이다.

Shor의 아이디어는 두 줄로 요약된다. $\boldsymbol{x}^\top \boldsymbol{M}\boldsymbol{x} = \mathrm{tr}(\boldsymbol{M}\boldsymbol{x}\boldsymbol{x}^\top)$이라는 항등식으로 목적과 제약을 모두 리프팅 변수 $\boldsymbol{X} \triangleq \boldsymbol{x}\boldsymbol{x}^\top$에 대한 선형 함수로 바꾸면 원 QCQP는 "$\boldsymbol{X} \succeq 0$이면서 rank-1"이라는 제약 위의 선형 목적 문제가 된다. 여기서 rank-1 제약 하나만 버리면 남는 것은 볼록한 **SDP**다. 탐색 공간이 $n$차원에서 $n(n+1)/2$로 늘어나는 대가로 볼록성을 얻는 거래다.

$$d^* = \min_{\boldsymbol{X}\in\mathbb{S}^n} \mathrm{tr}(\boldsymbol{C}\boldsymbol{X}) \;\; \text{s.t.} \;\; \mathrm{tr}(\boldsymbol{A}_i\boldsymbol{X})=b_i,\; \boldsymbol{X}\succeq 0.$$

쓸모는 이중성 부등식 $d^* \le p^*$에 있다. SDP 최솟값은 원 QCQP 최솟값의 아래쪽 경계를 준다. feasible set을 넓혔으니 답은 원래보다 더 작거나 같다는 당연한 사실이지만, 이 당연함이 증명 도구가 된다. 어떤 후보해 $\hat{\boldsymbol{x}}$가 있을 때 $f(\hat{\boldsymbol{x}}) - d^*$가 바로 그 후보의 최적성에서 얼마나 떨어져 있는지를 위에서 눌러주는 bound다. 여기서 "certifiable"이라는 이름이 나온다. 원 문제를 전역적으로 못 풀어도, 가진 해가 얼마나 나쁜지의 상한은 풀 수 있다는 뜻이다. SDP 해 $\boldsymbol{X}^*$가 우연히 rank-1로 떨어지면 $\boldsymbol{X}^* = \boldsymbol{x}^*\boldsymbol{x}^{*\top}$에서 $\boldsymbol{x}^*$가 바로 원 QCQP의 전역 최솟값이다. 이 "favorable situation"이 SLAM 문제에서 얼마나 자주 일어나는지, 그리고 그 조건이 무엇인지가 이후 논문들의 주제가 된다.

Carlone이 2015년 IROS와 ICRA에서 발표한 두 편의 논문 — [Carlone et al. 2015 "Lagrangian duality in 3D SLAM"](https://arxiv.org/abs/1506.00746)과 [Carlone & Dellaert 2015 "Planar pose graph optimization"](https://doi.org/10.1109/ICRA.2015.7139264) — 이 계보의 출발점이다. 2D PGO에서 Lagrangian dual이 tight하다는 것, 즉 duality gap이 대개 0임을 경험적으로 보였고, 3D PGO에도 같은 틀이 확장 가능함을 시사했다. Carlone은 그 시점에 이미 그래프 SLAM의 실무적 한계를 알고 있었다. 그는 Ch.6의 g2o·GTSAM 커뮤니티에서 쓰이는 초기화 기법을 정리한 2014년 TRO 서베이를 갓 끝낸 참이었고, odometry 초기값이 루프 클로저와 충돌할 때 최적화가 얼마나 자주 틀린 지점에서 멈추는지를 본 뒤였다. 그 경험 위에서 쓰인 2015년 논문의 결론은 조심스러웠다. "duality gap이 보통 0이라는 경험적 사실"을 보고할 뿐, 그것이 언제 성립하는지에 대한 닫힌 조건은 주지 못했다.

같은 시기 [Briales & Gonzalez-Jimenez (2017)](https://arxiv.org/abs/1702.03235)의 Cartan-Sync가 SO(3) 위의 synchronization으로 이 프로그램을 밀었다. 말라가 대학의 두 사람은 Carlone과 독립적으로 3D PGO를 convex relaxation으로 다루는 시도를 했고, Riemannian 매니폴드 위에서 직접 Cartan connection을 이용하는 변형을 내놓았다. 수학 쪽에서는 독립적으로 Boumal·Absil·Sepulchre가 Riemannian optimization을 다듬고 있었고, 최적화 쪽에서는 Burer와 Monteiro의 low-rank SDP factorization이 2003년부터 자리잡고 있었다. 흩어져 있던 재료들이 2019년 한 편의 논문에서 조립된다.

---

## 6b.3 SE-Sync — Rosen 2019가 조립한 것

[Rosen, Carlone, Bandeira, Leonard의 SE-Sync (IJRR 2019)](https://arxiv.org/abs/1612.07386)는 certifiable SLAM의 캐논이다. MIT에서 John Leonard의 박사과정을 마친 David Rosen은 2016년 WAFR에서 SE-Sync 초기판을 발표했고, 3년 뒤 IJRR 정식판이 나왔다. Leonard는 Ch.4의 Durrant-Whyte와 함께 1990년대 초 "SLAM"이라는 이름이 자리 잡는 데 기여한 MIT 연구자였고, Rosen은 그 랩의 학생이었다. 공저자 Afonso Bandeira는 SDP와 synchronization 문제의 수학 쪽 전문가로, rank-deficient 2차 임계점의 전역성을 증명하는 이론 부분을 맡았다. 네 공저자의 배경이 서로 다른 계보 — 로보틱스, SLAM, 수학 최적화, 응용수학 — 인 것 자체가 이 논문이 무엇을 조립했는지를 말해준다. 이 논문이 한 일은 발명이 아니라 조립이었다. Shor relaxation, translation elimination, Burer-Monteiro low-rank parameterization, Boumal의 Riemannian staircase — 각각 다른 계보에서 10여 년씩 숙성된 재료들을 PGO라는 한 문제 위에서 맞물리게 한 것이다.

조립의 순서는 세 단계다. 첫째, 회전 고정 시 translation이 선형 최소자승이 된다는 관찰에서 $\boldsymbol{t}$를 닫힌 형태로 소거한다(Problem 6.2). 이 변형은 우아한데, translation이 선형 부분이라는 사실은 사실 Ch.6의 graph SLAM 계보가 오래전부터 알고 있던 것이었다. Carlone이 2014년 TRO에서 초기화 서베이를 하면서 명시적으로 지적했고, Rosen이 이 관찰을 convex relaxation의 첫 단계로 집어넣었다. 둘째, 남은 rotation-only 문제 $\min_{\boldsymbol{R}\in\mathrm{SO}(d)^n} \mathrm{tr}(\tilde{\boldsymbol{Q}}\boldsymbol{R}^\top\boldsymbol{R})$에 Shor relaxation을 적용해 SDP로 리프팅한다(Problem 6.3). 셋째, $dn \times dn$ 차원의 SDP는 그대로 풀면 interior-point method가 수천 포즈에서 무너지므로 Burer-Monteiro 재파라미터화 $\boldsymbol{Z} = \boldsymbol{Y}^\top \boldsymbol{Y}$로 Stiefel manifold 위의 저차원 비제약 문제로 바꾼다(Problem 6.4).

핵심 결과 두 개가 이 조립을 정당화한다. Theorem 6.1은 **exact recovery** 정리다. 측정 노이즈가 어떤 상수 $\beta$보다 작으면 SDP relaxation의 유일 해가 정확히 원 MLE의 전역 최솟값을 rank-1로 품고 있다는 진술이다. "어떤 노이즈까지 버티는가"에 대한 첫 정량적 답이었다. 다만 $\beta$는 ground-truth 행렬에 의존하기 때문에 인스턴스를 보기 전에는 모른다. 이론은 "충분히 작은 노이즈에서 통한다"는 것을 보장하지만 "이번 데이터에서 작은지"는 판정하지 않는다. Theorem 6.2는 Boumal et al.의 결과를 가져온 것으로, Stiefel manifold 위에서 찾은 2차 임계점이 rank-deficient하면 그것이 곧 전역 최솟값이라고 보장한다. 이 두 정리가 Riemannian Staircase 알고리즘을 가능하게 한다. rank를 작게 두고 시작해서, local solver로 2차 임계점을 찾고, rank-deficiency를 검사한다. 안 맞으면 rank를 하나 올려 계단을 오른다. 논문 §6.1에서 Rosen은 "실무 데이터셋에서 보통 한 계단이면 끝난다"고 적었다. 유한한 계단 수 안에 반드시 멈춘다는 보장도 따라온다. rank가 $dn + 1$에 도달하면 모든 $\boldsymbol{Y}$가 row rank-deficient가 되기 때문이다.

실제 숫자가 따라왔다. sphere, torus, garage 같은 표준 벤치마크에서 SE-Sync는 g2o·GTSAM 수준의 속도로 수렴하면서 a posteriori certificate를 함께 내놓았다. 후보해에 $F(\tilde{\boldsymbol{Q}}\hat{\boldsymbol{R}}^\top\hat{\boldsymbol{R}}) = p^*_{\mathrm{SDP}}$가 만족되면 그 해는 수학적으로 전역 최솟값임이 증명된 것이다. Lu-Milios 이후 20년 만에 백엔드 연구자들이 "이 해가 진짜 최솟값인가"에 대해 '예/아니오'를 찍을 수 있게 됐다. g2o가 "어떤 문제든 플러그인으로" 라는 범용성의 깃발을 들었다면, SE-Sync는 "해의 품질을 스스로 증언한다"는 다른 깃발을 들었다.

SE-Sync의 쓸모는 certificate 자체에 있지 단순한 속도에 있지 않다. g2o·GTSAM은 대개 빠르다. 문제는 언제 그 답을 믿어도 되는지였다. Rosen의 알고리즘은 끝에 숫자 하나를 더 토해낸다. "당신의 해가 전역 최솟값으로부터 최대 이만큼 떨어져 있다"라는 suboptimality bound. 이 bound가 0이면 해는 증명 가능하게 전역 최적이다. 2019년 이전 SLAM 백엔드는 이 질문에 침묵했다. 실무 엔지니어가 "이 지도가 정말 맞나"를 확인하려면 다른 초기값에서 다시 돌려보거나, 매 루프 클로저를 수작업으로 검증하는 수밖에 없었다. SE-Sync 이후에는 알고리즘이 스스로 증언을 남긴다.

> 📜 **예언 vs 실제.** Rosen은 IJRR 2019 논문 §8.2에서 "우리가 보인 algebraic simplification은 anisotropic noise·outlier·다양한 센서 모달리티로 확장될 수 있을 것"이라 적었다. 그 예언은 부분적으로 적중했다. 2023년 Holmes-Barfoot의 landmark-SLAM 확장, 2024년 Papalia의 CORA 범위 측정 확장, Yang-Carlone의 TEASER 계열이 실제 뒤따랐다. 그러나 "visual SLAM의 perspective projection까지 SE-Sync가 덮는다"는 가장 야심찬 확장은 2026년에도 오지 않았다. Projection이 rational function이라 polynomial optimization으로 편입되기 어렵다는 구조적 장벽이 드러났다. `[기술변화]`

> 🔗 **차용.** SE-Sync의 심장에 있는 Burer-Monteiro factorization은 [Burer & Monteiro (2003)](https://link.springer.com/article/10.1007/s10107-002-0352-8)가 수학 최적화에서 제안한 low-rank SDP 해법이다. 볼록성을 잃는 대신 차원을 급감시키는 tradeoff를 맞바꿨다. 그 위에 [Boumal-Voroninski-Bandeira (2016)](https://arxiv.org/abs/1605.08101)가 Riemannian 최적화 언어로 2차 임계점의 전역성을 보였고, Rosen이 SLAM 맥락에 가져왔다. 순수 수학에서 로봇 백엔드까지 16년이 걸렸다.

---

## 6b.4 Graph Laplacian과 Fisher Information의 뜻밖의 등가

§6.2는 다른 질문을 던진다. 전역 최솟값을 찾았다고 해서 그 추정값이 참값과 얼마나 가까운가? 답은 Cramér-Rao Lower Bound와 Fisher Information Matrix다. 회전을 알려진 것으로 고정한 단순 PGO 모델에서 Rosen-Khosoussi-Barfoot가 보인 결과는 놀랍다. FIM이 그래프의 weighted reduced Laplacian의 Kronecker product로 정확히 떨어진다.

$$\mathcal{I} = \boldsymbol{J}^\top \boldsymbol{\Sigma}^{-1} \boldsymbol{J} = \boldsymbol{L}_w \otimes \boldsymbol{I}_3.$$

이 등식이 의미하는 바는 교과서 같은 얇은 통찰이 아니다. 그래프의 구조만 알면 실제 측정 없이도 추정 정확도의 근사를 얻는다는 것이고, 그래프 이론에서 오래 쓰인 불변량이 SLAM 디자인 지표로 그대로 건너온다는 것이다. Kirchhoff의 Matrix-Tree Theorem에 따라 reduced Laplacian의 determinant는 가중 spanning tree 수와 같다. 이것이 D-optimality(정보 행렬 행렬식)에 대응한다. 알제브라 연결성(Fiedler value)은 E-optimality(최악 분산)에 대응한다. 1847년 Kirchhoff가 전기 회로망을 위해 증명한 정리가 180년 뒤 측정 선택·active SLAM의 이론 기반이 된 셈이다. 이 연결이 제공하는 것은 단순한 비유 이상이다. active SLAM에서 로봇이 다음에 어느 방향으로 움직일지, 어느 루프 클로저를 받아들일지, 어느 측정을 marginalize할지를 결정할 때 "FIM을 최대화"라는 기준을 Laplacian의 스펙트럼 조작으로 환산할 수 있다는 구체적 도구가 된다.

[Kasra Khosoussi와 Timothy Barfoot의 2014년 이후 작업](https://arxiv.org/abs/1709.08601)이 이 연결을 정립했다. Khosoussi는 Sydney에서 Dissanayake·Huang 지도로 박사과정을 밟았고, 이후 MIT와 Toronto를 거쳤다. 그의 10년짜리 프로그램은 한 문장으로 요약된다. "그래프 구조만 보고도 SLAM 추정의 품질을 예측할 수 있다." 3D PGO로 일반화된 형태에서는 Laplacian과 SE(3) adjoint representation의 Kronecker 결합이 등장하는데, 이 역시 그래프의 위상 정보와 기하 정보를 분리해 다룰 수 있게 한다. 현실적 함의는 크다. 현업 시스템이 유지해야 할 "측정 선택 기준"이 FIM의 full 행렬을 다루는 대신 6배 더 작은 Laplacian으로 근사 가능하다. 이것이 Ch.6에서 그 자리에만 두고 지나간 "루프 클로저 선택"의 수학적 근거가 된다.

Ch.4 §4.8이 짚은 EKF-SLAM의 consistency 문제도 이 결과와 맞닿아 있다. Julier-Uhlmann이 2001년에 지적한 EKF의 over-confidence 현상은 CRLB 관점에서 재해석하면 근사 선형화가 Fisher information을 과대 추정한다는 말과 같다. 백엔드가 바뀌어도 CRLB 자체는 옮겨 다닌다. Handbook §6.2가 FIM 챕터를 convex relaxation 챕터 옆에 붙여둔 까닭이다. "전역 최솟값을 찾는 문제"와 "그 최솟값이 얼마나 정확한가"는 쌍으로 다뤄야 한다.

> 🔗 **차용.** [Kirchhoff의 Matrix-Tree Theorem(1847)](https://en.wikipedia.org/wiki/Kirchhoff%27s_theorem)은 전기 회로망의 분석 도구로 태어났다. 이산수학과 조합론으로 재해석된 뒤 측정 설계 문헌으로 이식됐고, 2010년대 Khosoussi를 통해 SLAM active perception의 언어가 됐다. 한 정리의 180년 이주 경로다.

---

## 6b.5 확장과 한계 — TEASER, CORA, 그리고 Lasserre의 벽

SE-Sync가 나온 뒤 전선은 두 방향으로 넓어졌다. 첫째, 아웃라이어에 강건한 certifiable estimator. 둘째, range·landmark·anisotropic noise 같은 확장된 측정 모델.

아웃라이어 쪽이 먼저 압박이었다. Ch.6 §6.7이 짚었듯 실무의 포즈 그래프는 루프 클로저 검증이 완벽하지 않으면 오매칭이 섞이기 마련이고, Huber나 Cauchy 커널로 완화해도 일정 비율 이상의 아웃라이어 앞에서는 최적화가 무너진다. certifiable 계보가 아웃라이어에 대해 뭔가를 말해야 한다는 압박이 2017년 무렵 분명해졌다.

아웃라이어 쪽의 대표는 [Yang, Shi, Carlone의 TEASER (TRO 2020)](https://arxiv.org/abs/2001.07715)다. 3D 점군 등록에서 99%까지 아웃라이어가 섞여도 전역 최적 해를 찾아내는 알고리즘으로, truncated least squares 비용을 GNC(Graduated Non-Convexity) 래퍼 안에서 풀되 회전 부분 문제에 SDP relaxation을 붙여 certificate를 함께 내놓는다. Yang은 MIT에서 Carlone 지도로 박사과정을 밟았고, TEASER++는 오픈소스 C++·Python 구현으로 풀려 3D 스캔 정합 파이프라인 여러 곳에 들어갔다. TEASER의 수치는 당시 기준으로 믿기 어려웠다. RANSAC이 50% 아웃라이어에서 휘청이던 시점에 99%에서도 정답을 찍는다는 주장이다. 비결은 문제를 작은 부분으로 쪼개는 데 있었다. 스케일·translation·rotation을 순차로 푸는 대신 각 부분을 certifiable subproblem으로 다루고, 각 단계에서 전역 최적 보장과 함께 넘긴다. 이어진 [Yang & Carlone (2022)](https://arxiv.org/abs/2109.03349)는 이 접근을 Lasserre moment relaxation의 언어로 일반화해 "certifiably robust estimation"이라는 이름을 붙였다.

Range-aided SLAM은 [Papalia et al. CORA (2024)](https://arxiv.org/abs/2403.09295)의 자리다. 범위 측정 $(\|\boldsymbol{t}_j - \boldsymbol{t}_i\| - \tilde r_{ij})^2$는 그대로 쓰면 quartic이 되어 QCQP에서 벗어난다. Papalia는 보조 단위벡터 $\boldsymbol{b}_{ij} \in S^{d-1}$를 도입해 bearing lifting으로 QCQP 안에 다시 집어넣었다. 도입한 변수 하나당 $\|\boldsymbol{b}\|^2 = 1$이라는 이차 제약을 감수하는 대가로 전체가 certifiable framework에 들어온다. UWB·멀티로봇 셋업에서 Shor relaxation의 tightness 조건이 무엇인지가 CORA가 연 새 질문이다. 단일 로봇에서는 tight하지만, 서로의 상대 포즈 없이 범위만 교환하는 멀티로봇 상황에서는 일반적으로 relaxation이 exact하지 않다는 것을 CORA가 보였다. 이 부분 결과만으로도 "언제 Shor가 통하는가"라는 근본 질문의 범위가 좁혀진 셈이다.

Landmark SLAM 쪽에서는 [Connor Holmes와 Tim Barfoot (2023)](https://arxiv.org/abs/2308.05631)이 Toronto에서 Schur complement 기법으로 landmark 수에 대한 선형 스케일링을 복원했다. landmark를 단순 translation 변수로 취급한 뒤 Schur complement로 미리 소거하면, 남은 PGO가 SE-Sync가 그대로 받아먹는 형태가 된다. Holmes·Khosoussi·Rosen이 공동 저자로 Handbook Ch.6을 집필한 것도 이 계보의 현 세대가 자리를 공유하고 있다는 증거다. MIT, NEU, Toronto, Sydney 네 곳의 연구자들이 2015년 Carlone의 첫 시도 이후 한 계보를 이뤄왔고, 2025년 Handbook은 그 10년의 중간 정산이다.

그러나 벽도 드러났다. anisotropic noise와 truncated-quadratic outlier 모델을 POP(Polynomial Optimization Problem)로 일반화하면 Lasserre의 moment relaxation 계층이 필요해진다. 문제는 moment relaxation에서 유도되는 SDP가 **degenerate**하다는 것이다. constraint qualification이 실패하고 Riemannian Staircase의 수렴 조건이 깨진다. Yang이 2022년 제안한 sparse monomial basis 같은 우회가 있지만, 전용 solver는 일반 local solver보다 여전히 느리다. "certifiable"이라는 단어가 붙은 모든 기법이 실시간 SLAM에 곧바로 들어갈 수는 없다. SDP는 여전히 무겁고, moment hierarchy는 차수가 오를수록 기하급수적으로 커진다. 한 팀은 속도를, 다른 팀은 증명 가능성을 얻지만 둘을 동시에 쥐는 알고리즘은 아직 나오지 않았다.

Visual SLAM·VIO 쪽은 더 깊은 장벽 앞에 있다. perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$는 rational function이라 그대로는 polynomial optimization에 들어오지 않는다. 분모를 곱해서 polynomial로 바꿀 수는 있지만, feature 수만큼 새 변수가 생기고 각 feature마다 보조 제약이 추가된다. ORB-SLAM3이 다루는 수천 개 맵포인트를 생각하면 SDP 크기는 실시간 영역 밖이다. IMU preintegration도 마찬가지다. Forster가 2015년 정식화한 preintegration 모델은 exponential map과 bias drift가 얽혀 있어 polynomial 편입이 쉽지 않다. 결과적으로 Ch.7·Ch.8·Ch.13이 다룬 visual/VIO 주류 시스템은 certifiable framework 바깥에서 작동한다. 이 간극이 계보 전체의 현재 경계선이다.

> 📜 **예언 vs 실제.** Carlone이 2015년 ICRA에서 "Lagrangian dual이 tight한 인스턴스가 왜 대부분인지 이론적 해명이 필요하다"고 적었다. 10년이 지났고, 답은 부분적으로만 나왔다. Rosen-Carlone-Bandeira-Leonard의 exact recovery 정리가 "노이즈가 $\beta$ 이하"라는 충분조건을 주었지만, 실제 SLAM 인스턴스에서 $\beta$를 사전에 계산하는 방법은 없다. tightness가 언제 깨지는지에 대한 **사전**(a priori) 조건은 2026년 기준 여전히 per-instance certificate로 대체되어 있다. `[진행형]`

---

## 🧭 아직 열린 것

**Tightness가 깨지는 경계.** SE-Sync의 exact recovery 정리는 "노이즈가 $\beta$ 이하"라는 충분조건을 제시했지만, 실제 인스턴스에서 $\beta$를 계산하는 방법이 없다. a posteriori certificate가 있으니 실무에서는 풀고 나서 확인하면 된다. 그러나 언제 tight할지 사전에 판정할 수 있어야 알고리즘 설계가 진전된다. 무거운 아웃라이어가 섞이거나 그래프가 극단적으로 희소할 때 relaxation이 어떻게 실패하는지에 대한 계통적 연구는 아직 초기 단계다. 2022-2024년의 일부 논문이 특수한 그래프 클래스에서 exactness 조건을 유도했지만, 일반 SLAM 그래프에서 적용 가능한 닫힌 형태의 판정 기준은 없다.

**Visual SLAM·VIO와의 통합.** perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$는 rational function이지 polynomial이 아니다. 이 rational 구조는 moment relaxation으로 집어넣어도 변수 수가 feature 수에 비례해 폭발하고, 실시간 처리 수준의 SDP 크기로는 들어오지 않는다. IMU preintegration 모델도 biased noise와 exponential map이 얽혀 POP 편입이 어렵다. 결과적으로 2026년 기준 ORB-SLAM3·VINS-Fusion 같은 주류 visual/VIO 백엔드는 여전히 local solver 기반이며 certifiable guarantee 바깥에 있다. 이 간극이 certifiable 계보가 풀지 못한 가장 큰 자리다. Ch.7의 ORB-SLAM 계보와 Ch.8의 direct method가 certifiable framework으로 들어올 수 있는지는 2026년에도 열린 질문이다.

**Online certification과 스케일.** SE-Sync는 배치로 설계됐다. 실시간 온라인 시스템에서 새 측정이 들어올 때마다 SDP를 다시 풀어 certificate를 갱신하는 증분 certifiable SLAM은 아직 성숙하지 않았다. iSAM2가 배치 SAM에 대해 풀어낸 증분화 문제를 certifiable 쪽에서 반복해야 하는 셈이다. 새 factor가 추가될 때 Stiefel manifold 위의 해를 warm-start하는 방법, rank가 증가하는 상황을 증분으로 다루는 방법, 부분 certificate를 합쳐 전체 certificate를 구성하는 방법 — 모두 열린 연구다. 또한 도시 규모의 수만 포즈 그래프에서 moment relaxation 전용 solver의 속도는 여전히 문제다.

**Outlier-majority 상황.** 현재의 certifiable robust estimator들은 대부분 "소수 아웃라이어" 가정 위에 선다. 다수가 오염된 상황에서는 list-decodable regression 같은 다중 가설 certification이 필요한데, 이 방향은 통계학 쪽에서 막 시작된 수준이다. 자율주행 다중 로봇 센서가 동시에 흔들릴 수 있는 현실에서 이 문제의 실용화 거리는 짧지 않다. 2024년 전후로 Cheng·Shi·Carlone의 후속 작업이 이 방향으로 확장을 시도했지만, 아직 TEASER처럼 오픈소스 표준으로 자리잡은 도구는 없다.

---

이 챕터 전체가 Ch.6이 한 줄로 지나간 "지역 최솟값 수렴"(§6.7) 문장의 각주다. 그 민속적 관찰이 10년의 이론 프로그램으로 대체된 흐름이다. 한편 Carlone-Khosoussi-Rosen-Holmes-Barfoot-Dissanayake가 공저한 *The SLAM Handbook* Ch.6이 이 주제를 34페이지에 걸쳐 다룬다는 사실 자체가 이 계보의 현재 무게를 보여준다. 2015년 Carlone이 Lagrangian duality라는 단어를 SLAM 문헌에 처음 가져왔을 때, 그 단어 뒤에 10년의 협업 네트워크가 만들어지고 한 권의 편저 챕터로 묶이는 풍경은 아마 예상하기 어려웠을 것이다. 같은 10년 동안 Ch.12·Ch.13·Ch.16이 기록한 학습 기반 SLAM이 다른 경로로 나아갔다. 한쪽은 "해의 전역성을 증명하는" 방향으로, 다른 쪽은 "해를 신경망이 직접 예측하는" 방향으로. 두 계보가 언젠가 만날지, 아니면 SLAM이라는 분야를 두 개의 반쪽으로 나누어 지낼지는 2026년에도 답이 없다. Ch.19에서 이 챕터의 🧭 항목들이 "백엔드 이론의 공란"으로 수확된다.
