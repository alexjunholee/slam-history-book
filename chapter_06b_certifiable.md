# 7장 전역 최적성 인증

Lu와 Milios에서 g2o, GTSAM으로 이어지며 그래프 SLAM이 천하를 통일한 듯 보였지만, 그 심장부에는 아무도 해결하지 못한 불안한 균열이 하나 남아 있었다. 포즈 그래프 최적화(PGO)는 본질적으로 비볼록(non-convex) 문제이기 때문에, Gauss-Newton이나 Levenberg-Marquardt가 아무리 매끈하게 수렴했더라도 그 답이 진짜 전역 최솟값이 아니라 엉뚱한 지역 최솟값(local minimum)일 위험을 늘 안고 있었다. 실무자들은 대체로 *"오도메트리 초기값만 그럴듯하면 잘 풀린다"*는 경험칙에 기대어 시스템을 돌렸다. 하지만 초기값이 크게 어긋나거나 모호한 루프가 걸린 날이면, backend는 기괴하게 뒤틀린 지도를 내놓고도 실패 신호 하나 없이 태연히 "수렴 완료"를 선언하곤 했다.

2015년, 당시 Georgia Tech에 몸담고 있던 Luca Carlone은 Frank Dellaert, MIT의 David Rosen, John Leonard 등과 함께 바로 이 암묵의 공백을 정조준했다. 최적화기가 내놓은 해가 정말 전역 최적해인지 수학적으로 **인증(certify)**할 수는 없을까? Lagrangian duality에서 출발한 이 물음은 Briales와 Gonzalez-Jimenez의 Cartan-Sync(2017), Rosen의 SE-Sync(2019), Yang과 Carlone의 TEASER, 그리고 Papalia 등의 CORA로 이어지며 certifiable SLAM이라는 새 영역을 열었다. 흥미롭게도 이 돌파구를 연 도구들은 모두 SLAM 바깥에 있었다. Shor relaxation은 operations research에서, Burer-Monteiro factorization은 수리 최적화에서, Riemannian optimization은 미분기하에서, 그리고 Kirchhoff의 Matrix-Tree 정리는 19세기 전기 회로망 이론에서 건져 올린 보물이었다.

---

## 7.1 지역 최솟값 문제

그래프 SLAM backend의 첫 번째 아킬레스건은 초기값 의존성이다(→ §6.7). 비용 함수가 비볼록해지는 근본 원인은 3차원 회전 행렬 $\boldsymbol{R}_i \in \mathrm{SO}(3)$가 만족해야 하는 곡선형 비선형 제약에 있다. 초기 자세 추정이 참값에서 조금만 멀어져도 Gauss-Newton은 엉뚱한 골짜기로 미끄러져 내려간다. *The SLAM Handbook* 6장의 그림 6.1을 보면, 여러 층으로 겹친 주차장을 주행한 실제 로봇 데이터에서 이 차이가 적나라하게 드러난다. 전역 최적해를 인증해 내는 SE-Sync는 주차장 층들을 칼같이 쌓아 올린 반면, 무작위 초기값에서 출발한 국소 최적화기는 전혀 엉뚱하게 뭉개진 지역 최솟값에 갇혀 버린다.

2000년대 후반까지 실전 엔지니어들은 두 가지 방어선으로 이 문제를 버텨 냈다. 오도메트리로 최대한 정확한 초기값을 넘겨주고, frontend에서 잘못된 loop closure(outlier)를 가혹하게 걸러 내는 전략이었다. 물론 효과는 있었지만, 정작 수렴한 결과가 전역 최적인지 판별할 심판은 어디에도 없었다. 2010년 무렵 Shoudong Huang과 Gamini Dissanayake는 설령 초기값이 좋아도 데이터가 모호하면 최적화기가 틀린 답에서 멈출 수 있음을 지적했고, 이윽고 PGO의 여러 특수 사례가 계산 복잡도 이론상 NP-hard라는 사실까지 밝혀졌다. 그런데도 희한하게 실전에서는 g2o로 문제가 대개 잘 풀렸다. 2010년대 중반 이론 연구자들은 바로 그 간극—*최악의 경우 NP-hard인 문제가 왜 실제 로봇 데이터에서는 이토록 잘 풀리며, 언제 틀리는지*—을 파헤치기 시작했다.

Gauss-Newton은 어디까지나 국소 조건만 보장한다. 이론적인 국소 최솟값에서는 기울기가 0이고 Hessian이 양의 준정부호여야 하지만, 실제 수치 solver는 정지 기준이나 수치 오차 때문에 그 조건에 닿기 전에 일찍 멈추기도 한다. backend가 아무런 경고 없이 조용히 멈추는 한, 사용자는 지금 눈앞의 지도가 진짜인지 가짜인지 알 도리가 없었다.

> 🔗 **차용.** Huber나 Cauchy 같은 robust kernel은 20세기 중반의 강건 통계학(robust statistics)에서 빌려 온 도구다. 1996년 [Black과 Rangarajan](https://doi.org/10.1007/BF00131148)은 이런 robust 비용 함수를 outlier 가중치도 함께 최적화하는 형태로 바꿔 쓸 수 있음을 보였다(Black-Rangarajan duality). 이 대응 관계는 훗날 SLAM과 3D 점군 정합에서 GNC(graduated non-convexity, 볼록에 가까운 쉬운 문제에서 시작해 서서히 원래의 비볼록 비용으로 옮겨 가는 기법)를 설계하는 이론적 디딤돌이 됐다.

---

## 7.2 Shor relaxation

PGO가 비볼록인 유일한 이유가 회전 제약 $\boldsymbol{R}_i \in \mathrm{SO}(d)$ 때문이라면, 이 제약을 다룰 수 있는 다른 수학으로 옮겨 적을 수는 없을까? 먼저 직교성 조건 $\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}$는 명백한 이차 방정식이다. 3차원에서 오른손 좌표계를 강제하는 $\det(\boldsymbol{R})=+1$ 조건은 원래 삼차식이지만, 세 열벡터의 외적 관계($\boldsymbol{r}_1 \times \boldsymbol{r}_2 = \boldsymbol{r}_3$ 등)로 풀어 쓰면 역시 이차 제약으로 바뀐다. 목적함수 또한 자세들에 대한 이차식이므로, PGO 전체는 결국 **QCQP**(Quadratically Constrained Quadratic Program, 이차 제약 이차 계획)라는 표준 최적화 문제로 깔끔하게 번역된다.

바로 이 QCQP에 대해서는 1987년 소련의 수학자 Naum Shor가 이미 강력한 볼록 완화(convex relaxation) 기법을 만들어 두었다. Shor의 착안은 스칼라 대수의 항등식 $\boldsymbol{x}^\top \boldsymbol{M}\boldsymbol{x} = \mathrm{tr}(\boldsymbol{M}\boldsymbol{x}\boldsymbol{x}^\top)$에서 출발한다. 고차원의 대칭 행렬 변수 $\boldsymbol{X} \triangleq \boldsymbol{x}\boldsymbol{x}^\top$을 새로 도입해 문제를 한 차원 위로 끌어올리면(**lifting**), 원래의 모든 이차식은 $\boldsymbol{X}$에 대한 단순한 *선형식*으로 바뀐다. 비볼록성은 오직 "$\boldsymbol{X} \succeq 0$이고 **rank가 1**이다"라는 단 하나의 조건으로 집약된다. 이제 거기서 까다로운 rank-1 제약만 과감히 지워 버리면, 놀랍게도 전역 최적해를 다항 시간에 찾을 수 있는 순수 볼록 최적화 문제, 즉 **SDP**(semidefinite program, 반정부호 계획)가 탄생한다(변수 개수가 $n$개에서 $n(n+1)/2$개로 늘어나는 대가는 치러야 한다).

$$d^* = \min_{\boldsymbol{X}\in\mathbb{S}^n} \mathrm{tr}(\boldsymbol{C}\boldsymbol{X}) \;\; \text{s.t.} \;\; \mathrm{tr}(\boldsymbol{A}_i\boldsymbol{X})=b_i,\; \boldsymbol{X}\succeq 0.$$

이 완화가 마법을 부리는 지점은 부등식 $d^* \le p^*$에 있다. 완화한 SDP의 최솟값 $d^*$는 원래 풀려던 어려운 QCQP의 진짜 전역 최솟값 $p^*$보다 절대 클 수 없다(하한). 따라서 우리가 어떻게든 구한 후보해 $\hat{\boldsymbol{x}}$의 비용 $f(\hat{\boldsymbol{x}})$와 $d^*$의 차이인 $f(\hat{\boldsymbol{x}}) - d^*$를 재면, 지금 손에 쥔 해가 진짜 전역 최적에서 최대 얼마나 떨어져 있는지, 즉 **오차의 상한(suboptimality bound)**을 정확히 알 수 있다. **"certifiable(인증 가능한)"**이라는 수식어가 바로 여기서 나왔다. 게다가 만약 운 좋게 SDP의 해 $\boldsymbol{X}^*$의 rank가 정확히 1로 떨어지면($\boldsymbol{X}^* = \boldsymbol{x}^*\boldsymbol{x}^{*\top}$), 그 $\boldsymbol{x}^*$가 곧 원래 비볼록 PGO의 유일무이한 전역 최적해라는 수학적 **증명**까지 공짜로 얻는다.

SLAM에 이 렌즈를 처음 들이댄 것은 2015년 ICRA와 IROS에 실린 Luca Carlone의 두 논문, [Carlone & Dellaert (2015), "Duality-based verification techniques for 2D SLAM"](https://doi.org/10.1109/ICRA.2015.7139835)과 [Carlone et al. (2015), "Lagrangian duality in 3D SLAM"](https://arxiv.org/abs/1506.00746)이었다. 두 논문은 후보해가 전역 최적인지 Lagrangian dual로 검증하는 절차를 세웠고, 실제 벤치마크 실험에서 놀라울 만큼 많은 경우 duality gap($p^* - d^*$)이 정확히 0으로 떨어짐을 발견했다. 다만 *언제* gap이 0이 되는지까지는 아직 설명하지 못했다. 2017년에는 [Briales와 Gonzalez-Jimenez의 Cartan-Sync (RA-L 2017)](https://doi.org/10.1109/LRA.2017.2718661)가 pose synchronization 관점으로 이 맥을 이었고, 때마침 수리 최적화 쪽에서 2003년 무르익은 Burer-Monteiro의 low-rank SDP 분해와 Boumal·Absil·Sepulchre의 Riemannian optimization 기법이 맞물리며 결정타가 준비되고 있었다.

---

## 7.3 SE-Sync

[Rosen, Carlone, Bandeira, Leonard의 **SE-Sync** (*IJRR* 2019)](https://arxiv.org/abs/1612.07386)는 이 모든 수학적 조각을 단 하나의 고속 알고리즘으로 엮어 내며 certifiable SLAM의 대명사가 됐다. David Rosen은 MIT에서 John Leonard(Durrant-Whyte와 함께 EKF-SLAM 틀을 세운 바로 그 연구자다, → 4장)의 지도로 박사학위를 받았다. Rosen은 Carlone·Leonard, 그리고 SDP와 synchronization 이론의 대가인 수학자 Afonso Bandeira와 손잡고 Shor relaxation, 이동 변수 소거, Burer-Monteiro 매개화, Boumal의 Riemannian Staircase를 PGO 하나에 녹여 넣었다.

SE-Sync의 연산은 세 단계의 우아한 변환을 거친다. 첫째, 회전을 고정해 두면 이동 변수 $\boldsymbol{t}$는 단순한 선형 최소자승 문제가 된다는 점을 이용해(그래프 SLAM 연구자들이 예전부터 알던 성질이다) $\boldsymbol{t}$를 닫힌 형태로 미리 소거한다. 둘째, 오직 회전 변수만 남은 $\min_{\boldsymbol{R}\in\mathrm{SO}(d)^n} \mathrm{tr}(\tilde{\boldsymbol{Q}}\boldsymbol{R}^\top\boldsymbol{R})$에 Shor relaxation을 걸어 볼록한 SDP로 바꾼다. 셋째, 이 $dn \times dn$ 크기의 거대한 SDP를 일반 interior-point method로 풀면 자세가 몇천 개만 돼도 메모리가 터져 버리므로, Burer-Monteiro 저랭크 재매개화 $\boldsymbol{Z} = \boldsymbol{Y}^\top \boldsymbol{Y}$를 도입해 Stiefel manifold 위의 훨씬 작고 매끄러운 비제약 최적화 문제로 바꿔 푼다.

SE-Sync가 실전에서 전역 최적해를 확신하는 근거는 **exact recovery** 정리와 **Riemannian Staircase**의 결합이다. 먼저 측정 노이즈가 특정 임계 상수 $\beta$보다 작다면, SDP relaxation의 해 $\boldsymbol{Z}^*$는 유일하며 rank가 정확히 $d$로 떨어져 원래 최대우도 추정(MLE)의 전역 최적해를 한 치 오차 없이 복원한다는 것이 수학적으로 증명된다(일반 벡터 QCQP에서는 rank-1이 조건이지만, 여기서는 $\boldsymbol{Z}=\boldsymbol{R}^\top\boldsymbol{R}$이고 $\boldsymbol{R}\in\mathbb{R}^{d\times dn}$이므로 정확한 해의 rank가 $d$다). 물론 $\beta$는 알 수 없는 참값에 따라 달라 사전에 계산할 수 없다. 이때 Boumal 등이 증명한 정리가 구원투수로 등판한다. 이 정리에 따르면, Stiefel manifold 위에서 찾은 2차 임계점이 rank-deficient하면 그 점이 곧 원래 SDP의 전역 최솟값이다. SE-Sync의 Riemannian Staircase는 작은 rank에서 출발해 2차 임계점을 찾은 뒤 rank-deficiency를 검사하고, 조건을 못 채우면 rank를 한 칸씩 높여 다시 푼다. 이론상 rank가 $dn+1$에 닿으면 모든 $\boldsymbol{Y}$가 row rank-deficient해지므로 유한한 횟수 안에 반드시 종료하며, *SLAM Handbook* 6장에 따르면 실제 데이터에서는 대부분 한두 계단만 올라가도 답을 얻는다.

성능과 신뢰성 모두 압도적이었다. 합성 데이터(`sphere`, `torus`, `grid`)와 실측 데이터(`garage`, `cubicle`, `rim`) 전부에서 SE-Sync는 Gauss-Newton과 동일한 전역 최적해에 도달했을 뿐 아니라, 속도마저 `garage`에서 3.3배, `rim`에서는 무려 15.7배나 빨랐다. 무엇보다 g2o나 GTSAM이 해만 달랑 돌려주고 침묵하던 것과 달리, SE-Sync는 해와 함께 수학적인 **사후(a posteriori) 인증서**를 내놓았다. 인증을 통과하면 그 지도는 전역 최적임이 증명된 것이고, 통과하지 못하면 믿을 수 없는 불확실한 해임을 스스로 경고했다. Lu와 Milios 이후 20여 년 만에 비로소 SLAM backend가 스스로의 정답을 증명하는 눈을 뜬 것이다.

> 📜 **예언 vs 실제.** Rosen 등은 SE-Sync 결론에서 일부 측정에 outlier가 섞여도 견디는 강건한 인증 추정기와, 더 일반적인 polynomial optimization 문제 전반을 아우르는 구조적 SDP relaxation의 시대를 내다보았다. 그 전망은 이후 현실이 됐다. 점군 정합의 TEASER 계열, landmark SLAM을 다룬 Holmes와 Barfoot(2023), 거리 측정 기반의 CORA(Papalia et al., 2024)가 줄을 이었다. 한편 visual SLAM의 perspective projection은 분수식(유리함수)이라 다항 최적화에 곧바로 넣기 까다로웠는데, 2025년 RSS에서 Han과 Yang이 발표한 [XM](https://arxiv.org/abs/2502.04640)은 학습한 depth로 2D keypoint를 3D로 먼저 올려 projection 분모를 없앤 scaled bundle adjustment를 세우고, 이를 SfM 파이프라인에서 SDP relaxation으로 풀어 전역 최적성 인증서까지 받아 냈다.

> 🔗 **차용.** SE-Sync를 실시간 크기로 굴린 핵심 엔진인 Burer-Monteiro factorization은 [Burer와 Monteiro(2003)](https://link.springer.com/article/10.1007/s10107-002-0352-8)의 저랭크 SDP 해법에서 왔다. 여기에 미분기하의 눈을 얹어 2차 임계점이 전역 최적이 되는 조건을 뚫어 낸 것이 [Boumal, Voroninski, Bandeira(2016)](https://arxiv.org/abs/1606.04970)였고, Rosen이 이 수학을 곧바로 SLAM으로 가져왔다.

---

## 7.4 Graph Laplacian과 Fisher Information

"이게 진짜 전역 최솟값인가?"라는 질문에 답을 얻고 나면 곧장 두 번째 질문이 뒤따른다. *"그렇다면 그 전역 최적해는 실제 참값과 얼마나 가까운가?"* 추정의 정밀도 한계를 재는 고전적 잣대는 Cramér-Rao Lower Bound(CRLB)와 **Fisher Information Matrix(FIM)**다. 놀랍게도 회전을 고정한 단순 PGO 모델에서 FIM을 계산해 보면, 정확히 포즈 그래프의 가중 reduced Laplacian과 단위행렬의 Kronecker product로 떨어진다.

$$\mathcal{I} = \boldsymbol{J}^\top \boldsymbol{\Sigma}^{-1} \boldsymbol{J} = \boldsymbol{L}_w \otimes \boldsymbol{I}_3.$$

이 짧은 등식이 뜻하는 바는 엄청나다. 실제 센서 측정을 한 번도 해 보지 않고 그래프의 *연결 구조*만 쥐고 있어도 최종 추정 정확도가 어느 정도일지 미리 가늠할 수 있기 때문이다. 1847년 Gustav Kirchhoff가 전기 회로망의 전류를 풀려고 증명한 **Matrix-Tree 정리**에 따르면, reduced Laplacian의 행렬식은 그래프가 품은 가중 spanning tree의 총수와 같으며, 이는 곧 정보 행렬의 행렬식을 최대화하는 실험 설계 기준(D-optimality)과 정확히 맞닿는다. 또한 Laplacian의 두 번째로 작은 고윳값인 대수적 연결도(algebraic connectivity, Fiedler value)는 최악의 방향 분산을 억제하는 E-optimality에 대응한다. 19세기 전기 회로 정리가 21세기 로봇공학에서 "어느 경로로 움직여야 지도가 가장 정확해질까"를 계산하는 **active SLAM**과 loop closure 측정 선택의 수학적 심장으로 부활한 것이다.

이 연결 고리를 3D SLAM 전반으로 체계화한 주역은 [Kasra Khosoussi, Shoudong Huang, Gamini Dissanayake 등(2014년 이후)](https://doi.org/10.1177/0278364918823086)이었다(Khosoussi는 University of Technology Sydney에서 Huang과 Dissanayake의 지도로 2017년 박사를 받은 뒤 MIT로 옮겨 연구를 이었다). 3D PGO로 확장하더라도 Laplacian과 SE(3) adjoint 표현이 Kronecker product 형태로 깔끔히 결합해 그래프 위상 정보와 기하 정보를 분리해 다룰 수 있으며, Laplacian의 차원은 전체 FIM보다 6배나 작다. 덕분에 어느 loop closure를 남기고 버릴지 고를 때 무거운 FIM 대신 Laplacian 스펙트럼을 보면 계산이 훨씬 가벼워진다.

4장에서 본 EKF-SLAM의 **inconsistency(과신)** 문제 역시 이 CRLB의 눈으로 다시 보면 정체가 한눈에 드러난다. Julier와 Uhlmann(2001)이 짚은 EKF의 과신은 결국 잘못된 선형화로 Fisher information을 실제 물리적 한계보다 부풀려 계산하는 현상에 다름아니기 때문이다. *The SLAM Handbook* 6장이 6.1절에서 certifiable solver를 다루고 곧바로 6.2절에서 Fisher information과 추정 정확도를 나란히 배치한 것도, "최솟값을 확실히 찾는 일"과 "그 최솟값이 얼마나 믿을 만한지 재는 일"이 동전의 양면처럼 붙어 있기 때문이다.

> 🔗 **차용.** [Kirchhoff의 Matrix-Tree 정리(1847)](https://en.wikipedia.org/wiki/Kirchhoff%27s_theorem)는 원래 19세기 전기 저항 회로를 분석하기 위해 탄생해 조합론과 통계적 실험 설계를 거친 뒤, 2010년대 Khosoussi 등의 손을 거쳐 SLAM의 active mapping과 그래프 가지치기 기준으로 자리 잡았다.

---

## 7.5 확장과 한계: TEASER, CORA, Lasserre relaxation

SE-Sync가 순수 포즈 그래프의 문을 연 뒤, certifiable 연구는 현실의 더 거친 문제들—대규모 **outlier**가 판치는 환경, 거리 측정(range), landmark, 비등방 잡음(anisotropic noise)—쪽으로 뻗어 나갔다.

가장 먼저 부딪힌 난적은 **outlier**였다. 아무리 frontend가 깐깐해도 잘못된 loop closure가 섞여 들어가 일정 비율을 넘기면 Huber나 Cauchy kernel조차 맥없이 무너진다. 2017년 무렵부터 이 난제를 겨냥한 연구가 본격화됐고, 대표작이 [Yang, Shi, Carlone의 **TEASER** (*TRO* 2021)](https://arxiv.org/abs/2001.07715)다. TEASER는 3D 점군 정합 벤치마크에서 대응점의 **99% 이상이 가짜(outlier)**인 극한 상황에서도 scale이 주어졌을 때 정확한 변환을 건져 올렸다. 비결은 오차가 커지면 비용을 상수로 잘라 버리는 truncated least squares(TLS)로 문제를 다시 쓴 뒤, scale·rotation·translation을 분리해 scale과 translation은 다항 시간의 adaptive voting으로, rotation은 SDP relaxation으로 풀어 인증서까지 떼어 준 데 있다. 뒤이어 [Yang과 Carlone (*TPAMI* 2022)](https://arxiv.org/abs/2109.03349)은 TLS 같은 비볼록 강건 비용을 일반 다항 최적화 문제(POP)로 바꾸고, 무거운 Lasserre 계층보다 훨씬 가벼운 희소 SDP relaxation 체계를 제시했다.

센서 모달리티 확장에서는 UWB 같은 거리 측정을 결합한 range-aided SLAM을 다룬 [Papalia 등의 **CORA** (*TRO* 2024)](https://arxiv.org/abs/2302.11614)가 주목을 받았다. 거리 잔차 $(\|\boldsymbol{t}_j - \boldsymbol{t}_i\| - \tilde r_{ij})^2$에는 제곱근(노름)이 들어 있어 그대로는 QCQP가 되지 않는다. CORA는 두 점 사이의 방향을 나타내는 보조 단위벡터 $\boldsymbol{b}_{ij} \in S^{d-1}$를 변수로 끼워 넣어 이 문제를 깔끔한 QCQP로 바꿨다. 흥미롭게도 *SLAM Handbook* 6장에서 짚듯, CORA의 relaxation은 SE-Sync와 달리 노이즈가 아무리 작아도 *언제나* exact해지지는 않았다(예컨대 여러 로봇이 상대 자세 없이 거리만 서로 재는 경우). 그럼에도 그래프 연결이 튼튼한 다수의 실제 SLAM 환경에서는 exact recovery를 달성했다. 한편 landmark를 포함한 SLAM에서는 [Holmes와 Barfoot (*RA-L* 2023)](https://doi.org/10.1109/LRA.2023.3238173)이 Schur complement로 landmark 변수를 먼저 대수적으로 소거해 문제를 SE-Sync에 곧바로 태울 수 있는 형태로 다듬었다(Holmes, Khosoussi, Rosen은 바로 *SLAM Handbook* 6장의 공저자들이다).

그러나 벽도 뚜렷하다. 비등방 잡음과 truncated-quadratic outlier까지 한꺼번에 아우르려고 일반 POP로 넘어가면 **Lasserre moment relaxation**이 필요한데, 이 계층이 만들어 내는 거대한 SDP 행렬은 중복 제약이 무수히 얽혀 **degenerate**해지기 일쑤다. 그러면 Riemannian Staircase가 수렴을 보장받기 위해 기대던 constraint qualification이 깨지고 dual 해가 무수히 많아져 수치적 구현이 몹시 까다로워진다. 실제로 *Handbook*은 이런 SDP에는 Riemannian Staircase를 적용할 수 없다고 선을 긋는다. 2022년 Yang 등이 이 계층을 푸는 전용 solver를 내놓긴 했으나 여전히 국소 solver보다 한참 느리며, Visual SLAM의 perspective projection과 IMU preintegration을 이 다항식 틀에 무리 없이 담는 문제도 큰 난제로 남아 있다.

> 📜 **예언 vs 실제.** Carlone 등은 2015년 IROS 논문 말미에 *"Many theoretical questions remain open, e.g., under which conditions the duality gap is zero?"*라는 질문을 남겼다. 10년이 흐른 지금도 그 답은 절반만 채워졌다. SE-Sync는 노이즈가 $\beta$ 이하이면 exact하다는 충분조건을 증명하긴 했지만, 실제 현장의 데이터에서 그 $\beta$가 얼마인지 사전에 계산할 방법이 없기 때문이다. 그래서 2026년 오늘날에도 이론가와 엔지니어는 사전(a priori) 보장 대신 매 문제마다 풀고 난 뒤 **사후(a posteriori) 인증서**를 계산해 전역 최적 여부를 확인하는 전략을 쓴다.

---

## 아직 열린 문제

**Visual SLAM과 VIO에 인증을 얹기.** 핀홀 카메라의 perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$는 분모에 깊이 $Z$가 들어 있는 유리함수다. 다항 최적화에 넣으려고 분모를 양변에 곱해 다항식으로 바꾸면, 관측되는 feature마다 새 변수와 보조 제약이 눈덩이처럼 불어난다. ORB-SLAM3처럼 map point가 수천, 수만 개에 이르는 실시간 Visual SLAM에서 이 거대한 SDP를 제시간에 풀기란 불가능에 가깝다. Forster(2015)의 IMU preintegration 역시 Lie group 위의 exponential map과 bias drift가 얽혀 있어 다항식으로 쓰기 어렵다. 이 때문에 2026년 현재에도 주류 Visual SLAM과 VIO 시스템 대부분은 전역 최적성 인증 없이 국소 solver 위에서 돌아간다.

**Online certification과 규모 확장.** SE-Sync는 본질적으로 데이터를 모아 한 번에 푸는 배치(batch) 해법이다. Square Root SAM이 iSAM2로 증분화됐듯, certifiable solver 역시 새 측정이 들어올 때마다 인증서를 실시간으로 갱신하려는 연구가 warm-start, rank 증분, 부분 인증서 합성 등의 아이디어로 이어지고 있다. 도시 규모 그래프에서 무거운 moment relaxation solver의 속도를 끌어올리는 일 역시 갈 길이 멀다.

**극단적 outlier 환경의 일반화.** TEASER가 99% outlier를 뚫어 냈지만, 그 수학적 보장은 점군 정합 같은 특정 구조와 오염 조건 아래에서만 성립한다. 이를 일반적인 SLAM과 다중 모달 센서 문제로 넓히고, 그럴듯한 해석이 여러 개 공존할 때 단일 해 대신 후보 집합을 인증해 내는 list-decodable regression(Cheng, Shi, Carlone의 2024년 후속 연구 등)을 실용화하는 과제가 남아 있다.

---

Rosen, Khosoussi, Holmes, Dissanayake, Barfoot, Carlone이 공동 집필한 *The SLAM Handbook* 6장은 이 certifiable SLAM의 이론과 계보를 149~181쪽(2026년 4월 21일 공개본 기준)에 걸쳐 집대성한다. 한편 이들이 기하학적 인증의 대수학을 파고들던 같은 10년 동안, 또 다른 진영의 연구자들은 신경망이 이미지에서 곧바로 궤적과 지도를 예측하는 학습 기반 SLAM의 길을 넓히고 있었다(→ 23장 §23.4).

이제 1부와 2부에서 다룬 고전 기하(사진측량, 특징점, SfM)와 확률·그래프 최적화 backend(EKF, PTAM, g2o/iSAM2, SE-Sync)의 무기가 모두 손에 쥐어졌다. backend가 튼튼한 반석 위에 올라서자, 2010년대 중반 실전 Visual SLAM의 승부처는 다시금 *"어떻게 살아 움직이는 카메라의 이미지에서 특징을 뽑고, 방대한 키프레임을 엮어, 거대한 루프를 놓치지 않고 닫을 것인가"*라는 시스템 아키텍처의 문제로 돌아갔다. 그 모든 퍼즐을 하나로 조립해 낸 기념비적 시스템이 바로 **ORB-SLAM**이다(→ 8장).
