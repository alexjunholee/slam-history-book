# Ch.6 — Graph SLAM 혁명

1997년 Feng Lu와 Evangelos Milios는 레이저 스캔을 하나씩 누적 지도에 붙이는 방식이 등록 오차 때문에 일관되지 않은 지도를 만들 수 있다고 지적했다. 대신 각 스캔의 local frame과 frame 사이의 상대 공간 관계를 모두 유지하고, 그 제약을 동시에 풀어 전체 pose를 맞췄다. 다만 Lu-Milios가 이 방향의 유일한 시조는 아니다. 그보다 10여 년 앞서 LAAS의 [Chatila와 Laumond(1985)](https://www.semanticscholar.org/paper/Position-referencing-and-consistent-world-modeling-Chatila-Laumond/c34a678e40a7d80cb3683f07fc837179fd9bf3ee)가 이동 로봇의 참조 좌표계와 일관된 월드 모델을 논의했고, 1999년 [Gutmann과 Konolige](https://www.semanticscholar.org/paper/Incremental-mapping-of-large-cyclic-environments-Gutmann-Konolige/3c1bda51b8ca59f1836ed1b96c485d905804989a)가 대형 순환 환경의 증분 지도 작성에 포즈 정합을 적용했으며, 2000년대 초 Thrun 그룹이 *full SLAM* 문제로 이 접근을 정식화했다. [Folkesson과 Christensen(2004)](http://www.hichristensen.net/hic-papers/folkesson-icra2004.pdf), Konolige, Dellaert도 뒤이어 각자의 정식화를 내놓았다. Lu-Milios 1997의 분명한 기여는 "레이저 스캔 정합 + 배치 최대우도 추정"이라는 구체적인 파이프라인을 완결된 형태로 제시한 데 있다. Smith-Cheeseman이 확률 지도의 수학적 토대를 놓고 Davison이 실시간 단안 SLAM의 가능성을 보인 사이, 이 병렬 기여자들은 SLAM을 전체 궤적의 동시 추정 문제로 바꾸고 있었다. 2000년대의 EKF-SLAM은 landmark 수가 늘수록 $O(N^2)$ 공분산 갱신에 막혔고, Klein과 Murray의 PTAM(2007)은 별도의 BA 기반 keyframe 구조로 tracking과 mapping을 나눠 실시간 최적화의 가능성을 보였다. 필터와 나란히 발전한 graph-smoothing 해법은 여러 연구 집단의 작업을 거쳐 SLAM backend의 한 축이 되었다.

---

## 6.1 레이저 스캔에서 포즈 그래프로: Lu-Milios 1997

[Lu & Milios 1997. "Globally Consistent Range Scan Alignment"](https://doi.org/10.1023/A:1008854305733)이 등장하기 전까지, 연속 레이저 스캔의 정합(alignment)은 ICP(Iterative Closest Point) 계열의 국소 정합으로 이어 붙이는 경우가 많았다. ICP는 두 스캔을 국소적으로 잘 맞추지만, 드리프트가 누적되면 수십 미터 이후 지도가 뒤틀린다. 루프를 다시 돌아왔을 때 출발점과 지도가 맞지 않는다.

Lu와 Milios의 아이디어는 단순했다. 로봇의 포즈 시퀀스 $x_1, x_2, \ldots, x_n$을 노드로, 각 포즈 쌍 사이의 상대 측정값을 엣지로 표현하면, 지도 구성 문제는 그래프 위의 에너지 최소화 문제가 된다. 엣지 하나하나는 두 포즈 사이의 상대변환 $\hat{z}_{ij}$와 그 불확실성 $\Omega_{ij}$를 담는다. 전체 비용 함수는

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

여기서 $h(x_i, x_j)$는 두 포즈로부터 기대 상대변환을 계산하는 함수이며, $z_{ij}$는 실제 측정된 상대변환, $\Omega_{ij} = \Sigma_{ij}^{-1}$는 측정 불확실성의 역행렬인 정보 행렬이다.

루프 클로저도 이 공식에 자연스럽게 포함된다. 나중에 같은 장소를 다시 방문했을 때 얻은 상대 측정값을 그래프에 엣지로 추가하면, 전체 최적화가 그 제약을 반영하여 모든 포즈를 조정한다. EKF에서 루프 클로저는 covariance를 $O(N^2)$ 단위로 갱신하는 무거운 작업이었다. 포즈 그래프에서는 엣지 하나로 제약을 표현하지만, 이를 추정에 반영하려면 그래프를 다시 최적화해야 한다.

> 🔗 **차용.** Lu-Milios의 포즈 그래프 최적화 정식화는 [Levenberg(1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/)와 [Marquardt(1963)](https://www.stat.cmu.edu/technometrics/70-79/VOL-14-03/v1403757.pdf)의 비선형 최소자승 알고리즘을 기반으로 한다. 수십 년 앞서 비선형 파라미터 추정을 위해 개발된 수치 최적화 기법이 실내 레이저 맵핑의 백엔드에 도착했다.

당시 Lu-Milios의 해법은 모든 포즈를 동시에 푸는 배치(batch) 선형 시스템이었다. 스캔 수가 늘어나면 선형 시스템의 크기도 함께 커진다. 그래서 개념 증명의 성격이 강했다. 그러나 전역 일관성을 달성할 수 있으며, 그 도구가 필터가 아닌 최적화임을 보여주었다. 같은 시기 Gutmann-Konolige는 증분성에, Folkesson-Christensen은 데이터 연관 강건성에, Thrun 그룹은 대규모 실환경 적용에 방점을 찍으며 같은 결론을 서로 다른 문제에서 구체화했다.

---

## 6.2 희소성의 발견: 정보 행렬과 포즈 그래프의 확장

Lu-Milios의 아이디어가 발표된 후 5년간, 여러 그룹이 같은 방향에서 확장을 시도했다. 공통된 발견은 정보 행렬(information matrix, $\Omega = \Sigma^{-1}$)의 **희소성(sparsity)**이었다.

EKF-SLAM의 covariance 행렬 $\Sigma$는 조밀(dense)하다. 로봇이 새 landmark를 관측할 때마다 기존 모든 landmark와의 상관관계가 갱신된다. 로봇 포즈를 marginalize한 상태에서 $n$개의 2D landmark가 있으면 $\Sigma$는 $2n \times 2n$ 행렬이고, 갱신 비용은 $O(n^2)$다. 100개 landmark 정도에서 실시간성이 무너지는 이유다.

반면 포즈 그래프의 정보 행렬은 다르다. 로봇의 포즈 $x_i$와 $x_j$가 직접 측정 관계에 있을 때만 $\Omega$의 $(i,j)$ 블록에 비영(non-zero) 항이 생긴다. 연속 이동 시 인근 포즈들만 엣지로 연결되고, 먼 포즈들은 직접 연결되지 않는다. $\Omega$는 그래프 토폴로지를 반영한 띠형(banded) 희소 구조를 가진다. 루프 클로저가 없는 순수 주행 시나리오에서 이 구조는 정확히 tridiagonal에 가깝다.

Sebastian Thrun 그룹의 [Sparse Extended Information Filter(SEIF)](http://www.cs.cmu.edu/~thrun/papers/thrun.tr-seif02.pdf), Edwin Olson의 연구는 이 희소성을 명시적으로 활용하기 시작했다. 희소 선형 대수 풀이기(sparse solver)를 쓰면 계산 비용이 $O(n^2)$에서 크게 줄어들 수 있었다. 실제 복잡도는 그래프 구조에 의존하지만, 로봇이 제한된 지역 내에서 움직이는 현실 시나리오에서는 $O(n \log n)$ 수준이 가능했다.

> 🔗 **차용.** Thrun 그룹의 sparse information filter(SEIF)와 [Eustice의 exactly sparse delayed-state filter](https://web.mit.edu/2.166/www/handouts/eustice_et_al_ieeetro_2006.pdf)는 정보 행렬의 희소성이 필터 기반에서도 활용 가능하다는 것을 보였다. 이 희소성 통찰은 Dellaert의 factor graph 공식화와 Bayes tree 자료구조로 이어지는 맥락을 형성한다.

2006년 ICRA에서 [Olson, Leonard, Teller](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf)는 stochastic gradient descent로 포즈 그래프를 최적화하는 방법을 발표했다. 수렴 보장은 없었다. 그래도 수백 노드 규모에서 충분히 빠르게 돌았고, Olson의 구현 코드는 이후 커뮤니티 전반에 퍼졌다.

---

## 6.3 Factor Graph와 Square Root SAM

2006년 Dellaert와 당시 박사과정이던 Kaess가 발표한 [Square Root SAM](https://doi.org/10.1177/0278364906072768)은 SLAM 백엔드를 factor graph로 정식화했다. Dellaert는 Georgia Tech에서 확률론적 그래픽 모델(probabilistic graphical model)을 연구해 왔다. Square Root SAM은 SLAM을 베이지안 추론 문제로 표현하고 factor graph 위에서 그 추론을 수행했다.

**Factor graph**(변수 노드와 factor 노드를 엣지로 연결한 이분 그래프)에서 변수 노드는 로봇 포즈와 landmark의 위치, factor 노드는 관측값 또는 사전 확률(prior)이다. Factor $f_k(x_{i_1}, x_{i_2}, \ldots)$는 연결된 변수들 사이의 확률적 제약을 나타낸다. 전체 결합 확률은

$$p(X) \propto \prod_k f_k(X_k)$$

이며, MAP 추정은 이 확률을 최대화하는 $X^*$를 찾는 것이다. Gaussian factor 하에서 이것은 비선형 최소자승 문제가 된다.

Jacobian 행렬 $J$에 QR 분해를 적용하면 상삼각(upper triangular) 행렬 $R$이 남는다. $R^T R = J^T J = \Omega$이며, $R$이 바로 "square root information matrix"다. 이 $R$의 희소 구조는 Jacobian 자체가 아니라 변수 제거(variable elimination) 순서와 factor graph 토폴로지가 결정한다. 적절한 ordering(예: AMD, COLAMD)을 선택하면 fill-in을 최소화하여 희소한 $R$을 얻을 수 있다.

이 공식화는 EKF의 covariance 갱신보다 수치적으로 안정하다. 지도 전체의 랜드마크와 포즈를 일관된 방식으로 함께 최적화할 수 있으며, 루프 클로저는 새 factor를 추가하는 것으로 표현된다.

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM과 iSAM2: 온라인 증분 추론

Square Root SAM은 배치(batch) 방법이었다. 새 관측이 들어올 때마다 전체 $J^T J$를 다시 분해해야 한다. 밀집 행렬의 비용은 $O(n^3)$이며, 희소 행렬에서는 연결 구조와 소거 순서에 따라 달라진다. 온라인 로봇 시스템에서는 실용적이지 않았다.

2008년 [Kaess, Ranganathan, Dellaert가 발표한 **iSAM**(incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)은 이 문제를 Givens rotation으로 접근했다. 새 변수와 factor가 추가될 때, 기존 QR 분해를 처음부터 다시 수행하는 대신 새 행만 추가하여 Givens rotation으로 $R$을 갱신한다.

iSAM1의 본질적 한계는 재선형화 스케줄이었다. 비선형 factor를 선형화한 결과로 만든 $R$은 현재 추정값 근처의 1차 근사일 뿐이다. 로봇이 이동하면서 추정값이 선형화 지점에서 멀어지면 근사 오차가 누적된다. iSAM1의 대응은 **주기적 전면 재선형화(periodic full relinearization)**였다. 몇십 스텝마다 전체 factor graph를 처음부터 다시 선형화하고 QR 분해를 처음부터 다시 수행했다. 루프 클로저로 $R$에 채움(fill-in)이 발생해 희소 구조가 손상되는 것은 이 스케줄이 촉발되는 가시적 증상이었지만, 비용의 근본은 "전체를 주기적으로 다시 푼다"는 스케줄 자체에 있었다. 증분적으로 보이던 알고리즘이 주기마다 배치 알고리즘으로 되돌아가는 구조였다.

2012년 [iSAM2](https://doi.org/10.1177/0278364911430419)는 Bayes tree라는 자료구조로 이 문제를 해결했다. Bayes tree는 factor graph에 variable elimination을 적용하여 얻는 chordal Bayes net으로부터 구성되는 트리 구조다. Bayes net의 클리크(clique)를 노드로, 클리크 간 공유 변수(separator)를 엣지로 가진다. 새 factor가 추가될 때 Bayes tree에서 영향받는 클리크를 특정하고, 해당 서브트리만 factor graph로 되돌려 재선형화·재최적화한다. **Fluid relinearization**은 선형화 오차가 임계값을 넘는 factor만 선택적으로 골라 다시 선형화하고, 그 영향이 Bayes tree의 separator를 타고 필요한 만큼만 전파한다. iSAM1의 "주기마다 전체" 스케줄이 "필요한 factor만, 영향받는 clique만"으로 대체된 셈이다. 루프 클로저가 발생해도 연결 clique 집합이 국소적으로 한정되는 경우가 많아 전체 재계산을 피할 수 있었다.

> 🔗 **차용.** Bayes tree의 자료구조적 아이디어는 확률론적 그래픽 모델 문헌의 junction tree(join tree) 알고리즘 계보를 잇는다. Koller-Friedman의 [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/) 같은 표준 교과서가 다루는 제거 순서·chordal 그래프 기반 추론 기법이 대표적이다. 인공지능 추론 커뮤니티의 기법 계열이 실시간 로봇 SLAM에 이식된 것이다.

iSAM2는 [GTSAM(Georgia Tech Smoothing and Mapping)](https://gtsam.org) 라이브러리로 패키징됐다. C++ 코어에 Python 바인딩을 얹은 형태다. Dellaert가 Georgia Tech 재직 중 Google과도 일하던 시기에도 GTSAM 개발은 이어졌다. GTSAM의 공개 문서와 사례에는 자율주행, 드론, 로봇팔 보정 등 여러 응용이 포함된다.

---

## 6.5 g2o: ROS 생태계의 범용 그래프 최적화기

뮌헨 공대(TUM)·프라이부르크의 Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, Wolfram Burgard는 2011년 ICRA에서 [g2o](https://doi.org/10.1109/ICRA.2011.5979949)(general graph optimization)를 발표했다. g2o는 "어떤 종류의 그래프 최적화든 플러그인 방식으로 처리한다"는 원칙으로 설계된 실용적인 오픈소스 구현이었다.

g2o의 설계는 세 개념을 분리한다. vertex(변수 노드)와 edge(factor/제약)가 그래프를 구성하고, solver가 희소 선형 시스템을 푼다. 사용자는 vertex 타입과 edge의 오차 함수·Jacobian을 정의하면, g2o가 Gauss-Newton 또는 Levenberg-Marquardt로 전체 최적화를 수행한다. 희소 풀이기는 Cholmod, CSparse, Eigen 중 선택하거나 외부 라이브러리로 교체할 수 있다.

ROS(Robot Operating System)가 2010년대 초 모바일 로봇 연구에 널리 퍼지면서 g2o도 그래프 기반 SLAM의 대표 구현 가운데 하나가 됐다. ORB-SLAM과 LSD-SLAM이 g2o를 채택했지만, gmapping은 particle-filter 계열이고 Cartographer는 Ceres를 쓰므로 ROS SLAM 전체가 g2o로 통일된 것은 아니다. 영향력은 컸지만 범용 표준 하나가 모든 backend를 대체한 역사는 아니었다.

---

## 6.6 왜 분야가 여기로 수렴했나

Chatila-Laumond(1985), Lu-Milios(1997), Gutmann-Konolige(1999), Folkesson-Christensen(2004), Thrun 그룹, Dellaert(2006), Kaess(2012)까지 여러 그룹이 서로 다른 문제에서 출발해 그래프 제약을 유지하고 푸는 도구를 발전시켰다.

문제 모델링 방식이 바뀌었다. EKF-SLAM은 현재 상태의 최적 추정값과 불확실성을 유지하면서 과거를 marginalize한다. 이 필터 패러다임에서 과거 포즈는 사라지고, 누적 오차는 현재 추정값 속에 잠복한다. 루프 클로저를 닫으려면 현재 covariance에 무거운 갱신이 필요하다.

그래프 SLAM은 과거 포즈를 버리지 않는다. 포즈·landmark·관측값 모두 그래프에 살아 있고, 루프 클로저는 새 엣지를 추가하는 것으로 표현된다. 재최적화가 전체 궤적을 일관성 있게 조정한다(이산 keyframe 대신 시간 연속적인 궤적으로 그래프를 재정식화하는 계열은 Ch.7c Continuous-Time SLAM 참조). 이미 지나간 포즈도 수정 대상이 된다는 점이 필터와의 본질적 차이다.

계산 비용도 달랐다. EKF의 갱신 비용은 $O(N^2)$ (landmark 수 $N$에 대해), 정보 저장은 $O(N^2)$다. 그래프 방법은 희소 Cholesky(또는 QR) 분해를 활용하면 복잡도가 크게 줄어든다. 다만 갱신 비용은 그래프 연결, 분해 과정의 fill-in, 소거 순서와 다시 계산할 영역에 달려 있다. 공간적으로 좁은 곳을 움직인다는 조건만으로 $O(N \log N)$을 보장할 수는 없다.

> 📜 **예언 vs 실제.** Dellaert의 Square Root SAM(2006)이 제시한 배치 방식의 한계는 같은 그룹에서 곧바로 증분화 방향으로 이어졌다. 2008년 iSAM이 Givens rotation 기반 증분 갱신으로 이를 다뤘고, 2012년 iSAM2는 Bayes tree로 루프 클로저 상황의 효율성까지 끌어올렸다. GTSAM·Ceres·g2o는 비선형 최소제곱 문제를 다루지만, 사용 가능한 solver와 증분 자료구조는 서로 다르다. 세 논문은 동일한 문제 의식을 단계적으로 해소했으며, 이 계보는 거의 예고한 대로 실현됐다.

마지널리제이션(marginalization)의 유연성도 한몫했다. 그래프에서 오래된 포즈를 marginalize할 때 그 정보가 남은 변수들에 연결 factor로 보존된다. 필터도 과거 상태를 제거하며 그 정보를 현재 추정에 전달한다. 양쪽 모두 압축 과정의 근사와 재선형화 제약을 살펴야 한다. 슬라이딩 윈도우 최적화나 keyframe 선택 같은 공학적 트레이드오프가 여기서 등장한다.

---

## 6.7 비선형성과 강건성: 실무 엔지니어링의 층위

그래프 최적화의 이론적 우아함과 실제 구현 사이에는 간격이 있다. 그 간격을 메우는 작업이 2010년대 SLAM 엔지니어링의 상당 부분을 차지했다.

초기값 의존성이 한 문제다. 가우스-뉴턴이나 LM 최적화는 초기 포즈 추정이 참값에서 크게 벗어나 있으면 지역 최솟값(local minimum)에 수렴한다. 루프 클로저에서 잘못된 대응 관계가 섞이면 초기값이 훼손된다. 그래서 루프 클로저 검증과 아웃라이어 rejection이 백엔드 이전 단계의 핵심 작업이 됐다. 이 지역 최솟값 문제 자체를 볼록 완화(SDP)로 우회하여 전역 최적성을 증명 가능한 형태로 푸는 계열은 Ch.6b(Certifiable SLAM)에서 별도로 다룬다.

표준 최소자승은 아웃라이어에 취약하다는 것도 실무에서 금방 드러났다. Huber 비용이나 Cauchy 비용 같은 robust kernel을 쓰면 잘못된 매칭의 영향을 줄일 수 있다. g2o와 GTSAM 모두 robust kernel을 선택 가능하게 한다. 어느 kernel을 쓸지는 환경과 센서 특성에 따라 달라지며, 2026년에도 이 선택은 여전히 엔지니어의 경험에 의존한다.

Marginalization 근사도 문제다. iSAM2의 Bayes tree는 정확한 증분 추론을 제공하지만, 변수 수가 계속 증가하면 트리가 커진다. 실제 시스템에서는 오래된 포즈를 marginalize하여 트리 크기를 관리한다. 이 marginalization 과정에서 발생하는 fill-in이 information matrix를 조밀하게 만들 수 있다. 어떻게 truncate할지, Prior factor로 어떻게 근사할지가 구현 품질을 가른다.

> 📜 **예언 vs 실제.** g2o가 표방한 범용성은 특정 센서 목록을 모두 기본 제공한다는 뜻이 아니라, 사용자가 상태를 vertex로, 관측 제약을 edge로 정의해 같은 최적화 뼈대에 얹을 수 있다는 뜻이었다. 이후 연구들은 line·plane·관성·객체 제약을 각자의 시스템에 맞는 사용자 정의 edge로 구현했다. 어느 시스템을 사례로 들 때에는 그 시스템이 실제로 쓰는 추정기와 edge 구현을 확인해야 한다. 2026년에도 g2o의 핵심 유산은 고정된 factor 목록보다 이 확장 인터페이스에 있다.

---

## 🧭 아직 열린 것

어느 robust kernel을 선택해야 하는가. Huber, Cauchy, Geman-McClure, DCS 등 여러 선택지가 있지만, 주어진 환경과 센서에 어느 kernel이 최적인지를 사전에 결정하는 원칙적인 방법이 없다. 이 선택은 여전히 엔지니어의 직관과 경험에 의존한다. 학습 기반으로 cost function 자체를 최적화하는 연구가 있으나, 온라인 증분 시스템에 통합하는 것은 풀리지 않은 문제다.

비가우시안 상황을 factor graph 안에서 표현하는 것은 아직 열려 있다. GTSAM·g2o의 기본적인 연속 최적화는 가우시안 residual 모델에서 출발하지만, robust kernel·max-mixture·hybrid factor 같은 확장도 있다. 그래도 루프 클로저의 오매칭 확률이나 다중 가설 포즈를 정확하고 실시간으로 표현하는 범용 해법은 없다.

Bayes tree의 증분 효율은 새 factor가 건드리는 clique가 국소적일 때 가장 크다. 대규모 지도에서 loop-closure 제약이 조밀해지면 영향을 받는 subtree와 fill-in이 커져 계산량과 메모리가 늘 수 있다. 계층적 관리와 submap 분할은 이 확장을 제어하는 접근이다.

---

2010년대 들어 백엔드 논쟁은 잦아들었다. g2o와 GTSAM 같은 대표 도구가 널리 쓰이면서, 연구자들의 관심은 백엔드 위에 무엇을 얹느냐로 옮겨갔다. 어떤 feature로, 얼마나 멀리서 루프를 인식하는가가 새 물음이 되었다. 프론트엔드가 새 경쟁 무대였다.

한 가지 질문은 남았다. g2o와 GTSAM이 내놓은 해가 실제 전역 최솟값인가. Ch.6b는 프론트엔드 계보를 Ch.7에서 이어가기 전에 이 certifiability 문제를 다룬다.
