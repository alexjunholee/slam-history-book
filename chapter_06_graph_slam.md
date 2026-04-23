# Ch.6 — Graph SLAM 혁명

1997년 카네기 멜런의 한 지하 복도. Feng Lu와 Evangelos Milios는 레이저 스캔 여러 장을 서로 일관성 있게 맞추는 문제를 붙들고 있었다. EKF는 표준 선택지였지만, 두 사람은 다른 길을 택했다. 포즈들 사이의 상대 측정값을 직접 그래프로 모델링하고, 그 그래프 위에서 최소자승 최적화를 돌리는 것이었다. 결과는 Kalman 계열이 도달하지 못한 전역 일관성이었다. Smith-Cheeseman이 확률 지도의 수학적 토대를 놓았고, Davison이 실시간 단안 SLAM의 가능성을 증명했다면, Lu-Milios는 SLAM을 그래프 추론 문제로 재정의하는 첫 수를 두었다. Klein과 Murray의 PTAM(2007)은 tracking과 mapping을 분리하여 실시간 성능을 얻었지만, 수백 개 포즈가 누적될수록 EKF 백엔드의 $O(N^2)$ 갱신 비용이 병목이 되었다. 그 문제의 해답이 이미 10년 전 CMU 지하 복도에서 나와 있었다.

---

## 6.1 레이저 스캔에서 포즈 그래프로: Lu-Milios 1997

[Lu & Milios 1997. "Globally Consistent Range Scan Alignment"](https://doi.org/10.1023/A:1008854305733)이 등장하기 전까지, 연속 레이저 스캔의 정합(alignment)은 ICP(Iterative Closest Point) 계열의 국소 정합으로 이어 붙이는 경우가 많았다. ICP는 두 스캔을 국소적으로 잘 맞추지만, 드리프트가 누적되면 수십 미터 이후 지도가 뒤틀렸다. 루프를 다시 돌아왔을 때 출발점과 지도가 맞지 않았다.

Lu와 Milios의 아이디어는 단순했다. 로봇의 포즈 시퀀스 $x_1, x_2, \ldots, x_n$을 노드로, 각 포즈 쌍 사이의 상대 측정값을 엣지로 표현하면, 지도 구성 문제는 그래프 위의 에너지 최소화 문제가 된다. 엣지 하나하나는 두 포즈 사이의 상대변환 $\hat{z}_{ij}$와 그 불확실성 $\Omega_{ij}$를 담는다. 전체 비용 함수는

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

여기서 $h(x_i, x_j)$는 두 포즈로부터 기대 상대변환을 계산하는 함수이며, $z_{ij}$는 실제 측정된 상대변환, $\Omega_{ij} = \Sigma_{ij}^{-1}$는 측정 불확실성의 역행렬인 정보 행렬이다.

이 공식화의 핵심은 루프 클로저의 자연스러운 포함이다. 나중에 같은 장소를 다시 방문했을 때 얻은 상대 측정값을 그래프에 엣지로 추가하면, 전체 최적화가 그 제약을 반영하여 모든 포즈를 조정한다. EKF에서 루프 클로저는 covariance를 $O(N^2)$ 단위로 갱신하는 무거운 작업이었다. 포즈 그래프에서는 엣지 하나를 추가하는 것으로 충분하다.

> 🔗 **차용.** Lu-Milios의 포즈 그래프 최적화 정식화는 [Levenberg(1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/)와 [Marquardt(1963)](https://www.stat.cmu.edu/technometrics/70-79/VOL-14-03/v1403757.pdf)의 비선형 최소자승 알고리즘을 기반으로 한다. 수십 년 앞서 비선형 파라미터 추정을 위해 개발된 수치 최적화 기법이 실내 레이저 맵핑의 백엔드에 도착했다.

당시 Lu-Milios의 해법은 모든 포즈를 동시에 푸는 배치(batch) 선형 시스템이었다. 스캔 수가 늘어나면 선형 시스템의 크기도 함께 커진다. 그래서 현장 적용보다는 개념 증명에 가까웠다. 그러나 두 가지를 확실히 보여주었다. 전역 일관성은 달성 가능하다. 그리고 그 도구는 필터가 아닌 최적화다.

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

2006년 Dellaert와 당시 박사과정이던 Kaess가 발표한 [Square Root SAM](https://doi.org/10.1177/0278364906072768)은 SLAM 백엔드를 이해하는 방식을 다시 썼다. Dellaert는 Georgia Tech에서 확률론적 그래픽 모델(probabilistic graphical model)을 연구해왔다. 그는 SLAM을 베이지안 추론 문제로 보았고, factor graph 위에서 그 추론을 수행하는 것이 가장 자연스럽다고 봤다.

**Factor graph**(변수와 제약을 노드와 엣지로 표현한 이분 그래프)에서 변수 노드는 로봇 포즈와 landmark의 위치, factor 노드는 관측값 또는 사전 확률(prior)이다. Factor $f_k(x_{i_1}, x_{i_2}, \ldots)$는 연결된 변수들 사이의 확률적 제약을 나타낸다. 전체 결합 확률은

$$p(X) \propto \prod_k f_k(X_k)$$

이며, MAP 추정은 이 확률을 최대화하는 $X^*$를 찾는 것이다. Gaussian factor 하에서 이것은 비선형 최소자승 문제가 된다.

Dellaert의 통찰은 이 최소자승 문제의 구조에서 왔다. Jacobian 행렬 $J$에 QR 분해를 적용하면 상삼각(upper triangular) 행렬 $R$이 남는다. $R^T R = J^T J = \Omega$이며, $R$이 바로 "square root information matrix"다. 이 $R$의 희소 구조는 Jacobian 자체가 아니라 변수 제거(variable elimination) 순서와 factor graph 토폴로지가 결정한다. 적절한 ordering(예: AMD, COLAMD)을 선택하면 fill-in을 최소화하여 희소한 $R$을 얻을 수 있다.

이 공식화는 EKF의 covariance 갱신보다 수치적으로 안정하다. 지도 전체의 랜드마크와 포즈를 일관된 방식으로 함께 최적화할 수 있으며, 루프 클로저는 새 factor를 추가하는 것으로 표현된다.

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM과 iSAM2: 온라인 증분 추론

Square Root SAM은 배치(batch) 방법이었다. 새 관측이 들어올 때마다 전체 $J^T J$를 다시 분해하면 $O(n^3)$ 비용이 발생한다. 온라인 로봇 시스템에서는 실용적이지 않았다.

2008년 [Kaess, Ranganathan, Dellaert가 발표한 **iSAM**(incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)은 이 문제를 Givens rotation으로 접근했다. 새 변수와 factor가 추가될 때, 기존 QR 분해를 처음부터 다시 수행하는 대신 새 행만 추가하여 Givens rotation으로 $R$을 갱신한다. 선형화 오차가 누적되면 전체를 주기적으로 재선형화(periodic relinearization)하는 방식으로 정확도를 유지했다.

iSAM의 한계는 루프 클로저였다. 루프가 닫힐 때 멀리 떨어진 변수들이 갑자기 연결되면, $R$ 행렬에 채움(fill-in)이 발생하고 희소 구조가 손상된다. 이런 상황에서는 비용이 크게 증가했다.

2012년 [iSAM2](https://doi.org/10.1177/0278364911430419)는 Bayes tree라는 자료구조로 이 문제를 해결했다. Bayes tree는 factor graph에 variable elimination을 적용하여 얻는 chordal Bayes net으로부터 구성되는 트리 구조다. Bayes net의 클리크(clique)를 노드로, 클리크 간 공유 변수(separator)를 엣지로 가진다. 새 factor가 추가될 때 Bayes tree에서 영향받는 클리크를 특정하고, 해당 서브트리만 factor graph로 되돌려 재선형화·재최적화한다. 루프 클로저가 발생해도 연결 클리크 집합이 국소적으로 한정되는 경우가 많아 전체 재계산을 피할 수 있었다.

> 🔗 **차용.** Bayes tree의 자료구조적 아이디어는 확률론적 그래픽 모델 문헌의 junction tree(join tree) 알고리즘 계보를 잇는다—Koller-Friedman의 [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/) 같은 표준 교과서가 다루는 제거 순서·chordal 그래프 기반 추론 기법이 대표적이다. 인공지능 추론 커뮤니티의 기법 계열이 실시간 로봇 SLAM에 이식된 것이다.

iSAM2는 [GTSAM(Georgia Tech Smoothing and Mapping)](https://gtsam.org) 라이브러리로 패키징됐다. C++ 코어에 Python 바인딩을 얹은 형태다. Dellaert가 나중에 Google로 옮긴 후에도 GTSAM 개발은 끊기지 않았다. 2026년 기준으로 자율주행, 드론, 로봇팔 보정 등 여러 분야에서 사실상 표준 SLAM 백엔드로 쓰인다.

---

## 6.5 g2o: ROS 생태계의 표준

Georgia Tech 그룹이 이론 정제에 집중하는 동안, 뮌헨 공대(TUM)의 Rainer Kümmerle와 Giorgio Grisetti, Wolfram Burgard 그룹은 실용적인 오픈소스 구현에 집중했다. 2011년 ICRA에서 이들이 발표한 [g2o](https://doi.org/10.1109/ICRA.2011.5979949)(general graph optimization)는 "어떤 종류의 그래프 최적화든 플러그인 방식으로 처리한다"는 원칙으로 설계됐다.

g2o의 설계는 세 개념을 분리한다. vertex(변수 노드)와 edge(factor/제약)가 그래프를 구성하고, solver가 희소 선형 시스템을 푼다. 사용자는 vertex 타입과 edge의 오차 함수·Jacobian을 정의하면, g2o가 Gauss-Newton 또는 Levenberg-Marquardt로 전체 최적화를 수행한다. 희소 풀이기는 Cholmod, CSparse, Eigen 중 선택하거나 외부 라이브러리로 교체할 수 있다.

ROS(Robot Operating System)가 2010년대 초 모바일 로봇 연구의 표준 플랫폼으로 자리잡으면서, g2o는 사실상의 SLAM 백엔드 표준이 되었다. gmapping, Cartographer, ORB-SLAM, LSD-SLAM이 g2o 또는 g2o와 유사한 인터페이스를 채용했다. 포즈 그래프 SLAM을 새로 구현하려는 연구자가 있다면 g2o를 첫 번째 선택지로 고려했다.

---

## 6.6 왜 분야가 여기로 수렴했나

Lu-Milios(1997), Dellaert(2006), Kaess(2012) 세 그룹은 다른 도구로, 서로 다른 시기에, EKF 백엔드만으로는 충분하지 않다는 같은 결론에 도달했다.

전환의 핵심은 알고리즘 교체가 아니라 문제 모델링의 전환이었다. EKF-SLAM은 현재 상태의 최적 추정값과 불확실성을 유지하면서 과거를 marginalize한다. 이 필터 패러다임에서 과거 포즈는 사라지고, 누적 오차는 현재 추정값 속에 잠복한다. 루프 클로저를 닫으려면 현재 covariance에 무거운 갱신이 필요하다.

그래프 SLAM은 과거 포즈를 버리지 않는다. 포즈·landmark·관측값 모두 그래프에 살아 있고, 루프 클로저는 새 엣지를 추가하는 것으로 표현된다. 재최적화가 전체 궤적을 일관성 있게 조정한다. 이미 지나간 포즈도 수정 대상이 된다는 점이 필터와의 본질적 차이다.

계산 비용도 달랐다. EKF의 갱신 비용은 $O(N^2)$ (landmark 수 $N$에 대해), 정보 저장은 $O(N^2)$다. 그래프 방법은 희소 Cholesky(또는 QR) 분해를 활용하면 복잡도가 크게 줄어든다. 로봇이 제한된 지역 내에서 움직이는 현실 시나리오—희소 연결 그래프—에서 일반적으로 $O(N \log N)$ 수준의 갱신이 가능하다. 대규모 장기 SLAM에서 이 간격은 좁히기 어렵다.

> 📜 **예언 vs 실제.** Dellaert의 Square Root SAM(2006)이 제시한 배치 방식의 한계는 같은 그룹에서 곧바로 증분화 방향으로 이어졌다. 2008년 iSAM이 Givens rotation 기반 증분 갱신으로 이를 다뤘고, 2012년 iSAM2는 Bayes tree로 루프 클로저 상황의 효율성까지 끌어올렸다. GTSAM·Ceres·g2o 모두 같은 구조 위에서 경쟁한다. 세 논문이 동일한 문제 의식을 단계적으로 해소하는 계보를 이뤘다는 점에서, 이 라인은 거의 예고된 궤적대로 실현된 편에 가깝다. `[적중]`

마지널리제이션(marginalization)의 유연성도 한몫했다. 그래프에서 오래된 포즈를 marginalize할 때 그 정보가 남은 변수들에 연결 factor로 보존된다. 필터는 정보를 버렸지만, 그래프는 압축하면서도 정보를 지킬 수 있다. 슬라이딩 윈도우 최적화나 keyframe 선택 같은 공학적 트레이드오프가 여기서 등장한다.

---

## 6.7 비선형성과 강건성: 실무 엔지니어링의 층위

그래프 최적화의 이론적 우아함과 실제 구현 사이에는 간격이 있다. 그 간격을 메우는 작업이 2010년대 SLAM 엔지니어링의 상당 부분을 차지했다.

첫 번째 문제는 초기값 의존성이다. 가우스-뉴턴이나 LM 최적화는 초기 포즈 추정이 참값에서 크게 벗어나 있으면 지역 최솟값(local minimum)에 수렴한다. 루프 클로저에서 잘못된 대응 관계가 섞이면 초기값이 훼손된다. 그래서 루프 클로저 검증과 아웃라이어 rejection이 백엔드 이전 단계의 핵심 작업이 됐다.

표준 최소자승은 아웃라이어에 취약하다는 것도 실무에서 금방 드러났다. Huber 비용이나 Cauchy 비용 같은 robust kernel을 쓰면 잘못된 매칭의 영향을 줄일 수 있다. g2o와 GTSAM 모두 robust kernel을 선택 가능하게 한다. 어느 kernel을 쓸지는 환경과 센서 특성에 따라 달라지며, 2026년에도 이 선택은 여전히 엔지니어의 경험에 의존한다.

세 번째 문제는 marginalization 근사다. iSAM2의 Bayes tree는 정확한 증분 추론을 제공하지만, 변수 수가 계속 증가하면 트리가 커진다. 실제 시스템에서는 오래된 포즈를 marginalize하여 트리 크기를 관리한다. 이 marginalization 과정에서 발생하는 fill-in이 information matrix를 조밀하게 만들 수 있다. 어떻게 truncate할지, Prior factor로 어떻게 근사할지가 구현 품질을 가른다.

> 📜 **예언 vs 실제.** g2o가 표방한 "어떤 그래프 최적화 문제든 플러그인으로 처리한다"는 범용성은, 실제로 line·plane 같은 복잡한 기하 제약을 내부적으로 활용하는 시스템(OpenVINS, VINS-Fusion 계열 등)으로 부분적으로 확장됐다. 다만 2026년 기준 g2o 라이브러리 자체는 광범위한 기본 factor 확장보다 인터페이스 안정성과 기존 사용자 호환성 유지에 비중을 두고 있고, 새로운 factor 타입은 사용자 측에서 상속·fork·래핑으로 얹는 방식이 일반적이다. `[진행형]`

---

## 🧭 아직 열린 것

어느 robust kernel을 선택해야 하는가. Huber, Cauchy, Geman-McClure, DCS 등 여러 선택지가 있지만, 주어진 환경과 센서에 어느 kernel이 최적인지를 사전에 결정하는 원칙적인 방법이 없다. 이 선택은 여전히 엔지니어의 직관과 경험에 의존한다. 학습 기반으로 cost function 자체를 최적화하는 연구가 있으나, 온라인 증분 시스템에 통합하는 것은 풀리지 않은 문제다.

비가우시안 상황을 factor graph 안에서 표현하는 것은 아직 열려 있다. 현재 GTSAM·g2o의 factor는 거의 모두 가우시안 노이즈를 가정한다. 루프 클로저의 오매칭 확률, 다중 가설 포즈 같은 상황을 정확하게 표현하는 것은 이론적으로도 계산적으로도 어렵다. 맥스 믹스처(max-mixture) 모델 등의 시도가 있지만 범용 솔루션은 없다.

Bayes tree는 루프 클로저 수가 적을 때 효율적이다. 수십 킬로미터를 수시간 주행하며 수천 번의 루프 클로저가 발생하는 시나리오에서는 트리 구조가 복잡해지고 메모리 효율이 떨어진다. GTSAM의 실제 자율주행 데이터 적용에서 이 병목이 보고되어 있으며, 계층적 트리 관리나 서브맵 분할과의 결합이 현재 연구 방향 중 하나다.

---

2010년대 들어 백엔드 논쟁은 잦아들었다. g2o와 GTSAM이 실질적 표준이 되면서, 연구자들의 관심은 백엔드 위에 무엇을 얹느냐로 옮겨갔다. "어떻게 루프를 닫는가"보다 "어떤 feature로, 얼마나 멀리서 루프를 인식하는가"가 물음이 되었다. 프론트엔드가 새 경쟁 무대였다.
