# Ch.6 — Graph SLAM

1997년 캐나다 York University의 Feng Lu와 Evangelos Milios는 레이저 스캔을 하나씩 누적 지도에 붙여 나가면 정합 오차가 쌓여 지도가 서로 맞지 않게 된다고 지적했다. 두 사람은 스캔마다 local frame(스캔을 찍은 자리의 좌표계)을 남기고, frame 사이의 상대 위치를 모두 보관했다. 그런 다음 이 제약들을 한꺼번에 풀어 전체 pose를 맞췄다. 두 사람의 1997년 논문은 레이저 스캔 정합과 배치 최대우도 추정을 하나의 절차로 묶어 보였다. 이 방향을 연 사람은 두 사람 말고도 있다. 그보다 10여 년 앞서 LAAS의 [Chatila와 Laumond(1985)](https://www.semanticscholar.org/paper/Position-referencing-and-consistent-world-modeling-Chatila-Laumond/c34a678e40a7d80cb3683f07fc837179fd9bf3ee)가 이동 로봇의 참조 좌표계와 일관된 월드 모델을 논의했다. 1999년에는 [Gutmann과 Konolige](https://www.semanticscholar.org/paper/Incremental-mapping-of-large-cyclic-environments-Gutmann-Konolige/3c1bda51b8ca59f1836ed1b96c485d905804989a)가 큰 순환 환경을 증분 방식으로 지도화하면서 자세 정합을 썼다. 2000년대 초에는 Thrun 그룹이 이 접근을 *full SLAM* 문제로 정식화했고, [Folkesson과 Christensen(2004)](http://www.hichristensen.net/hic-papers/folkesson-icra2004.pdf), Konolige, Dellaert도 저마다 정식화를 내놓았다. 이 연구들을 거치며 SLAM은 전체 궤적을 한꺼번에 추정하는 문제가 됐다.

2000년대의 EKF-SLAM은 landmark가 늘수록 $O(N^2)$ 공분산 갱신에 막혔다. PTAM(2007)은 tracking과 mapping을 나누고 keyframe에 BA를 돌려 실시간 최적화를 해 보였다. 필터와 함께 발전한 graph 기반 smoothing은 여러 연구 집단을 거쳐 SLAM backend의 주요 방법이 됐다.

---

## 6.1 레이저 스캔에서 포즈 그래프로: Lu-Milios 1997

[Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping"](https://doi.org/10.1023/A:1008854305733)이 나오기 전에는 연속한 레이저 스캔을 ICP(Iterative Closest Point) 같은 국소 정합으로 이어 붙이는 일이 많았다. ICP는 두 스캔을 가까이에서 잘 맞춘다. 그러나 drift(누적 오차)가 쌓이면 수십 미터만 가도 지도가 뒤틀리고, 루프를 돌아 출발점에 오면 지도가 어긋난다.

로봇의 포즈 $x_1, x_2, \ldots, x_n$을 노드로, 포즈 쌍 사이의 상대 측정값을 엣지로 놓으면 지도 작성은 그래프의 비용을 최소화하는 문제가 된다. 엣지마다 두 포즈 사이의 상대변환 $\hat{z}_{ij}$와 그 불확실성 $\Omega_{ij}$가 붙는다. 전체 비용 함수는

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

이다. $h(x_i, x_j)$는 두 포즈에서 기대되는 상대변환을 계산하는 함수이고, $z_{ij}$는 실제로 잰 상대변환이다. $\Omega_{ij} = \Sigma_{ij}^{-1}$는 측정 공분산의 역행렬로, 정보 행렬이라고 부른다.

loop closure(전에 왔던 곳을 알아보고 궤적을 닫는 일)도 같은 식으로 다룬다. 같은 장소를 다시 찾았을 때 얻은 상대 측정값을 엣지로 더하면, 최적화가 그 제약에 맞춰 모든 포즈를 고친다. EKF에서 loop closure는 공분산 전체를 $O(N^2)$ 비용으로 갱신하는 무거운 작업이었다. 포즈 그래프에서는 제약 하나가 엣지 하나다. 다만 그 제약을 추정에 반영하려면 그래프를 다시 최적화해야 한다.

> 🔗 **차용.** 포즈 그래프 최적화는 비선형 최소자승 문제다. 이런 문제는 [Levenberg(1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/)와 [Marquardt(1963)](https://doi.org/10.1137/0111030)가 비선형 파라미터 추정용으로 만든 방법으로 풀어 왔다. 수십 년 된 수치 최적화 방법이 레이저 지도 작성의 backend에 쓰였다.

Lu와 Milios는 모든 포즈를 한 번에 푸는 배치(batch) 선형 시스템을 썼다. 스캔이 늘면 선형 시스템도 커져서 큰 환경에 바로 쓰기는 어려웠다. 그래도 최적화로 지도 전체를 일관되게 맞출 수 있음을 보였다. 그 뒤 Gutmann과 Konolige는 증분 계산을, Folkesson과 Christensen은 잘못된 data association(관측과 landmark의 대응)에 강한 방법을, Thrun 그룹은 큰 실제 환경에 적용하는 일을 중점으로 다뤘다.

---

## 6.2 정보 행렬의 희소성

Lu와 Milios의 논문이 나온 뒤 여러 그룹이 이 방법을 넓혔다. 이들이 공통으로 주목한 것은 정보 행렬(information matrix, $\Omega = \Sigma^{-1}$)의 **희소성(sparsity)**이었다.

EKF-SLAM의 공분산 행렬 $\Sigma$는 조밀(dense)하다. 로봇이 landmark를 하나 관측할 때마다 다른 모든 landmark와의 상관관계가 갱신된다. 2D landmark가 $n$개 있으면 공분산의 landmark 부분만 해도 $2n \times 2n$ 행렬이고, 갱신 비용은 $O(n^2)$다. landmark가 늘수록 실시간으로 돌리기 어려워진다.

포즈 그래프의 정보 행렬은 희소하다. 포즈 $x_i$와 $x_j$가 직접 측정으로 이어져 있을 때만 $\Omega$의 $(i,j)$ 블록이 0이 아니다. 로봇이 계속 움직이는 동안에는 이웃한 포즈끼리만 엣지로 이어지므로 $\Omega$는 띠 모양(banded) 희소 행렬이 된다. loop closure 없이 주행만 하면 이 행렬은 블록 삼중대각(block tridiagonal)이다.

Sebastian Thrun 그룹의 [Sparse Extended Information Filter(SEIF)](http://www.cs.cmu.edu/~thrun/papers/thrun.tr-seif02.pdf)는 이 희소성을 직접 이용해 계산 비용을 $O(n^2)$보다 크게 줄였다. 실제 비용은 그래프 구조에 따라 다르다(§6.6).

> 🔗 **차용.** [Eustice 등의 exactly sparse delayed-state filter](https://web.mit.edu/2.166/www/handouts/eustice_et_al_ieeetro_2006.pdf)도 필터에서 정보 행렬의 희소성을 이용할 수 있음을 보였다. 희소성을 이용한다는 생각은 Dellaert의 factor graph 정식화와 Bayes tree로 이어졌다.

2006년 ICRA에서 [Olson, Leonard, Teller](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf)는 stochastic gradient descent의 변형으로 포즈 그래프를 최적화하는 방법을 발표했다. 오도메트리에서 온 초기값이 크게 틀려도 궤적을 빨리 되찾는 것이 목표였다. 논문은 포즈 3,500개와 제약 5,600개로 된 합성 문제, 포즈 약 1,900개인 Killian Court 실측 데이터로 이 방법을 시험했다. Olson의 구현 코드는 그 뒤 널리 퍼졌다.

---

## 6.3 Factor Graph와 Square Root SAM

2006년 Dellaert와 당시 박사과정 학생이던 Kaess는 [Square Root SAM](https://doi.org/10.1177/0278364906072768)을 IJRR에 발표해 SLAM backend를 factor graph로 정식화했다. Dellaert는 Georgia Tech에서 probabilistic graphical model(확률 그래프 모델)을 연구해 왔다. Square Root SAM은 SLAM을 Bayes 추론 문제로 쓰고, 그 추론을 factor graph에서 풀었다.

**Factor graph**는 변수 노드와 factor 노드를 엣지로 이은 이분 그래프다. 변수 노드는 로봇 포즈와 landmark 위치이고, factor 노드는 관측값이나 사전 확률(prior)이다. Factor $f_k(x_{i_1}, x_{i_2}, \ldots)$는 연결된 변수들 사이의 확률적 제약을 나타낸다. 전체 결합 확률은

$$p(X) \propto \prod_k f_k(X_k)$$

이며, MAP 추정은 이 확률을 최대로 만드는 $X^*$를 찾는다. factor가 Gaussian이면 이 문제는 비선형 최소자승 문제가 된다.

Jacobian 행렬 $J$를 QR 분해하면 상삼각(upper triangular) 행렬 $R$이 남는다. $R^T R = J^T J = \Omega$이며, 이 $R$을 "square root information matrix"라고 부른다. $R$이 얼마나 희소한지는 변수 제거(variable elimination) 순서와 factor graph의 연결 구조가 정한다. AMD나 COLAMD 같은 순서 정하기 방법으로 fill-in(분해 중에 새로 생기는 0이 아닌 항)을 줄이면 희소한 $R$을 얻는다.

이 방식은 EKF의 공분산 갱신보다 수치적으로 안정하다. 지도 전체의 landmark와 포즈를 함께 최적화한다.

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM과 iSAM2: 온라인 증분 추론

Square Root SAM은 배치(batch) 방법이었다. 새 관측이 들어올 때마다 $J^T J$ 전체를 다시 분해해야 한다. 조밀 행렬이면 비용이 $O(n^3)$이고, 희소 행렬이면 연결 구조와 소거 순서에 따라 달라진다. 어느 쪽이든 온라인 로봇 시스템에 쓰기는 어려웠다.

2008년 Kaess, Ranganathan, Dellaert가 IEEE TRO에 발표한 [**iSAM**(incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)은 이 문제를 Givens rotation으로 풀었다. 변수와 factor가 새로 들어오면 QR 분해를 처음부터 다시 하지 않는다. 새 행만 붙이고 Givens rotation으로 $R$을 고친다.

iSAM에는 주기적으로 배치 계산을 다시 해야 하는 약점이 있었다. 비선형 factor를 선형화해서 만든 $R$은 현재 추정값 근처의 1차 근사일 뿐이다. 로봇이 움직이며 추정값이 선형화 지점에서 멀어지면 근사 오차가 커진다. 루프가 있는 궤적에서는 $R$에 fill-in이 생겨 희소 구조도 무너진다. iSAM은 두 문제를 함께 다뤘다. 논문의 실험에서는 100스텝마다 변수 순서를 다시 정하고, 그때 측정 함수도 다시 선형화해 $R$을 처음부터 다시 분해했다.

2012년 Kaess 등이 IJRR에 발표한 [iSAM2](https://doi.org/10.1177/0278364911430419)는 Bayes tree라는 자료구조로 이 주기적 배치 계산을 없앴다. factor graph에 variable elimination을 적용하면 chordal Bayes net을 얻는데, Bayes tree는 이 Bayes net에서 만드는 트리다. Bayes net의 clique가 노드이고, clique끼리 공유하는 변수(separator)가 엣지다. factor가 새로 들어오면 Bayes tree에서 영향을 받는 clique를 찾고, 그 부분만 factor graph로 되돌려 다시 선형화하고 다시 푼다. **Fluid relinearization**은 추정값이 선형화 지점에서 임계값 $\beta$보다 많이 움직인 변수만 골라 다시 선형화하고, 그 변수가 걸린 clique만 다시 계산한다. loop closure가 생겨도 영향을 받는 clique가 일부에 그치는 경우가 많아 전체를 다시 계산하지 않아도 됐다.

> 🔗 **차용.** Bayes tree의 바탕은 확률 그래프 모델 문헌의 junction tree(join tree) 알고리즘이다. Koller와 Friedman의 [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/) 같은 표준 교과서가 다루는 소거 순서와 chordal 그래프 기반 추론이 여기에 해당한다. iSAM2는 인공지능 분야의 이 추론 기법을 실시간 로봇 SLAM에 가져왔다.

iSAM2는 [GTSAM(Georgia Tech Smoothing and Mapping)](https://gtsam.org) 라이브러리로 공개됐다. C++로 짰고 Python 바인딩이 있다. Dellaert는 2015년 이후 Skydio, Facebook, Google AI, Verdant Robotics에서도 일했고, 그동안에도 GTSAM 개발은 이어졌다. GTSAM의 공개 문서와 사례에는 자율주행, 드론, 로봇팔 보정 같은 응용이 들어 있다.

---

## 6.5 g2o: ROS 생태계의 범용 그래프 최적화기

Freiburg 대학의 Rainer Kümmerle, Giorgio Grisetti, Wolfram Burgard와 Imperial College London의 Hauke Strasdat, Willow Garage의 Kurt Konolige는 2011년 ICRA에서 [g2o](https://doi.org/10.1109/ICRA.2011.5979949)(general graph optimization)를 발표했다. g2o는 여러 종류의 그래프 최적화 문제를 플러그인 방식으로 풀도록 만든 오픈소스 구현이다.

g2o는 vertex, edge, solver를 따로 정의한다. vertex(변수 노드)와 edge(factor 또는 제약)가 그래프를 이루고, solver가 희소 선형 시스템을 푼다. 사용자가 vertex 타입과 edge의 오차 함수, Jacobian을 정의하면 g2o가 Gauss-Newton이나 Levenberg-Marquardt로 전체를 최적화한다. 희소 solver는 CHOLMOD, CSparse, Eigen 가운데 고르거나 외부 라이브러리로 바꿀 수 있다.

2010년대 초 ROS(Robot Operating System)가 모바일 로봇 연구에 널리 퍼지면서 g2o도 그래프 기반 SLAM의 대표 구현 가운데 하나가 됐다. ORB-SLAM과 LSD-SLAM이 g2o를 썼다. 같은 시기 ROS에서 널리 쓰인 gmapping은 particle filter 계열이고, Cartographer는 Ceres를 쓴다.

---

## 6.6 그래프 방법이 주류가 된 이유

필터와 그래프 방법은 문제를 세우는 방식부터 달랐다. EKF-SLAM은 현재 상태의 최적 추정값과 불확실성을 유지하면서 과거를 marginalize한다. 과거 포즈는 상태에서 지워지고, 누적 오차는 현재 추정값에 섞여 남는다. loop closure를 반영하려면 현재 공분산을 크게 갱신해야 한다.

그래프 SLAM은 과거 포즈를 상태에 남겨 둔다. 포즈, landmark, 관측값이 모두 그래프에 남는다. loop closure 엣지를 더해 다시 최적화하면 전체 궤적이 서로 맞게 고쳐지므로 이미 지나간 포즈도 수정할 수 있다. 이산 keyframe 대신 시간에 따라 연속인 궤적으로 그래프를 다시 세우는 방법은 Ch.7c(Continuous-Time SLAM)에서 다룬다.

계산 비용도 달랐다. landmark 수가 $N$이면 EKF의 갱신 비용과 저장 공간은 모두 $O(N^2)$다. 그래프 방법은 희소 Cholesky나 QR 분해를 쓰면 계산량이 크게 준다. 다만 갱신 비용은 그래프 연결, 분해 중의 fill-in, 소거 순서, 다시 계산할 영역에 달려 있다. 좁은 공간을 움직인다는 조건만으로 $O(N \log N)$을 보장할 수는 없다.

marginalization(변수를 적분해 없애고 그 정보를 남은 변수에 넘기는 일)도 그래프에서는 다루기 쉬웠다. 오래된 포즈를 marginalize하면 그 정보는 남은 변수들을 잇는 factor로 남는다. 필터도 과거 상태를 지우면서 그 정보를 현재 추정에 넘긴다. 어느 쪽이든 정보를 압축할 때 생기는 근사와 재선형화 제약을 따져야 한다. sliding window 최적화나 keyframe 선택 같은 공학적 절충은 여기서 나온다.

---

## 6.7 비선형성과 강건성: 실제 구현의 문제들

이론으로 정리된 그래프 최적화를 실제 시스템에서 돌리려면 손볼 곳이 많았다.

먼저 초기값이다. Gauss-Newton이나 LM 최적화는 초기 포즈 추정이 참값에서 크게 벗어나 있으면 지역 최솟값(local minimum)에 수렴한다. loop closure에 잘못된 대응이 섞이면 해가 엉뚱한 곳으로 간다. 그래서 backend에 넘기기 전에 loop closure를 검증하고 outlier를 걸러 내는 일이 중요해졌다. 볼록 완화(SDP)로 지역 최솟값을 피하고 해가 전역 최적임을 증명하는 연구는 Ch.6b(Certifiable SLAM)에서 다룬다.

표준 최소자승이 outlier에 약하다는 것도 금방 드러났다. Huber 비용이나 Cauchy 비용 같은 robust kernel을 쓰면 잘못된 매칭의 영향을 줄일 수 있다. g2o와 GTSAM 모두 robust kernel을 고를 수 있다. 어느 kernel을 쓸지는 환경과 센서 특성을 보고 정한다.

marginalization 근사도 문제다. iSAM2의 Bayes tree는 증분 추론을 정확하게 하지만, 변수가 계속 늘면 트리도 커진다. 실제 시스템은 오래된 포즈를 marginalize해서 트리 크기를 관리한다. marginalize할 때 생기는 fill-in은 정보 행렬을 조밀하게 만든다. 어디서 잘라 낼지, prior factor로 어떻게 근사할지에 따라 구현 품질이 달라진다.

> 📜 **예언 vs 실제.** g2o가 내세운 범용성은 사용자가 상태를 vertex로, 관측 제약을 edge로 정의하면 같은 최적화 틀에서 풀 수 있다는 뜻이었다. 이후 연구들은 line, plane, 관성, 객체 제약을 각자의 시스템에 맞는 사용자 정의 edge로 구현했다.

---

## 🧭 아직 열린 것

어느 robust kernel을 골라야 하는가. Huber, Cauchy, Geman-McClure, DCS 같은 선택지가 있다. Chebrolu 등은 kernel을 보통 사람이 골라 손으로 조정한다고 지적했다. 이들의 [adaptive robust kernel](https://doi.org/10.1109/LRA.2021.3061331)(RA-L 2021)은 Barron의 일반화 kernel을 써서, 최적화 도중 잔차 분포에 맞춰 kernel 모양을 정했다. 논문은 이 방법을 ICP와 bundle adjustment에서 시험했다.

Gaussian이 아닌 분포를 factor graph에서 어떻게 나타낼지도 아직 풀리지 않았다. GTSAM과 g2o의 기본 최적화는 residual이 Gaussian이라고 가정한다. robust kernel, max-mixture, hybrid factor 같은 확장이 이 가정을 넘어 loop closure가 잘못 매칭됐을 확률이나 여러 포즈 가설을 다루려 한다.

Bayes tree의 증분 효율은 새 factor가 건드리는 clique가 일부에 그칠 때 가장 크다. 대규모 지도에서 loop closure 제약이 조밀해지면 영향을 받는 subtree와 fill-in이 커져 계산량과 메모리가 늘 수 있다. 지도를 계층으로 관리하거나 submap으로 나눠 이 증가를 억제하려는 연구가 있다.

---

2010년대 들어 backend를 둘러싼 논쟁은 잦아들었다. g2o와 GTSAM이 널리 쓰이자 연구자들은 frontend로 관심을 옮겼다. 어떤 feature를 쓰고, 루프를 얼마나 멀리서 알아보느냐가 새 질문이 됐다.

그래도 g2o와 GTSAM이 내놓은 해가 정말 전역 최솟값인지는 여전히 물을 수 있었다. Ch.6b는 이 certifiability(해가 전역 최적임을 증명하는 문제)를 다룬다.
