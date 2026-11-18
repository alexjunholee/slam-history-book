# 6장 그래프 SLAM

로봇이 복도를 따라 전진하며 레이저 스캔을 한 장씩 기존 지도에 갖다 붙이는(incremental registration) 방식에는 치명적인 약점이 있었다. 1997년 캐나다 York University의 Feng Lu와 Evangelos Milios가 정확히 짚어냈듯, 한 걸음마다 생기는 미세한 정합 오차가 지도에 그대로 굳어 버리면 몇 바퀴만 돌아도 지도가 뒤틀리고 출발점과 끝점이 어긋나기 때문이다. 두 사람은 스캔을 즉시 전역 지도에 박제하는 대신, 스캔마다 고유한 local frame을 남기고 프레임 사이의 상대적 제약 조건을 전부 보관한 뒤, 이 거대한 제약 연립방정식을 한꺼번에 풀어 전체 자세를 맞추는 해법을 제시했다. 레이저 스캔 정합을 배치 최대우도 추정과 결합한 이 1997년 논문은 훗날 **포즈 그래프 최적화(pose graph optimization)**라 불리는 패러다임의 깃발을 꽂았다.

물론 이 길의 문을 연 것이 Lu와 Milios만은 아니었다. 그보다 십여 년 앞선 1985년 LAAS의 [Chatila와 Laumond](https://www.semanticscholar.org/paper/Position-referencing-and-consistent-world-modeling-Chatila-Laumond/c34a678e40a7d80cb3683f07fc837179fd9bf3ee)가 이동 로봇의 참조 좌표계와 일관된 세계 모델링의 필요성을 일찍이 논의했고, 1999년에는 [Gutmann과 Konolige](https://www.semanticscholar.org/paper/Incremental-mapping-of-large-cyclic-environments-Gutmann-Konolige/3c1bda51b8ca59f1836ed1b96c485d905804989a)가 거대한 순환 환경을 증분 방식으로 매핑하며 자세 정합을 실증했다. 2000년대 초에는 Sebastian Thrun 그룹이 이를 *full SLAM* 문제로 정식화했고, [Folkesson과 Christensen(2004)](http://www.hichristensen.net/hic-papers/folkesson-icra2004.pdf), Konolige, Frank Dellaert 등이 저마다의 수학적 언어로 살을 붙였다. 2000년대 내내 EKF-SLAM이 $O(N^2)$ 공분산 행렬의 무게에 짓눌려 신음하는 동안(→ 4장), 과거의 모든 자세를 버리지 않고 그래프로 엮어 한꺼번에 다듬는 이 smoothing 철학은 조용히 세력을 키워 2010년대 SLAM backend의 왕좌를 차지했다.

---

## 6.1 레이저 스캔에서 포즈 그래프로: Lu-Milios 1997

[Lu & Milios (1997), "Globally Consistent Range Scan Alignment for Environment Mapping"](https://doi.org/10.1023/A:1008854305733) 이전의 레이저 지도 작성은 대개 인접한 두 스캔을 ICP(Iterative Closest Point) 같은 국소 정합으로 차례차례 이어 붙이는 데 머물렀다. ICP는 코앞의 두 스캔을 맞추는 데는 능숙했지만, 한 번 붙인 스캔의 오차를 되돌릴 수 없기에 수십 미터만 주행해도 누적 오차(drift)가 쌓이고, 루프를 돌아 처음 위치로 복귀하면 벽이 두 겹으로 갈라지곤 했다.

Lu와 Milios는 시야를 뒤집어 지도 작성을 그래프 위의 최적화 문제로 번역했다. 로봇이 스캔을 찍은 각 자세 $x_1, x_2, \ldots, x_n$을 그래프의 **노드**로 세우고, 두 자세 사이에서 얻은 상대 위치 측정값 $\hat{z}_{ij}$와 그 측정의 확신도(정보 행렬 $\Omega_{ij} = \Sigma_{ij}^{-1}$)를 **엣지**로 연결한 것이다. 그러면 일관된 지도를 찾는 일은 그래프 위 모든 엣지의 불일치 비용을 최소화하는 문제가 된다.

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

여기서 $h(x_i, x_j)$는 현재 추정된 두 자세로부터 계산한 예측 상대변환이고, $z_{ij}$는 실제 센서(오도메트리나 스캔 매칭)로 관측한 상대변환이다.

이 그래프 표현의 진가는 **loop closure** 순간에 빛을 발했다. 로봇이 예전에 지나온 장소를 다시 알아봤을 때, 해야 할 일은 그 두 자세 사이에 새로운 엣지 하나를 덧붙이는 것뿐이다. 그런 뒤 전체 그래프를 다시 최적화하면 loop closure 제약이 엣지를 타고 흐르며 루프 위의 모든 과거 자세를 한꺼번에 바로잡는다. 매 스텝 거대한 공분산 행렬 전체를 $O(N^2)$으로 갱신해야 했던 EKF와는 구조 자체가 달랐다. 다만 Lu와 Milios의 1997년 원형은 모든 자세를 매번 처음부터 한꺼번에 푸는 배치(batch) 선형 시스템이었기에, 스캔 수가 늘어나면 계산이 무거워져 큰 환경에 곧장 쓰기는 버거웠다. 하지만 최적화로 지도 전체의 전역 일관성을 세울 수 있음을 보인 이 한 발이, 이후 Gutmann·Konolige의 증분 계산, Folkesson·Christensen의 강건한 data association, Thrun 그룹의 대규모 실외 매핑으로 이어지는 거대한 흐름을 열었다.

> 🔗 **차용.** 포즈 그래프 최적화는 수학적으로 비선형 최소자승 문제다. [Levenberg(1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/)와 [Marquardt(1963)](https://doi.org/10.1137/0111030)가 반세기 전 비선형 파라미터 추정을 위해 빚어 둔 댐핑 최소자승 해법이(→ §1.3), 레이저 로봇 지도의 backend 심장으로 고스란히 이식됐다.

---

## 6.2 정보 행렬의 희소성

Lu와 Milios의 씨앗이 거목으로 자라날 수 있었던 핵심 비밀은 후속 연구자들이 포착한 **정보 행렬(information matrix, $\Omega = \Sigma^{-1}$)의 희소성(sparsity)**에 있었다.

EKF-SLAM이 쥐고 있던 공분산 행렬 $\Sigma$는 빽빽하게 채워진 조밀(dense) 행렬이다. 로봇이 landmark 하나만 관측해도 그 오차가 로봇을 매개로 지도의 다른 모든 landmark와 얽히기 때문에, 비대각 항이 전부 0이 아닌 값으로 차오른다. 2D landmark가 $n$개일 때 공분산의 landmark 블록만 $2n \times 2n$ 크기이므로, 한 번의 갱신에 $O(n^2)$ 비용을 치러야 하는 굴레에서 벗어날 수 없었다.

놀랍게도 공분산 행렬의 역행렬인 정보 행렬 $\Omega$는 풍경이 완전히 달랐다. 과거 자세를 지우지 않고 남겨 둔 포즈 그래프에서는, 두 자세 $x_i$와 $x_j$ 사이에 직접적인 관측 엣지가 있을 때만 $\Omega$의 $(i,j)$ 블록이 0이 아닌 값을 갖는다. 로봇이 루프를 닫지 않고 전진만 할 때는 바로 이웃한 자세끼리만 연결되므로, $\Omega$는 대각선 주변만 채워진 띠 모양(banded), 더 정확히는 블록 삼중대각(block tridiagonal) 희소 행렬이 된다. 전체 원소의 절대다수가 깨끗한 0으로 비어 있는 것이다.

이 희소성을 파고든 시도는 필터 진영과 최적화 진영 양쪽에서 터져 나왔다. Sebastian Thrun 그룹의 [Sparse Extended Information Filter(SEIF)](http://www.cs.cmu.edu/~thrun/papers/thrun.tr-seif02.pdf)는 정보 행렬의 약한 연결을 인위적으로 잘라 내 계산 비용을 $O(n^2)$ 아래로 끌어내렸고(다만 정확한 비용과 일관성은 그래프 구조에 좌우된다, §6.6), 2006년 ICRA에서는 [Olson, Leonard, Teller](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf)가 확률적 경사 하강법(SGD)의 변형으로 포즈 그래프를 푸는 기법을 내놓아 큰 주목을 받았다. 오도메트리 초기값이 심하게 찌그러져 있어도 빠르게 궤적을 펴내는 이 방법은 자세 3,500개·제약 5,600개짜리 합성 데이터와 자세 약 1,900개인 MIT Killian Court 실측 데이터에서 위력을 입증했고, Olson이 공개한 코드는 순식간에 여러 연구실로 퍼져 나갔다.

> 🔗 **차용.** 같은 시기 [Eustice 등의 exactly sparse delayed-state filter](https://web.mit.edu/2.166/www/handouts/eustice_et_al_ieeetro_2006.pdf) 역시 과거 로봇 상태를 보존하면 근사 없이도 정보 행렬이 정확히 희소해진다는 사실을 필터 틀 안에서 증명했다. 이 희소성의 수학적 통찰은 곧 Frank Dellaert의 factor graph 정식화와 Bayes tree로 만개하게 된다.

---

## 6.3 Factor Graph와 Square Root SAM

그래프 SLAM의 수학에 가장 우아하고 범용적인 옷을 입힌 것은 2006년 *IJRR*에 실린 Frank Dellaert와 당시 박사과정 학생 Michael Kaess의 [**Square Root SAM**](https://doi.org/10.1177/0278364906072768)이었다. Georgia Tech에서 확률 그래프 모델(probabilistic graphical model)을 연구하던 Dellaert는 SLAM을 단순한 행렬 방정식이 아니라 그래프 위의 베이즈 추론 문제로 다시 썼고, 그 언어로 **factor graph**를 택했다.

Factor graph는 구해야 할 미지수인 **변수 노드**(로봇 자세와 landmark 위치)와 센서 관측이나 사전 확률을 뜻하는 **factor 노드**를 엣지로 엮은 이분 그래프(bipartite graph)다. 각 factor $f_k(X_k)$는 자신과 맞닿은 소수의 변수들 사이에 걸린 확률적 제약을 표현하며, 전체 시스템의 결합 확률은 모든 factor의 곱으로 떨어진다.

$$p(X) \propto \prod_k f_k(X_k)$$

가장 그럴듯한 지도와 궤적 $X^*$를 찾는 MAP(Maximum A Posteriori) 추정은 이 곱을 최대화하는 일이며, 모든 factor의 오차가 가우시안을 따르면 곧바로 비선형 최소자승 문제와 같아진다.

여기서 Dellaert와 Kaess가 꺼낸 핵심 도구는 $J^T J$를 직접 구하지 않고 Jacobian 행렬 $J$ 자체를 **QR 분해**해 상삼각 행렬 $R$을 얻는 방식이었다. $R^T R = J^T J = \Omega$를 만족하는 이 행렬 $R$을 논문은 **square root information matrix**라 불렀다. $R$이 얼마나 비어 있는가(희소성)는 변수를 어떤 순서로 소거하는지(variable elimination ordering)와 그래프의 연결 위상에 달려 있는데, AMD나 COLAMD 같은 그래프 기반 순서 정렬 알고리즘을 쓰면 분해 과정에서 0이던 자리가 채워지는 현상(fill-in)을 극적으로 억제할 수 있었다. 게다가 $J^T J$를 제곱해 만들지 않고 $J$에서 곧장 $R$을 뽑아내니 조건수(condition number)가 나빠지지 않아, EKF의 공분산 갱신보다 수치적으로도 훨씬 단단했다.

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM과 iSAM2: 온라인 증분 추론

Square Root SAM은 그래프 SLAM의 아름다운 표준형을 세웠지만, 결정적인 약점이 하나 있었다. 본질적으로 **배치(batch)** 알고리즘이었다는 점이다. 새 센서 관측이 하나 들어올 때마다 전체 Jacobian $J$를 다시 쌓고 QR 분해를 처음부터 새로 돌려야 했다(조밀 행렬 기준 $O(n^3)$, 희소 행렬이라도 연결 구조에 따라 무거워진다). 움직이는 로봇 위에서 매 프레임 답을 내놓기에는 벅찬 구조였다.

Kaess, Ranganathan, Dellaert는 2008년 *IEEE TRO*에 발표한 [**iSAM**(incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)에서 이 병목을 선형대수의 고전 기법인 **Givens rotation**으로 풀었다. 새 변수와 factor가 들어오면 기존 상삼각 행렬 $R$을 버리지 않고 새 행만 아래에 붙인 뒤, Givens rotation으로 대각선 아래의 원소만 국소적으로 지워 $R$의 상삼각 형태를 증분(incremental) 방식으로 복구한 것이다.

하지만 iSAM에도 어쩔 수 없는 타협이 남아 있었다. 비선형 factor를 선형화해 얻은 행렬 $R$은 특정 시점의 추정치 주변에서만 유효한 1차 근사였기에, 로봇이 움직이며 추정값이 처음 선형화 지점에서 멀어지면 근사 오차가 누적됐다. 게다가 루프가 여러 개 얽히면 $R$ 내부에 fill-in이 야금야금 번져 희소성이 무너졌다. 그래서 iSAM은 실험에서 100스텝마다 변수 순서를 다시 정하고 전체 관측을 다시 선형화해 $R$을 처음부터 새로 분해하는 주기적 배치 재계산(periodic batch reordering & relinearization)을 치러야 했다.

이 마지막 주기적 리셋마저 완전히 지워 낸 결정판이 2012년 *IJRR*의 [**iSAM2**](https://doi.org/10.1177/0278364911430419)다. Kaess와 동료들은 행렬 $R$ 대신 **Bayes tree**라는 새로운 자료구조를 고안했다. factor graph에 변수 소거를 적용해 얻는 chordal Bayes net의 clique들을 노드로, clique 사이에 공유되는 변수(separator)를 엣지로 삼아 트리 구조를 만든 것이다. 이제 새 factor가 추가되면 Bayes tree에서 그 factor의 영향을 받는 일부 clique만 찾아내 잠깐 factor graph로 되돌려 풀고 트리에 다시 꽂으면 끝난다. 여기에 **fluid relinearization**을 얹어, 전체 변수를 주기적으로 다 뜯어고치는 대신 추정치가 선형화 지점에서 임계값 $\beta$ 이상 움직인 변수만 콕 집어 그 변수가 걸친 clique만 국소적으로 재선형화했다. 덕분에 거대한 loop closure가 터져도 실제 영향권에 드는 clique만 부분적으로 갱신하면 되므로, 진정한 의미의 상시 증분 추론이 완성됐다.

이 엔진은 [**GTSAM**(Georgia Tech Smoothing and Mapping)](https://gtsam.org)이라는 C++ 오픈소스 라이브러리(Python 바인딩 포함)로 세상에 나왔다. Dellaert가 2015년 이후 Skydio, Facebook, Google AI, Verdant Robotics 등을 거치며 산업계와 학계를 넘나드는 동안에도 GTSAM은 꾸준히 진화해, 오늘날 자율주행과 드론, 로봇팔 캘리브레이션에 이르기까지 광범위한 현장의 주력 backend로 쓰이고 있다.

> 🔗 **차용.** Bayes tree의 뿌리는 확률 그래프 모델 문헌의 **junction tree(join tree)** 알고리즘이다. Daphne Koller와 Nir Friedman의 교과서 [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/)가 체계화한 변수 소거 순서와 chordal 그래프 위 추론 이론을 실시간 로봇 SLAM의 증분 최적화 엔진으로 완벽히 이식한 것이 iSAM2다.

---

## 6.5 g2o: ROS 생태계의 범용 그래프 최적화기

미국에서 Dellaert와 Kaess가 GTSAM을 다듬던 같은 시기, 유럽과 캘리포니아의 또 다른 연합군은 조금 다른 실용적 철학으로 그래프 최적화의 표준 도구를 빚어냈다. 2011년 ICRA에서 Freiburg 대학의 Rainer Kümmerle, Giorgio Grisetti, Wolfram Burgard, Imperial College London의 Hauke Strasdat, 그리고 Willow Garage의 Kurt Konolige가 함께 발표한 [**g2o**(general graph optimization)](https://doi.org/10.1109/ICRA.2011.5979949)다.

g2o의 목표는 이름처럼 "범용성"이었다. 2D 포즈 그래프든, 3D landmark bundle adjustment든, Sim(3) 스케일 드리프트 보정이든, 사용자가 그래프의 노드(`vertex`)와 제약(`edge`), 그리고 오차 함수와 Jacobian만 플러그인처럼 정의해 끼워 넣으면, 내부 엔진이 Gauss-Newton이나 Levenberg-Marquardt로 희소 선형 시스템을 풀어 주는 유연한 프레임워크를 지향했다. 실제 선형 대수 계산도 CHOLMOD, CSparse, Eigen 등 검증된 희소 solver 가운데 입맛에 맞게 바꿔 끼울 수 있게 설계했다.

타이밍도 절묘했다. 2010년대 초반 ROS(Robot Operating System)가 전 세계 로봇 연구실의 공통 운영체제로 번져 나가던 시점에, g2o는 곧바로 ROS 기반 SLAM의 대표 backend로 안착했다. 이후 컴퓨터 비전 SLAM의 양대 산맥이 된 ORB-SLAM과 LSD-SLAM이 모두 g2o를 심장으로 택했다(같은 시기 ROS에서 2D 라이다 매핑의 표준이 된 gmapping은 particle filter 계열을, Google의 Cartographer는 Ceres Solver를 썼다).

---

## 6.6 그래프 방법이 주류가 된 이유

필터에서 그래프로의 세대교체는 단순한 유행이 아니라 문제 설정의 근본적 차이에서 비롯됐다.

첫째는 **과거를 대하는 태도**다. EKF-SLAM은 현재 상태만을 유지하기 위해 지나간 로봇 자세들을 매 스텝 주변화(marginalization)로 지워 버린다. 일단 지워진 과거 자세의 선형화 오차는 현재 공분산 속에 영구히 박제돼, 나중에 loop closure가 일어나도 과거 궤적을 펴낼 방법이 마땅치 않다. 반면 그래프 SLAM은 지나온 자세와 관측을 그래프 안에 고스란히 살려 둔다. 덕분에 수백 미터 뒤에서 루프가 닫히더라도 전체 궤적과 과거 자세까지 한꺼번에 되돌려 고칠 수 있다(→ 10장).

둘째는 **계산 구조와 수치 확장성**이다. landmark가 $N$개일 때 EKF는 조밀한 공분산 행렬 때문에 무조건 $O(N^2)$ 비용을 물어야 했다. 그래프 SLAM은 희소 Cholesky나 QR 분해를 통해 실제 연결된 이웃만 건드리므로 계산 부담이 극적으로 줄어든다(물론 정확한 복잡도는 그래프의 위상, 분해 시 발생하는 fill-in, 소거 순서, 재선형화 범위에 따라 달라지므로 좁은 공간을 돈다는 이유만으로 무조건 $O(N \log N)$이 보장되는 것은 아니다).

셋째는 **주변화(marginalization)의 투명성**이다. 변수가 끝없이 늘어나면 그래프 SLAM 역시 오래된 자세를 적분해 없애고 그 정보를 남은 변수들을 잇는 factor로 압축해야 한다. 이때 정보가 엉키면서 생기는 근사와 재선형화 제약을 어느 선에서 타협할지—예컨대 sliding window 크기나 keyframe 선택 전략—엔지니어가 그래프 구조를 보며 훨씬 명시적으로 통제할 수 있었다.

---

## 6.7 비선형성과 강건성: 실제 구현의 문제

물론 수학이 깔끔하다고 실전 시스템이 저절로 돌아가지는 않는다. 이론적 그래프 최적화를 움직이는 로봇에 올리자마자 세 가지 현실의 벽이 나타났다.

가장 먼저 부딪힌 것은 **초기값 의존성**이다. Gauss-Newton과 Levenberg-Marquardt는 본질적으로 국소 최적화기(local optimizer)이므로, 오도메트리에서 넘어온 초기 자세가 참값에서 크게 벗어나 있으면 엉뚱한 지역 최솟값(local minimum)에 빠져 헤어나지 못한다. 여기에 단 하나의 잘못된 loop closure 엣지(outlier)라도 섞여 들어가면, 최소자승의 제곱 오차가 그 가짜 엣지를 맞추려고 멀쩡한 지도 전체를 기괴하게 비틀어 놓는다. 결국 backend에 데이터를 넘기기 전 frontend 단계에서 오매칭을 철저히 걸러 내는 검증 장치가 사활을 걸 문제로 떠올랐다(→ 7장).

두 번째 방어막은 **robust kernel**이었다. 표준 이차 손실이 outlier에 맥없이 무너진다는 사실이 드러나면서, 오차가 일정 수준을 넘어서면 가중치를 깎아 내리는 Huber나 Cauchy 같은 강건 비용 함수가 필수로 자리 잡았다. 오늘날 g2o와 GTSAM 모두 다양한 robust kernel을 기본 내장하고 있으며, 센서 특성과 환경에 맞춰 선택해 쓴다.

세 번째는 **장기 운용 시 주변화 근사**다. iSAM2의 Bayes tree가 아무리 영리해도 로봇이 하루 종일 주행하며 새 자세를 쌓아 가면 트리와 메모리는 결국 커진다. 실전 시스템은 오래된 자세를 주기적으로 주변화해 크기를 묶어 두는데, 변수를 지울 때마다 남은 변수들 사이에 새로운 연결이 빽빽하게 생기는 fill-in이 일어나 정보 행렬의 희소성을 잠식한다. 어디까지 남기고 어떤 prior factor로 근사할지에 따라 장기 주행 시스템의 정확도와 속도가 갈렸다.

> 📜 **예언 vs 실제.** g2o가 내세운 "일반성(generality)"은 사용자가 상태(vertex)와 제약(edge)만 정의하면 어떤 기하 문제든 같은 틀에서 풀 수 있다는 약속이었다. 그 약속은 놀라울 만큼 풍성하게 실현돼, 이후 연구자들은 점과 자세를 넘어 직선(line), 평면(plane), IMU 관성 사전적분, 심지어 3D 객체(object) 제약까지 저마다의 사용자 정의 엣지로 빚어 넣어 그래프 SLAM을 확장했다.

---

## 아직 열린 문제

**어떤 robust kernel을 쓸 것인가.** Huber, Cauchy, Geman-McClure, DCS 등 선택지는 많지만, 어느 환경에서 어떤 kernel과 파라미터가 최선인지는 여전히 엔지니어의 경험과 수작업 튜닝에 맡겨져 있다. Chebrolu 등(RA-L 2021)은 [adaptive robust kernel](https://doi.org/10.1109/LRA.2021.3061331)을 제안해 Barron의 일반화 kernel 모양을 최적화 도중 잔차 분포에 맞춰 자동으로 조율하려 했고 ICP와 BA에서 가능성을 보였지만, 범용 표준까지는 아직 갈 길이 남았다.

**비가우시안과 다중 가설의 표현.** GTSAM과 g2o의 표준 최적화기는 모든 잔차가 단봉 가우시안 분포를 따른다는 전제 위에 서 있다. 현실의 잘못된 loop closure 확률이나 "복도의 첫 번째 문인지 두 번째 문인지" 모호한 다중 가설을 담기 위해 robust kernel을 넘어 max-mixture, hybrid factor 같은 확장이 시도되고 있으나, 비가우시안 분포를 실시간 factor graph 안에서 매끄럽게 다루는 일은 여전히 어려운 숙제다.

**초대규모·고밀도 루프 환경의 확장성.** Bayes tree의 증분 효율은 새 factor가 건드리는 clique가 국소적일 때 극대화된다. 반대로 도시 규모의 거대한 지도 안에서 수많은 loop closure가 촘촘히 얽히면 영향을 받는 subtree와 fill-in이 급격히 커져 계산량과 메모리가 함께 뛴다. 이를 누르기 위해 지도를 계층적으로 쪼개거나 submap으로 묶어 관리하는 연구가 계속 이어지고 있다.

---

2010년대에 접어들어 g2o와 GTSAM이 사실상 표준 backend로 자리 잡으면서, SLAM backend를 둘러싼 방법론 논쟁은 한동안 수면 아래로 가라앉는 듯 보였다. 연구자들의 시선은 자연스레 어떤 특징을 뽑아 어떻게 루프를 알아볼 것인가 하는 frontend 쪽으로 옮겨 갔다.

하지만 backend의 수식을 가만히 들여다본 이론가들 앞에는 여전히 답 없는 근본 질문 하나가 버티고 있었다. g2o와 GTSAM이 "수렴했다"며 돌려준 그 해가, 과연 운 좋게 찾은 국소 최솟값(local minimum)이 아니라 진짜 **전역 최솟값(global minimum)**이라는 보장이 어디에 있는가?(→ 7장)
