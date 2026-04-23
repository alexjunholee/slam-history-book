# Ch.14 — NeRF 충격과 SLAM 접목: iMAP→NICE-SLAM

Ch.13에서 DROID-SLAM은 learned representation이 SLAM의 핵심 루프(tracking)에 직접 들어올 수 있음을 보였다. MLP나 recurrent network가 feature를 만들고, 그 feature 위에서 포즈를 최적화했다. 그런데 learned representation을 tracking이 아니라 *지도 자체*에 쓸 수 있다면? Imperial College의 Edgar Sucar는 그 질문을 2021년 iMAP으로 답했고, 그 답의 재료를 SLAM 바깥에서 가져왔다. NeRF였다.

2020년 3월, Ben Mildenhall과 동료들이 arXiv에 올린 [Mildenhall et al. 2020. NeRF](https://arxiv.org/abs/2003.08934)는 8개 이미지로 새로운 시점의 사진을 만들어냈다. 그 사진은 빛과 그림자의 결을 가지고 있었다. SLAM 커뮤니티는 처음에 이것을 렌더링 문제로 보았다. 지도를 *만드는* 방법이 아니라 지도를 *보여주는* 방법이라고. 그 인식이 바뀌는 데는 14개월이 걸렸다. 2021년 ICCV에서 Sucar가 iMAP을 발표하면서, NeRF가 렌더링 도구가 아니라 지도 표현 자체로 쓰일 수 있다는 게 드러났다. iMAP은 KinectFusion(Ch.9)의 계보를 이었다. implicit neural field가 TSDF voxel grid를 대체할 수 있다는 가설의 첫 구현체였다.

---

## NeRF: MLP 기반 공간 표현

NeRF의 핵심 아이디어는 하나의 MLP가 3D 공간 전체를 암묵적으로 기억한다는 것이다. 입력은 공간 좌표 $(x, y, z)$와 시선 방향 $(\theta, \phi)$. 출력은 그 위치의 색상 $(r, g, b)$과 밀도 $\sigma$. 이것만으로 어떻게 장면 전체를 표현하는가.

렌더링은 volume rendering 방정식으로 이루어진다. 카메라 원점 $\mathbf{o}$에서 방향 $\mathbf{d}$로 나간 광선을 $t$ 매개변수로 샘플링한다:

$$\hat{C}(\mathbf{r}) = \int_{t_n}^{t_f} T(t)\,\sigma\!\left(\mathbf{r}(t)\right) \mathbf{c}\!\left(\mathbf{r}(t), \mathbf{d}\right)\, dt$$

여기서 $T(t) = \exp\!\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s))\, ds\right)$는 광선이 거기까지 막히지 않고 도달할 누적 투과율이다. 실제로는 이 적분을 구간별 리만 합으로 근사한다.

> 🔗 **차용.** Volume rendering 방정식은 [Kajiya & Von Herzen(1984)](https://courses.cs.duke.edu/cps296.8/spring03/papers/RayTracingVolumeDensities.pdf)의 고전 그래픽스 논문에서 왔다. 40년 가까이 오프라인 렌더링의 물리 기반 도구였던 것을 Mildenhall은 역방향 최적화의 손실 함수로 전환했다.

MLP가 고주파 공간 신호를 학습하지 못하는 문제를 Mildenhall et al.(2020) NeRF 논문은 positional encoding으로 풀었다. 좌표 $(x, y, z)$를 사인·코사인 함수로 여러 주파수에 걸쳐 투영하면, 네트워크가 세밀한 텍스처와 날카로운 경계를 학습할 수 있다:

$$\gamma(p) = \left(\sin(2^0 \pi p),\, \cos(2^0 \pi p),\, \ldots,\, \sin(2^{L-1} \pi p),\, \cos(2^{L-1} \pi p)\right)$$

> 🔗 **차용.** NeRF의 positional encoding은 Mildenhall et al.(2020) 원 논문에 포함된 것이다. 같은 해 [Tancik et al.(2020)](https://arxiv.org/abs/2006.10739) "Fourier Features Let Networks Learn High Frequency Functions"가 NTK(neural tangent kernel) 이론으로 이 기법의 작동 원리를 설명했다.

NeRF의 학습은 역방향이다. 알고 있는 카메라 포즈에서 찍은 이미지들과 렌더링 결과를 비교해 픽셀 단위 L2 손실을 최소화한다. 최적화가 끝나면 MLP 가중치 자체가 장면의 geometry와 appearance를 저장한다. 복셀도, 메시도, 포인트클라우드도 쓰지 않는다. 공간은 네트워크 파라미터 안에 있다.

그러나 원래 NeRF에는 뚜렷한 약점이 있었다. 학습에 수 시간이 걸렸고, 한 장면에 특화되었으며, 카메라 포즈는 COLMAP 같은 외부 SfM으로 미리 구해야 했다. 이것을 SLAM에 이식하려면 포즈 추정과 지도 학습을 동시에, 실시간에 가깝게 해야 한다.

---

## iMAP: 최초의 neural implicit SLAM

Imperial College Dyson Robot Learning Lab의 Edgar Sucar가 2021년 ICCV에 발표한 [Sucar et al. 2021. iMAP](https://doi.org/10.1109/ICCV48922.2021.00612)은 그 시도였다. **iMAP**(Implicit MAP)은 RGB-D 카메라의 입력을 받아 단일 MLP를 지도로 쓰면서 포즈를 동시에 최적화했다.

구조는 두 개의 교번 최적화 루프다. *mapping* 루프는 현재 키프레임과 과거 랜덤 샘플 키프레임에서 광선을 샘플링해 MLP를 업데이트한다. *tracking* 루프는 MLP를 고정하고 현재 프레임의 포즈를 렌더링 손실로 최적화한다. 두 루프는 공유된 단일 MLP 위에서 동작한다.

손실 함수는 두 가지다. 색상 손실 $\mathcal{L}_{\text{color}} = \|\hat{C} - C\|_2^2$과 깊이 손실 $\mathcal{L}_{\text{depth}} = \|\hat{D} - D\|_2^2$. RGB-D를 쓰므로 depth supervision이 있어 geometry 학습이 안정적이었다.

iMAP은 개념 증명이었다. 소규모 실내 장면에서 동작했지만 두 가지 구조적 문제가 있었다. 첫째, 단일 MLP는 새로운 영역이 추가될수록 이전 영역을 잊어버렸다. 신경망의 catastrophic forgetting 문제다. Sucar는 keyframe replay로 부분 완화했으나 근본 해결이 아니었다. 둘째, 장면이 커질수록 단일 MLP의 표현력이 부족해졌다. MLP의 forward pass는 파라미터 수와 무관하게 전체 공간을 하나의 함수로 취급하기 때문이다.

> 📜 **예언 vs 실제.** Sucar는 iMAP 논문 Conclusion에서 "future directions for iMAP include how to make more structured and compositional representations that reason explicitly about the self similarity in scenes"라고 적었다. 구조화·합성적 표현 방향은 실제로 후속 연구의 중심 줄기가 되었다. 5개월 뒤 ETH 취리히의 NICE-SLAM 사전공개는 multi-resolution voxel feature grid로 공간을 계층적으로 쪼갰고, Wang et al.의 Co-SLAM(2023)은 hash grid와 coordinate encoding을 합성해 RTX 3090Ti에서 10-17Hz 수준의 준실시간까지 밀어붙였다. 다만 "self-similarity를 명시적으로 추론하는" 쪽은 NeRF-SLAM 본류에서 크게 발전하지 않았고, 단일 MLP를 정교화하는 계보 역시 중심에서 밀려났다. `[적중+부분]`

---

## NICE-SLAM: 계층 격자와 scalability

iMAP의 단일 MLP 문제에 대한 직접적인 답은 ETH 취리히의 Zihan Zhu·Songyou Peng이 2022년 CVPR에서 발표한 [Zhu et al. 2022. NICE-SLAM](https://arxiv.org/abs/2112.12130)에서 나왔다. **NICE-SLAM**(Neural Implicit Scalable Coding for SLAM)은 단일 MLP 대신 multi-resolution voxel feature grid와 작은 MLP decoder를 결합했다.

아이디어는 공간을 명시적 복셀 격자로 나누되, 각 복셀에 학습 가능한 feature vector를 두는 것이다. 렌더링 시 샘플 좌표 주변 복셀들의 feature를 trilinear interpolation으로 결합한 뒤 작은 MLP에 통과시켜 색상과 occupancy를 얻는다. MLP는 크지 않아도 된다. 공간 정보의 대부분은 격자에 담겨 있기 때문이다.

NICE-SLAM은 세 단계 해상도 격자를 계층적으로 쌓았다. 거친 격자는 전체 geometry 형태를 담고, 중간 격자는 구조의 세부를, 세밀한 격자는 texture를 담는다. 새로운 영역이 추가되면 해당 복셀의 feature만 업데이트하면 되므로 다른 영역의 catastrophic forgetting이 크게 줄었다.

tracking에서 NICE-SLAM은 iMAP과 유사하게 MLP와 격자 feature를 고정하고 포즈를 최적화했다. mapping에서는 격자 feature를 업데이트했다. Replica·ScanNet 데이터셋에서 iMAP보다 넓은 공간을 다뤘고 세부 표현 품질도 높았다.

그러나 한계가 있었다. 격자 자체의 메모리가 해상도의 세제곱으로 증가했다. 실내 방 한두 개는 다룰 수 있었지만 복층 건물이나 야외로의 확장은 여전히 미해결이었다. 속도도 실시간과 거리가 있었다.

Thomas Müller의 [Müller et al. 2022. Instant-NGP](https://nvlabs.github.io/instant-ngp/)는 2022년 SIGGRAPH에서 이 병목을 다른 각도에서 공략했다. hash table 기반 feature encoding으로 복셀 격자의 메모리 폭발을 해결하고 학습 속도를 수 분에서 수 초로 줄였다. Instant-NGP는 SLAM 논문이 아니었지만, 이후 NeRF-SLAM 연구들이 거의 모두 hash encoding을 채용했다.

> 🔗 **차용.** NICE-SLAM의 multi-resolution feature grid는 Instant-NGP의 hash encoding과 시기적으로 겹치며 독립적으로 설계되었지만, 실제 NeRF-SLAM 구현에서는 Instant-NGP의 hash grid가 NICE-SLAM 격자를 빠르게 대체했다. TSDF를 격자에 저장하던 KinectFusion(Ch.9)의 논리적 후계가 feature를 격자에 저장하는 방식으로 이어진 계보이기도 하다.

---

## Co-SLAM과 NeRF-SLAM: 두 가지 통합 방향

iMAP·NICE-SLAM 이후 2022년 말부터 여러 시스템이 갈래를 나눴다. 한 방향은 implicit representation을 더 효율적으로 만드는 것, 다른 방향은 전통 SLAM의 강건한 backend를 NeRF map과 결합하는 것이었다.

UCL의 [Wang et al.(2023) **Co-SLAM**](https://arxiv.org/abs/2304.14377)은 전자에 속한다. joint coordinate·parametric encoding을 써서 multi-resolution hash grid와 one-blob 인코딩을 결합했다. 두 표현이 서로 보완하도록 설계해 빠른 수렴과 surface completeness를 함께 노렸다. hash grid가 관측된 dense 영역을 빠르게 채우고, coordinate encoding이 미관측 영역에 smooth prior를 제공하는 방식이었다. Replica 데이터셋에서 RTX 3090 기준 15-17Hz. NeRF 기반 SLAM이 처음으로 준실시간 영역에 닿은 지점이었다.

같은 해 같은 CVPR에서 Idiap/EPFL의 [Johari et al.의 **ESLAM**](https://arxiv.org/abs/2211.11704)은 비슷한 문제를 다른 각도에서 풀었다. 3D feature grid 대신 multi-scale axis-aligned feature plane을 써 메모리 증가를 $O(n^3)$에서 $O(n^2)$로 낮추고, volume density 대신 TSDF를 decoding 목표로 삼아 수렴을 가속했다.

Antoni Rosinol(MIT)이 2023년에 낸 [**NeRF-SLAM**](https://arxiv.org/abs/2210.13641)은 다른 접근이었다. 전통 SLAM의 tracking과 backend(factor graph 최적화)를 그대로 쓰고, 지도 표현만 NeRF로 교체했다. 포즈와 dense depth는 모두 DROID-SLAM frontend가 제공했다. Rosinol은 이 포즈·깊이와 불확실성을 입력으로 받아 Instant-NGP 기반 map을 병렬로 쌓았다.

> 🔗 **차용.** NeRF-SLAM의 backend는 Dellaert의 factor graph 최적화(Ch.6) 위에서 작동한다. "NeRF가 지도를 바꿀 수 있다"는 가설 아래에서도 포즈 추정의 핵심 수학은 2005년 이후 확립된 그래프 구조 위에 그대로 남아 있었다.

Rosinol은 모듈성을 골랐다. NeRF를 전체 파이프라인에 강제 삽입하는 대신 지도 표현 계층에서만 교체했다. 덕분에 루프 클로저 같은 전통 SLAM 기능이 그대로 남았다.

---

## iMAP의 구조적 한계와 그 의미

돌아보면 iMAP의 의의는 성능보다 개념에 있었다. 단일 MLP가 전체 장면을 기억할 수 있고, 그 MLP를 실시간에 가깝게 업데이트하면서 포즈까지 최적화할 수 있다는 것을 보인 최초의 시스템이었다.

단일 MLP의 근본 문제는 지역성(locality)의 부재다. 공간의 어떤 부분을 렌더링하든 MLP 전체를 통과한다. 결과가 두 가지다. 첫째, 새 영역을 학습하면 가중치 전체가 바뀌어 기존 영역의 표현이 훼손된다(catastrophic forgetting). 둘째, 장면이 커질수록 단일 MLP가 담아야 할 공간 다양성이 늘어나 더 큰 네트워크, 더 많은 이터레이션이 필요해진다. 표현 용량은 파라미터 수에 선형으로 묶여 있는데 장면 복잡도는 공간 부피에 따라 커진다. 지역성 없는 표현은 규모가 커질수록 불리하다.

NICE-SLAM의 격자, Instant-NGP의 hash encoding, Co-SLAM의 이중 인코딩은 모두 이 지역성 문제의 답이었다. 공간을 국소적으로 나눠 각 부분이 자신의 영역만 기억하게 하면, 새 정보 추가가 기존 기억을 덜 침범하고, 특정 영역 렌더링 비용이 전체 장면 크기와 분리된다.

---

## 🧭 아직 열린 것

**실시간 NeRF-SLAM.** 2023년 기준 iMAP·NICE-SLAM은 실시간과 거리가 있었고, Co-SLAM이 RTX 3090Ti에서 10-17Hz로 준실시간까지 닿았지만 모바일·로봇 임베디드 환경의 실시간에는 여전히 미치지 못했다. Gaussian Splatting(Ch.15)이 명시적 표현으로의 복귀를 통해 속도 문제를 다른 방식으로 해결했지만, implicit neural field 자체의 고전적 실시간 SLAM(30fps 이상, 소비자 GPU 없이)은 미완으로 남아 있다. Instant-NGP가 렌더링 속도를 극적으로 높였음에도 동시 추적·지도 구축 루프의 전체 처리량은 여전히 제약이 있다.

**대규모 야외 환경.** [Block-NeRF](https://arxiv.org/abs/2202.05263)(2022, Tancik et al.)처럼 공간을 여러 국소 NeRF로 분할하는 시도는 있었지만, SLAM의 루프 클로저·전역 일관성 요구와 매끄럽게 맞물리지 못했다. 도시 규모 NeRF-SLAM은 개방형 문제다.

**semantic·편집 가능한 implicit 지도.** NeRF map은 렌더링에 최적화되어 있어 semantic label 삽입이나 사후 편집이 어렵다. "이 물체를 지도에서 지워라"나 "이 영역을 다른 용도로 분류하라"는 조작이 TSDF나 포인트클라우드 대비 훨씬 불편하다. language-guided NeRF editing 연구([LERF](https://arxiv.org/abs/2303.09553), [Nerfstudio](https://arxiv.org/abs/2302.04264) 생태계)가 진행 중이나 SLAM 파이프라인과의 실시간 통합은 2026년 현재 연구 단계다.

---

iMAP·NICE-SLAM이 implicit field를 극한까지 밀어붙이는 동안, 연구 커뮤니티의 일각은 반대 방향을 보고 있었다. 지도를 MLP 가중치나 feature grid 안에 암묵적으로 가두는 대신, 공간에 명시적으로 배치된 수백만 개의 작은 타원체로 흩뿌리면 렌더링은 빠르고 편집은 직관적일 수 있었다. 2023년 SIGGRAPH에서 Bernhard Kerbl의 논문이 나오기 전까지 그것은 아직 가설이었다.
