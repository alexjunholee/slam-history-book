# Ch.15 — Gaussian Splatting 시대: 3DGS에서 GS-SLAM까지

Ch.14에서 iMAP과 NICE-SLAM은 MLP로 공간을 기억하는 방법의 가능성을 보여주었다. 그러나 대가가 있었다. MLP는 불투명했다. 어떤 뉴런이 어떤 공간을 담당하는지 알 수 없었고, 새 관측이 들어올 때마다 전체 네트워크를 건드려야 했다. Co-SLAM의 RTX 3090에서 0.1fps는 "실시간 SLAM"이라는 말과 공존하기 어려운 수치였다. 장면은 네트워크 파라미터 안에 갇혀 있었고, 그 안을 들여다볼 방법이 없었다.

2023년 8월 SIGGRAPH에서 Bernhard Kerbl, Georgios Kopanas, Thomas Leimkuhler, George Drettakis가 발표한 논문은 컴퓨터 비전 커뮤니티에 짧고 거센 충격을 남겼다. 3D Gaussian Splatting(3DGS)는 NeRF가 3년에 걸쳐 쌓아온 암묵적 표현(implicit representation) 패러다임을 정면으로 부정하지 않으면서도, 결정적으로 다른 선택을 했다. iMAP·NICE-SLAM·Co-SLAM이 MLP와 voxel grid로 장면을 잠그는 동안, 3DGS는 수백만 개의 작은 타원체, 즉 Gaussian primitive를 공간에 흩뿌렸다. SLAM 커뮤니티가 6개월 만에 이 표현으로 쏠린 데에는 이유가 있었다. Kerbl의 선택은 새로운 발명이 아니었다. Matthias Zwicker의 EWA splatting(2001)이라는 20년 된 그래픽스 기법이 그 뿌리였고, NeRF의 differentiable rendering 정신은 그대로 계승했다. 차이는 표현의 형식에 있었다.

---

## 3DGS의 구조

Kerbl은 장면을 explicit한 Gaussian 집합으로 표현했다. 각 Gaussian은 위치(mean) $\boldsymbol{\mu} \in \mathbb{R}^3$, 공분산 행렬 $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, 불투명도 $\alpha \in (0,1]$, 그리고 구면조화함수(spherical harmonics) 계수로 표현된 색상을 가진다. 공분산은 학습 안정성을 위해 스케일 벡터 $\mathbf{s}$와 단위 쿼터니언 $\mathbf{q}$로 분해한다:

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

렌더링은 projected 2D Gaussian을 깊이 순서대로 알파 블렌딩(alpha-blending)하는 방식으로 이루어진다. 여기서 각 Gaussian의 유효 불투명도 $\alpha_i$는 learnable opacity $\sigma_i$와 픽셀 위치에서 평가된 2D Gaussian 밀도 $G_i(\mathbf{x})$의 곱이다. 픽셀 색상 $C$는

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = \sigma_i \cdot G_i(\mathbf{x})$$

이것이 핵심이다. Volume rendering 적분을 수치적으로 근사하는 NeRF와 달리, 3DGS는 GPU 래스터화(rasterization) 파이프라인에 직접 올라간다. 학습된 타일 기반 래스터라이저는 forward pass와 backward pass 모두 커스텀 CUDA 커널로 구현되어 있다. 결과적으로 RTX 3090 한 장에서 30fps 이상의 실시간 렌더링이 가능했다. NICE-SLAM이 같은 카드에서 0.1fps를 내던 상황과 비교하면 300배의 차이다.

초기화는 SfM에서 나온 sparse point cloud를 사용한다. 이후 학습 과정에서 Gaussian을 분열(splitting), 복제(cloning), 제거(pruning)하는 **densification** 절차를 반복한다. 뷰 공간(view-space) 위치 그래디언트가 임계치를 넘으면, scale이 큰 Gaussian은 두 자식으로 분열(split)하고 scale이 작은 Gaussian은 동일 위치에 복제(clone)한다. 투명도가 낮은 Gaussian은 주기적으로 제거한다.

> 🔗 **차용.** 3DGS의 래스터화 기반 splatting은 Zwicker et al.의 EWA splatting (2001)을 직접 계승한다. Zwicker는 점 구름을 렌더링하기 위해 각 점에 타원형 가중 평균 커널을 씌웠다. Kerbl은 그 커널을 learnable Gaussian으로 교체하고 GPU 타일 래스터라이저로 가속했다.

---

## 왜 SLAM에 즉시 유용했나

implicit representation은 SLAM에 어울리지 않는 특성을 가지고 있었다. MLP 기반 NeRF는 새 관측이 들어올 때마다 전체 네트워크를 재학습해야 했고, catastrophic forgetting 문제로 인해 incremental update가 어려웠다. 지도 확장은 네트워크 크기 재조정을 의미했다. NICE-SLAM의 voxel grid는 이 문제를 완화했지만, 해상도와 메모리의 트레이드오프를 피할 수 없었다.

3DGS는 이 문제를 구조적으로 해결했다. Gaussian은 공간에 명시적으로 위치한 객체이므로 새 키프레임이 들어올 때 해당 영역에 Gaussian을 추가하면 된다. Densification 절차는 keyframe 추가와 자연스럽게 연결된다. 지도는 Gaussian 집합의 확장으로 표현된다. 렌더링 품질은 NeRF 수준이다. 그리고 무엇보다, 실시간이다.

2023년 후반에 GS-SLAM 논문들이 쏟아진 데에는 이 계산이 있었다.

---

## GS-SLAM: 첫 번째 시도

Chi Yan(홍콩대)과 공동 연구자들은 2023년 11월 [Yan et al. 2023. GS-SLAM](https://arxiv.org/abs/2311.11700)을 arXiv에 게시했다. 3DGS를 SLAM 파이프라인에 통합한 최초의 시스템이었다.

GS-SLAM의 구조는 전통 SLAM 프레임워크를 따른다. Tracking은 현재 프레임의 포즈를 추정하고, Mapping은 Gaussian 지도를 갱신한다. 핵심 기여는 두 가지였다. 첫째, adaptive Gaussian expansion: 새 키프레임이 추가될 때 coverage가 낮은 영역에 Gaussian을 삽입하는 메커니즘. 둘째, geometry-aware Gaussian selection: 렌더링 손실 역전파 시 기여가 큰 Gaussian만 선택적으로 최적화해 속도를 확보한다.

Tracking은 포즈를 렌더링 포토메트릭 손실로 최적화한다. 현재 프레임 이미지 I와 Gaussian 지도로부터 렌더링한 이미지 Î 사이의 L1 손실과 D-SSIM을 결합한다:

$$\mathcal{L}_{track} = (1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$$

여기서 λ = 0.2는 3DGS 원 논문에서 온 기본값이다. 이 손실을 포즈에 대해 미분할 수 있는 이유가 3DGS의 differentiable rasterizer다.

Replica 데이터셋에서 NICE-SLAM 대비 렌더링 품질(PSNR)을 유지하면서 처리 속도를 크게 향상시켰다. 그러나 첫 버전의 한계도 명확했다. RGB-D 카메라(depth sensor 필요)를 전제했고, 실외 대규모 환경에서의 검증은 없었다.

---

## SplaTAM: 단순함의 미덕

Nikhil Keetha(카네기멜론대)의 [Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126)은 설계 철학에서 GS-SLAM과 달랐다. Keetha는 복잡한 선택 메커니즘 대신 실루엣 마스크 기반의 단순한 densification을 채택했다.

SplaTAM의 핵심 아이디어는 **silhouette mask**다. 현재 뷰에서 기존 Gaussian으로 설명되지 않는 영역(렌더링된 마스크에서 비어있는 부분)에 새 Gaussian을 추가한다. 이 규칙은 구현이 단순하고 직관적이다. Gaussian이 어디에 있어야 하는지를 명시적으로 계산하는 대신, 어디에 없는지를 보고 채운다.

Tracking은 포즈, Mapping은 Gaussian 파라미터를 각각 최적화하며 두 단계를 엄격히 분리한다. 이 분리가 안정성의 원천이다. GS-SLAM이 tracking과 mapping을 얽어 교대로 최적화할 때 발생하는 간섭을 SplaTAM은 피했다.

> 🔗 **차용.** SplaTAM의 키프레임 기반 지도 관리 구조는 PTAM(Klein & Murray, 2007)의 아이디어가 새 표현 위에서 재동작하는 사례다. PTAM이 keyframe을 선택적으로 삽입해 지도를 유지하던 방식이, SplaTAM에서는 Gaussian densification의 트리거로 변환되었다.

Replica 데이터셋 기준으로 SplaTAM은 PSNR 34.11 dB를 기록했다. 같은 데이터셋에서 iMAP는 19.20 dB, NICE-SLAM은 24.43 dB였다. 렌더링 품질 격차는 명확했다.

Keetha는 2024 CVPR 논문의 §6에서 "monocular, outdoor, larger-scale"을 다음 과제로 명시했다. monocular는 같은 해 MonoGS가 해결했다.

> 📜 **예언 vs 실제.** Keetha는 SplaTAM(2024) §6에서 "monocular 카메라, outdoor 환경, 더 큰 스케일"을 명시적 한계로 적었다. monocular 확장은 같은 해 Matsuki의 MonoGS(2024 CVPR)가 해결했다. outdoor와 large-scale은 2024-2025년 현재 진행 중이다. `[적중+진행형]`

---

## MonoGS: depth 없이

Hidenobu Matsuki(Imperial College Dyson Robotics Lab)의 [Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741)는 제약을 하나 제거했다. depth sensor 없이, 단안(monocular) RGB 카메라만으로 3DGS SLAM을 구동한다.

monocular 설정의 난점은 scale이다. depth 없이 메트릭 스케일을 복원하는 것은 SfM에서도 어려운 문제다. MonoGS의 해법은 Gaussian geometry를 직접 최적화하는 것이다. 렌더링된 depth와 인접 Gaussian 간의 기하 일관성 손실을 추가했다.

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

여기서 $\mathbf{s}_k \in \mathbb{R}^3$는 k번째 Gaussian의 스케일 벡터, $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$는 세 축 스케일의 평균이다. 이 등방성(isotropy) 정규화는 Gaussian이 지나치게 얇은 판 형태로 퇴화하는 것을 막는다. 특히 monocular에서 depth 감독 없이 Gaussian이 카메라 평면에 붙어버리는 현상을 억제한다.

Tracking에서 MonoGS는 포즈를 Gaussian 렌더링 photometric loss로 직접 최적화한다. 첫 프레임 초기화에서는 monocular depth prior를 사용해 Gaussian의 초기 위치를 설정하고, 이후 프레임에서는 이전 포즈를 시작점으로 삼아 렌더링 기반 refinement를 수행한다. depth sensor 없이 scale을 유지하는 핵심은 isotropic regularization과 keyframe 간 geometry 일관성 손실의 결합이다.

> 🔗 **차용.** MonoGS의 monocular depth prior 활용은 Ch.11에서 다룬 Godard의 MonoDepth2 계열 아이디어를 계승한다. depth 감독 없이도 구조를 복원할 수 있다는 self-supervised monocular depth의 통찰이 Gaussian 초기화 전략으로 흡수된 사례다.

Matsuki는 Imperial Dyson Robotics Lab 출신이다. Davison이 지도한 연구실에서 나온 Sucar(iMAP), Bloesch(CodeSLAM)와 같은 계보다. Lab의 관심이 implicit MLP → Gaussian explicit 표현으로 이동한 흐름을 MonoGS가 대표한다.

TUM-RGBD 데이터셋에서 MonoGS는 ATE RMSE 0.028m를 기록했다. ORB-SLAM3 monocular(0.049m)보다 나은 수치다. 렌더링 품질(PSNR 33.4 dB)은 depth 입력 없이 SplaTAM에 근접했다.

---

## RTG-SLAM과 실시간의 정의

GS-SLAM 계열이 풀어야 할 다음 문제는 속도였다. GS-SLAM과 SplaTAM은 실시간이라 부르기 어려운 속도에서 동작했다. Zhejiang University의 Peng Zhen과 연구팀이 2024년 발표한 RTG-SLAM은 명시적으로 실시간 목표를 설정했다.

RTG-SLAM의 전략은 Gaussian의 수를 제어하는 것이다. 모든 Gaussian을 동등하게 최적화하는 대신, 현재 카메라 뷰에서 기여도가 큰 Gaussian만 활성화해 최적화한다. Gaussian을 surfel(표면 원소) 기반으로 초기화해 geometry 합리성을 유지하면서 수를 줄인다. 결과적으로 Replica 데이터셋에서 실시간에 근접한 처리 속도를 달성했다.

---

## 사라진 경쟁자들

2024년을 기점으로 TSDF·occupancy grid는 SLAM mapping의 주류에서 밀려났다. 여전히 embedded 시스템이나 안전이 요구되는 환경에서는 쓰이지만, 연구 전선에서는 보조 역할로 후퇴했다. NeRF 기반 SLAM도 같은 해 기준으로 3DGS 대비 렌더링 속도와 update 유연성에서 불리해 보조 위치로 이동했다.

이 전환은 표현의 전환이면서 동시에 하드웨어 친화성의 전환이다. GPU 래스터라이저는 GPU 레이마처(ray marcher)보다 훨씬 잘 최적화되어 있다. 3DGS가 기존 그래픽스 파이프라인 위에서 자연스럽게 실행된다는 사실이 NeRF에 대한 채택 우위를 빠르게 만들었다.

> 🔗 **차용.** 3DGS의 differentiable rendering 정신은 NeRF에서 직접 계승한다. 장면 표현을 gradient로 최적화한다는 아이디어, photometric loss로 관측과 렌더링을 연결하는 방식은 Mildenhall et al.(2020)의 유산이다. Kerbl은 표현(implicit MLP → explicit Gaussian)을 교체했지만 패러다임을 계승했다.

> 📜 **예언 vs 실제.** Kerbl et al.은 3DGS(2023) §6에서 "정적 장면 한계, dynamic scene과 editing, training 안정화"를 명시적 한계로 적었다. dynamic 방향은 2024년 4DGS(Wu et al.), Deformable 3DGS 등이 본격 탐구를 시작했고, editing은 Dreamgaussian·GaussianEditor가 대응했다. training 안정화(densification heuristics의 민감성)는 2025년 현재도 진행 중이다. `[진행형]`

---

## 🧭 아직 열린 것

Memory scaling. Gaussian의 수는 장면 크기에 따라 선형으로 증가한다. 실내 Replica 데이터셋에서 수십만 개로 충분하던 것이 outdoor 도시 구역에서는 수천만 개로 늘어난다. Gaussian pruning과 level-of-detail 계층화가 연구되고 있지만, 대규모 환경에서 메모리와 렌더링 품질의 트레이드오프를 합리적으로 관리하는 방법은 아직 합의가 없다. Compact 3DGS 계열(Lee et al. 2024, Niedermayr et al. 2024)이 압축 방향을 탐색 중이다.

Semantic 통합. Gaussian에 semantic label을 부여하는 시도(LangSplat, LERF 등)가 2023-2024년에 활발하게 이루어졌다. 그러나 SLAM 파이프라인에서 semantic Gaussian을 실시간으로 갱신하면서 동시에 tracking 품질을 유지하는 통합된 방식은 아직 없다. semantic과 geometry를 공동 최적화할 때 발생하는 interference를 어떻게 처리할 것인가가 핵심 문제다.

Dynamic scene. 4DGS와 Deformable 3DGS는 시간 차원을 Gaussian에 추가하는 방향을 제안했다. SLAM 설정에서 dynamic object는 배경과 다른 움직임을 가지므로 별도로 처리해야 한다. GS-SLAM(Yan et al. 2023), SplaTAM(Keetha et al. 2024), MonoGS(Matsuki et al. 2024) 모두 정적 세계 가정을 유지한다. Dynamic SLAM에서 Gaussian이 어떻게 이동하는 객체를 표현하고 추적할 것인가는 2025년 기준으로 열려 있다.

그러나 3DGS가 남긴 또 다른 질문이 있었다. Gaussian은 어디서 오는가. SfM point cloud에서, 또는 depth sensor에서. 포즈를 알아야 Gaussian을 놓을 수 있고, Gaussian이 있어야 포즈를 추정할 수 있다. 이 닭-달걀 문제는 GS-SLAM 계열이 여전히 외부 초기화에 의존하게 만들었다. Ch.16에서 다루는 DUSt3R와 그 후계들은 다른 출발점을 선택했다. geometry를 표현 위에 올리는 대신, geometry를 처음부터 학습한다.
