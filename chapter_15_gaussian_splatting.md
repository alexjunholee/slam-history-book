# Ch.15 — Gaussian Splatting: 3DGS에서 GS-SLAM까지

iMAP과 NICE-SLAM은 MLP로 공간을 저장할 수 있음을 보였지만, 그 표현은 직접 고치기 어려웠다. iMAP에서는 새 관측으로 MLP를 갱신하면 다른 영역까지 함께 바뀌었다. NICE-SLAM은 지역 feature grid를 써서 이 문제를 줄였지만, RTX 3090 Ti에서 잰 Replica 처리 속도가 1 fps에도 못 미쳐 실시간 SLAM이라 부르기 어려웠다. 장면이 네트워크 파라미터 안에 들어 있어서 사람이 내용을 직접 들여다보기도 어려웠다.

2023년 8월 SIGGRAPH에서 Bernhard Kerbl(Inria), Georgios Kopanas, Thomas Leimkühler, George Drettakis가 3D Gaussian Splatting(3DGS) [논문](https://arxiv.org/abs/2308.04079)을 발표했다. 이들은 MLP나 voxel grid를 쓰지 않고, 작은 타원체인 Gaussian primitive 수백만 개를 공간에 놓아 장면을 명시적으로 표현했다. 렌더링 방식은 Matthias Zwicker 등의 EWA splatting(2001)을 바탕으로 했고, 표현은 NeRF처럼 differentiable rendering(미분 가능한 렌더링)으로 최적화했다. 그 뒤 6개월 동안 이 표현을 쓴 SLAM 논문이 잇따라 나왔다.

---

## 3DGS의 구조

3DGS에서 Gaussian 하나는 위치(mean) $\boldsymbol{\mu} \in \mathbb{R}^3$, 공분산 행렬 $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, 학습하는 불투명도 $o \in (0,1)$, 구면조화함수(spherical harmonics) 계수로 나타낸 색으로 이루어진다. 학습을 안정시키려고 공분산은 스케일 벡터 $\mathbf{s}$와 단위 쿼터니언 $\mathbf{q}$로 나눠 둔다. $\mathbf{S}$는 $\mathbf{s}$로 만든 대각 행렬, $\mathbf{R}$은 $\mathbf{q}$로 만든 회전 행렬이다:

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

렌더링할 때는 화면에 투영한 2D Gaussian을 깊이 순서대로 알파 블렌딩(alpha-blending)한다. Gaussian마다 유효 불투명도 $\alpha_i$는 학습한 불투명도 $o_i$에 픽셀 위치에서 계산한 2D Gaussian 값 $G_i(\mathbf{x})$를 곱한 값이다. 픽셀 색 $C$는 다음과 같다.

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = o_i G_i(\mathbf{x})$$

3DGS는 GPU 래스터화(rasterization)로 렌더링한다. NeRF처럼 광선마다 volume rendering 적분을 수치로 근사하는 과정이 없다. 저자들은 타일 기반 래스터라이저의 forward와 backward를 모두 CUDA 커널로 직접 짰고, A6000 GPU에서 1080p 영상을 30 fps 이상으로 렌더링했다. 이 수치는 학습을 마친 장면에서 새 시점을 그리는 속도다. tracking과 지도 갱신까지 포함한 NICE-SLAM의 처리 속도와 바로 견줄 값은 아니다.

초기값은 SfM이 만든 sparse point cloud다. 학습하는 동안 Gaussian을 나누고(splitting), 복제하고(cloning), 지우는(pruning) **densification** 절차를 되풀이한다. 뷰 공간(view-space) 위치 그래디언트가 임계치를 넘은 Gaussian 가운데, 덜 복원된 영역의 작은 Gaussian은 같은 크기로 복제해 그래디언트 방향으로 옮긴다. 분산이 큰 영역의 큰 Gaussian은 작은 Gaussian 둘로 나눈다. 불투명도가 낮은 Gaussian은 주기적으로 지운다.

> 🔗 **차용.** 3DGS가 3D Gaussian을 화면의 2D Gaussian으로 투영하는 식은 Zwicker et al.의 [EWA splatting (2001)](https://www.cs.umd.edu/~zwicker/publications/EWAVolumeSplatting-VIS01.pdf)에서 왔다. Zwicker 등은 volume 데이터를 렌더링하려고 샘플마다 타원형 Gaussian 커널을 화면에 투영했다. 3DGS는 이 커널의 변수를 학습하게 했고, GPU 타일 래스터라이저로 속도를 높였다.

---

## 3DGS가 SLAM 지도에 맞았던 까닭

Ch.14에서 본 catastrophic forgetting 때문에 MLP 기반 NeRF 지도는 조금씩 갱신하기 어려웠다. NICE-SLAM의 voxel grid는 이 문제를 줄였지만, 해상도를 높이면 메모리가 늘었다.

Gaussian은 공간의 정해진 자리에 놓이므로, 새 keyframe이 들어오면 그 영역에 Gaussian을 더하면 된다. densification은 keyframe을 더하는 방식과 잘 맞았다. 렌더링 품질은 NeRF 수준이었고 속도는 훨씬 빨랐다.

---

## GS-SLAM: 초기 시도

Chi Yan(Shanghai AI Laboratory, HKUST)과 공동 연구자들은 2023년 11월 [GS-SLAM](https://arxiv.org/abs/2311.11700)을 arXiv에 올렸고, 이 논문은 CVPR 2024에 실렸다. 비슷한 시기에 공개된 3DGS SLAM 원고 가운데 하나로, 3DGS를 tracking과 mapping에 모두 쓴 초기 시스템이다.

GS-SLAM은 기존 SLAM 구조를 따른다. tracking은 현재 프레임의 자세를 추정하고, mapping은 Gaussian 지도를 갱신한다. 새로 제안한 것은 adaptive expansion과 coarse-to-fine tracking이다. adaptive expansion은 새로 관측한 영역에 Gaussian을 더하고, 표면 앞에 떠 있는 잡음 Gaussian의 불투명도를 크게 낮춘다. coarse-to-fine tracking은 먼저 성기게 뽑은 픽셀로 대략의 자세를 구한 뒤, 믿을 만한 Gaussian만 골라 자세를 다시 최적화한다. 이렇게 해서 시간을 줄이고 추정을 안정시켰다.

tracking은 렌더링한 영상과 관측 영상의 photometric 손실로 자세를 최적화한다. 손실은 샘플링한 픽셀의 L1 색 손실이다:

$$\mathcal{L}_{track} = \sum_m \|\mathbf{C}_m - \hat{\mathbf{C}}_m\|_1$$

mapping에서는 depth L1과 color L1의 가중합을 쓴다. 3DGS 원 논문의 학습 손실 $(1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$ ($\lambda=0.2$)은 이후 Gaussian 지도 학습에서 기본으로 쓰였다. 이 손실을 자세로 미분할 수 있는 것은 3DGS의 rasterizer가 미분 가능하기 때문이다.

Replica 데이터셋에서 GS-SLAM은 렌더링 PSNR이 두 번째로 좋은 방법보다 1.52 dB 높았고, 시스템 전체가 RTX 4090에서 8.34 fps로 돌았다. 저자들은 한계로 질 좋은 depth 데이터에 기대는 점과 큰 장면에서 메모리가 많이 드는 점을 들었다.

---

## SplaTAM: silhouette 기반 densification

Nikhil Keetha(카네기멜론대)의 [Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126)은 더 단순한 densification 규칙을 썼다.

SplaTAM은 렌더링한 Gaussian이 픽셀을 얼마나 덮는지 나타내는 **silhouette mask**로, 현재 뷰에서 기존 Gaussian이 설명하지 못하는 영역을 찾는다. silhouette 값이 0.5보다 작은 픽셀과, 관측 depth가 렌더링 depth보다 앞에 있으면서 depth 오차가 크게 벌어진 픽셀에 새 Gaussian을 더한다.

SplaTAM은 지도를 고정한 채 자세를 최적화하는 tracking과, 자세를 고정한 채 Gaussian 변수를 조정하는 mapping을 번갈아 돌린다. GS-SLAM도 이렇게 단계를 나눈다.

> 🔗 **차용.** keyframe을 골라 지도를 유지하는 방식은 PTAM(Klein & Murray, 2007, Ch.5)부터 쓰였다. SplaTAM은 mapping 때 현재 프레임, 가장 최근 keyframe, 그리고 현재 프레임과 가장 많이 겹치는 이전 keyframe들을 골라 함께 최적화한다.

Replica 데이터셋에서 SplaTAM의 평균 PSNR은 34.11 dB였다. 같은 논문의 표에서 NICE-SLAM은 24.42 dB였다.

> 📜 **예언 vs 실제.** Keetha는 SplaTAM(2024)의 Limitations & Future Work에서 motion blur, 큰 depth noise, 급한 회전에 약한 점을 한계로 들었다. OpenVDB 같은 표현으로 큰 장면까지 넓힐 수 있다고 적었고, 알려진 intrinsics와 dense depth 없이 돌게 하는 일을 앞으로 풀 과제로 꼽았다. 같은 해 Matsuki의 MonoGS(CVPR 2024)는 단안 RGB만 쓰는 시스템을 따로 내놓았다. intrinsics 없이 동작하는 방법과 대규모 장면 문제는 2025년에도 연구 중이다.

---

## MonoGS: monocular RGB

Hidenobu Matsuki(Imperial College Dyson Robotics Lab)의 [Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741)는 depth sensor 없이 단안(monocular) RGB 카메라만으로 3DGS SLAM을 돌렸다. Matsuki는 Sucar(iMAP), Bloesch(CodeSLAM)와 같은 Davison 그룹 소속이었다.

단안 카메라만으로는 절대 scale을 알 수 없다. Matsuki는 Gaussian의 형상을 직접 최적화하고 등방성 손실을 더했지만, 단안 궤적의 scale은 여전히 정해지지 않는다.

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

여기서 $\mathbf{s}_k \in \mathbb{R}^3$는 k번째 Gaussian의 스케일 벡터이고, $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$는 세 축 스케일의 평균이다. 이 등방성(isotropy) 정규화는 세 축 길이가 크게 벌어지지 않게 한다. 시점이 부족하면 Gaussian이 시선 방향으로 길게 늘어나 렌더링과 tracking을 망치는데, 이 정규화가 그런 현상을 줄인다.

tracking에서는 Gaussian을 렌더링한 영상과 관측 영상의 photometric loss로 자세를 직접 최적화했다. 새 Gaussian을 넣을 때, 렌더링한 depth가 있는 픽셀에서는 그 깊이 주변에 작은 분산으로 초기화한다. depth가 없는 픽셀에서는 렌더링한 깊이의 중앙값 주변에 큰 분산으로 초기화한다. 이렇게 잡은 깊이는 여러 시점의 관측으로 고쳐 나간다.

> 🔗 **구분.** MonoGS는 미리 학습한 monocular depth 예측기나 외부 tracking 모듈을 쓰지 않는다.

[MonoGS의 2023년 첫 arXiv 원고](https://arxiv.org/abs/2312.06741v1)에서 TUM-RGBD 데이터셋의 평균 ATE RMSE는 monocular 4.44 cm, RGB-D 1.58 cm였다. 단안 ATE는 정답 궤적에 scale을 맞춘 뒤 잰 값이고, RGB-D 값은 rigid alignment만 해서 잰 값이다. 렌더링 품질은 RGB-D 설정의 Replica에서 평균 PSNR(peak signal-to-noise ratio, 높을수록 정답 영상에 가깝다) 37.50 dB였다.

---

## RTG-SLAM과 실시간 처리

다음 과제는 속도였다. 앞에서 본 GS-SLAM은 RTX 4090에서 8.34 fps로 돌았다. Zhejiang University의 Zhexi Peng 등이 발표한 [RTG-SLAM (SIGGRAPH 2024)](https://arxiv.org/abs/2404.19706)은 큰 장면을 실시간으로 RGB-D 재구성하는 것을 목표로 했다.

RTG-SLAM은 Gaussian마다 불투명하거나 거의 투명하도록 강제한다. 불투명한 Gaussian은 표면과 주된 색을 맡고, 투명한 Gaussian은 남은 색 차이를 맡는다. 불투명한 Gaussian 하나가 표면 한 영역을 덮으므로 Gaussian을 여러 개 겹칠 필요가 없다. 그래서 메모리와 계산량이 크게 준다. 또 Gaussian을 안정·불안정으로 나눠 불안정한 것만 최적화하고, 그것이 덮는 픽셀만 렌더링한다. 논문은 최신 NeRF 기반 RGB-D SLAM보다 약 2배 빠르고 메모리는 절반가량이라고 보고했다.

---

## 3DGS 이후 달라진 표현 선택

2024년 이후 3DGS 논문이 빠르게 늘면서 SLAM 지도로 고를 수 있는 표현도 많아졌다. 임베디드, 경로 계획, 안전이 중요한 응용에서는 지금도 TSDF와 occupancy grid를 쓴다. 예를 들어 NVIDIA의 [nvblox](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html)는 Jetson 같은 장치에서 TSDF 지도를 만들고, 이를 경로 계획용 거리장(ESDF)과 cost map으로 바꾼다.

3DGS가 빨리 퍼진 데에는 하드웨어도 작용했다고 본다. 3DGS가 쓰는 래스터화에 맞춰 GPU가 오랫동안 최적화돼 왔고, NeRF는 ray marching(광선을 따라 샘플을 찍는 렌더링)으로 그린다.

> 🔗 **차용.** 장면 표현을 gradient로 최적화하고 photometric loss로 관측과 렌더링을 비교하는 방식은 NeRF(Mildenhall et al. 2020)에서 왔다. 3DGS는 이 방식을 그대로 쓰면서 표현만 implicit MLP에서 명시적 Gaussian으로 바꿨다.

> 📜 **예언 vs 실제.** Kerbl et al.은 3DGS(2023)의 Limitations에서 관측이 부족한 영역의 artifact, 길쭉한(elongated) artifact, popping, 정규화(regularization)를 하지 않은 점, 메모리 소비(큰 장면을 훈련할 때 20 GB를 넘을 수 있고, 렌더링에도 수백 MB)를 한계로 꼽았다. 개선 방향으로는 antialiasing, 더 원칙적인 culling, point cloud 압축 기법을 가져오는 일을 들었다. 메모리 압축은 2024년 [Compact 3DGS](https://arxiv.org/abs/2311.13681)와 [Niedermayr et al.](https://arxiv.org/abs/2401.02436)이 다뤘다. dynamic scene 확장([4DGS](https://arxiv.org/abs/2310.08528), [Deformable 3DGS](https://arxiv.org/abs/2309.13101))과 생성·편집([DreamGaussian](https://arxiv.org/abs/2309.16653), [GaussianEditor](https://arxiv.org/abs/2311.14521))은 원 논문이 언급하지 않았지만 2024년 전후로 따로 연구 분야가 됐다.

---

## 🧭 아직 열린 것

**Memory scaling.** Gaussian 수는 장면 크기에 비례해 늘어난다. 실내 Replica 데이터셋보다 실외 도시 구역에서 훨씬 많은 Gaussian이 필요하다. Gaussian pruning과 level-of-detail 계층 구조가 연구되고 있다. 큰 환경에서 메모리와 렌더링 품질을 함께 맞추는 일은 남은 과제다.

**Semantic 통합.** 2023년 [LERF](https://arxiv.org/abs/2303.09553)가 NeRF에 언어 feature를 붙였고, [LangSplat](https://arxiv.org/abs/2312.16084)은 Gaussian 표현에 언어 feature를 붙였다. 그 뒤 semantic Gaussian을 SLAM에 넣은 연구도 나왔다. 실시간 갱신, tracking 품질, semantic 정확도를 같은 조건에서 함께 비교하는 일은 아직 남아 있다.

**Dynamic scene.** 4DGS와 Deformable 3DGS는 Gaussian에 시간 차원을 더하는 방법을 제안했다. GS-SLAM(Yan et al. 2024), SplaTAM(Keetha et al. 2024), MonoGS(Matsuki et al. 2024)는 모두 세계가 정적이라고 가정한다. 움직이는 물체를 다루는 SLAM은 Ch.15b에서 다룬다.

초기 Gaussian을 어디서 얻을지도 문제였다. 자세를 알아야 Gaussian을 놓을 수 있고, Gaussian이 있어야 자세를 추정할 수 있다. 초기화 방법은 시스템마다 달랐다. RGB-D 방법은 센서가 잰 깊이를 썼다. MonoGS는 렌더링한 깊이로 시스템 안에서 초기 깊이를 잡았다. Ch.16에서 다루는 DUSt3R와 후속 연구는 이미지에서 기하를 바로 예측하는 네트워크를 학습했다.
