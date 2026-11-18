# 18장 3D Gaussian Splatting 기반 SLAM

iMAP과 NICE-SLAM은 신경망과 격자가 3D 공간을 저장할 수 있음을 입증했지만, 그 지도는 손대기 까다로운 블랙박스였다. iMAP에서는 새 관측으로 MLP를 조금만 고쳐도 다른 영역의 기억이 함께 무너졌고, NICE-SLAM은 지역 feature grid로 망각은 줄였지만 RTX 3090 Ti에서 잰 Replica 처리 속도가 1 fps에도 미치지 못해 실시간 SLAM이라 부르기 민망했다. 장면 전체가 네트워크 파라미터 속에 잠겨 있어 사람이 지도를 열어 보고 직접 다듬기도 어려웠다.

2023년 8월 SIGGRAPH에서 전환점이 찾아왔다. Bernhard Kerbl(Inria), Georgios Kopanas, Thomas Leimkühler, George Drettakis가 내놓은 3D Gaussian Splatting(3DGS) [논문](https://arxiv.org/abs/2308.04079)은 MLP도 복셀 격자도 버리고, 공간에 띄운 작은 반투명 타원체—Gaussian primitive 수백만 개—로 장면을 명시적으로 조립했다. 렌더링 기법은 Matthias Zwicker 등의 EWA splatting(2001)에서 빌렸고, 최적화 방식은 NeRF처럼 미분 가능한 렌더링(differentiable rendering)을 따랐다. 논문이 나오자마자 불과 6개월 사이에 이 표현을 심장으로 삼은 SLAM 논문들이 쏟아졌다.

---

## 18.1 3DGS의 구조

3DGS의 기본 단위인 Gaussian 하나는 네 가지 성질로 정의된다. 3차원 위치(mean) $\boldsymbol{\mu} \in \mathbb{R}^3$, 모양과 방향을 결정하는 공분산 행렬 $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, 학습되는 불투명도 $o \in (0,1)$, 그리고 시점마다 달라지는 빛깔을 담는 구면조화함수(spherical harmonics) 계수다. 최적화 도중 공분산 행렬이 양의 준정부호 조건을 잃지 않도록 실제 학습에서는 이를 스케일 벡터 $\mathbf{s}$와 단위 쿼터니언 $\mathbf{q}$로 쪼개어 관리한다. $\mathbf{S}$가 $\mathbf{s}$로 만든 대각 행렬, $\mathbf{R}$이 $\mathbf{q}$로 만든 회전 행렬일 때 공분산은 다음과 같이 조립된다.

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

장면을 그릴 때는 3D Gaussian들을 화면에 투영해 2D 타원으로 만든 뒤, 카메라로부터 가까운 순서대로 알파 블렌딩(alpha-blending)으로 포갠다. 각 Gaussian의 유효 불투명도 $\alpha_i$는 학습한 불투명도 $o_i$에 픽셀 좌표에서의 2D Gaussian 값 $G_i(\mathbf{x})$를 곱해 구하며, 최종 픽셀 색 $C$는 다음 식을 따른다.

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = o_i G_i(\mathbf{x})$$

속도의 비결은 GPU 래스터화(rasterization)에 있다. NeRF처럼 픽셀마다 쏜 광선을 따라 수십 개의 샘플을 찍고 MLP를 통과시키는 ray marching 적분이 아예 없다. Kerbl 등은 화면을 타일로 쪼개는 타일 기반 래스터라이저의 forward와 backward 패스를 모두 CUDA 커널로 직접 짰고, 그 결과 A6000 GPU에서 1080p 영상을 30 fps 이상으로 그려 냈다. 물론 이 수치는 이미 학습이 끝난 장면에서 새 시점을 그리는 렌더링 속도이므로, tracking과 지도 갱신을 함께 돌리는 NICE-SLAM의 전체 처리 속도와 같은 선에서 비교할 값은 아니다.

지도는 보통 SfM이 뱉어 낸 sparse point cloud로 첫 발을 뗀다. 학습이 진행되는 동안 3DGS는 **densification**이라 부르는 분열·복제·가지치기(splitting, cloning, pruning) 절차를 주기적으로 밟는다. 뷰 공간(view-space) 위치 그래디언트가 임계치를 넘겨 아직 모양이 덜 잡힌 것으로 판명된 영역 가운데, 작은 Gaussian이 빈틈을 채워야 할 곳에서는 같은 크기로 복제해 그래디언트 방향으로 밀어 놓고, 분산이 큰 덩어리 Gaussian이 세밀함을 뭉개고 있는 곳에서는 작은 Gaussian 둘로 쪼갠다. 반대로 불투명도가 바닥난 유령 Gaussian은 주기적으로 도려낸다.

> 🔗 **차용.** 3DGS가 3D Gaussian을 화면의 2D Gaussian으로 투영하는 수학적 틀은 Zwicker et al.의 [EWA splatting (2001)](https://www.cs.umd.edu/~zwicker/publications/EWAVolumeSplatting-VIS01.pdf)에서 왔다. Zwicker 등은 volume 데이터를 그리려고 샘플마다 타원형 Gaussian 커널을 화면에 투영했고, 3DGS는 이 커널의 파라미터를 역전파로 학습하게 만든 뒤 GPU 타일 래스터라이저로 속도를 끌어올렸다.

---

## 18.2 3DGS가 SLAM 지도에 맞았던 까닭

NeRF 기반 SLAM이 겪던 두 가지 고질병을 떠올리면 3DGS가 왜 그토록 빨리 SLAM 연구자들을 사로잡았는지 이해하기 쉽다. 단일 MLP는 catastrophic forgetting 탓에 지도를 조금씩 덧붙이기 어려웠고, NICE-SLAM의 voxel grid는 지역성을 회복한 대가로 해상도를 높일수록 메모리가 세제곱으로 불어났다.

반면 Gaussian은 공간의 정해진 좌표에 놓이는 독립된 알갱이다. 새 keyframe이 들어오면 새로 드러난 영역에만 Gaussian을 뿌리면 될 뿐, 멀리 떨어진 기존 영역의 가중치를 건드릴 이유가 없다. 3DGS의 densification 절차 자체가 keyframe을 하나씩 쌓아 가는 SLAM의 리듬과 자연스럽게 맞아떨어졌고, 렌더링 품질은 NeRF에 필적하면서도 속도는 비교할 수 없이 빨랐다.

---

## 18.3 GS-SLAM: 초기 시도

이 표현을 가장 먼저 SLAM의 tracking과 mapping 양쪽에 들이민 초기 시스템 가운데 하나가 Shanghai AI Laboratory와 HKUST의 Chi Yan 등이 2023년 11월 arXiv에 올린 [GS-SLAM](https://arxiv.org/abs/2311.11700)(CVPR 2024)이다.

GS-SLAM은 고전 SLAM의 이분 구조를 그대로 뼈대로 삼아, tracking이 현재 프레임의 카메라 자세를 풀고 mapping이 Gaussian 지도를 다듬는다. 여기에 두 가지 장치를 얹었다. 하나는 새 관측 영역에는 Gaussian을 보태되 표면 앞에 어정쩡하게 떠 있는 잡음 Gaussian의 불투명도를 과감히 낮추는 adaptive expansion이고, 다른 하나는 성기게 뽑은 픽셀로 먼저 대략의 자세를 잡은 뒤 신뢰할 만한 Gaussian만 추려 정밀 최적화로 들어가는 coarse-to-fine tracking이다.

tracking 단계에서는 렌더링한 영상과 실제 관측 영상의 픽셀 색 차이를 줄이는 photometric 손실로 자세를 푼다. 샘플링한 픽셀에 대한 손실 식은 다음과 같다.

$$\mathcal{L}_{track} = \sum_m \|\mathbf{C}_m - \hat{\mathbf{C}}_m\|_1$$

mapping 단계에서는 depth L1 오차와 color L1 오차의 가중합을 쓰며, 3DGS 원 논문에서 정립된 손실 $(1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$ ($\lambda=0.2$)은 이후 Gaussian 기반 SLAM 지도를 학습하는 거의 모든 시스템의 기본 공식이 됐다. 이 모든 손실의 그래디언트를 카메라 자세까지 되돌려 보낼 수 있는 것은 rasterizer가 미분 가능하기 때문이다.

Replica 데이터셋에서 GS-SLAM은 렌더링 PSNR을 차선책보다 1.52 dB 끌어올렸고, 전체 파이프라인이 RTX 4090에서 8.34 fps로 돌아갔다. 다만 저자들은 양질의 깊이 센서에 기대고 있다는 점과 장면이 커질수록 메모리가 가파르게 불어나는 점을 한계로 짚었다.

---

## 18.4 SplaTAM: silhouette 기반 densification

비슷한 시기 카네기멜론대의 Nikhil Keetha 등이 발표한 [Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126)은 새 Gaussian을 어디에 심을지 훨씬 단순하고 명쾌한 규칙으로 정했다. 렌더링된 Gaussian들이 픽셀을 얼마나 촘촘히 덮었는지 재는 **silhouette mask**를 도입해, silhouette 값이 0.5 미만인 빈 구멍이거나 센서로 잰 깊이가 렌더링 깊이보다 앞에 있으면서 오차가 큰 영역에만 새 Gaussian을 추가했다.

GS-SLAM과 마찬가지로 SplaTAM도 지도를 얼려 두고 카메라 자세를 맞추는 tracking과, 자세를 고정하고 Gaussian 파라미터를 조율하는 mapping을 번갈아 돌렸다. 결과는 눈에 띄었다. Replica 데이터셋에서 SplaTAM의 평균 PSNR은 34.11 dB에 달해, 같은 실험 표에 오른 NICE-SLAM의 24.42 dB를 큰 폭으로 앞섰다.

> 📜 **예언 vs 실제.** Keetha는 SplaTAM(2024)의 Limitations & Future Work에서 motion blur, 큰 depth noise, 급격한 회전에 취약하다는 점을 솔직히 적었다. 앞으로의 과제로는 OpenVDB 같은 자료구조를 써서 대규모 장면으로 넓히는 일과, 미리 알려진 카메라 내부 파라미터(intrinsics)나 dense depth 없이도 동작하게 만드는 일을 꼽았다. 같은 해 Matsuki의 MonoGS(CVPR 2024)가 단안 RGB 전용 시스템을 따로 내놓았고, intrinsics 없는 환경과 대규모 장면 문제는 2025년에도 여전히 연구가 진행 중이다.

> 🔗 **차용.** 수많은 과거 프레임을 모두 붙잡고 있지 않고 핵심 keyframe만 추려 지도를 다듬는 전략은 PTAM(Klein & Murray, 2007, 5장) 이래의 유산이다. SplaTAM은 mapping할 때 현재 프레임, 직전 keyframe, 그리고 현재 시야와 가장 많이 겹치는 이전 keyframe을 뽑아 함께 최적화한다.

---

## 18.5 MonoGS: monocular RGB

SplaTAM이 RGB-D 센서의 깊이 측정에 기댔다면, Imperial College Dyson Robotics Lab의 Hidenobu Matsuki가 내놓은 [Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741)는 깊이 센서 없이 단안(monocular) RGB 카메라 한 대로 3DGS SLAM을 밀어붙였다. Matsuki 역시 iMAP의 Sucar, CodeSLAM의 Bloesch와 같은 Davison 연구실 계보에 속한 연구자였다.

단안 영상만으로 Gaussian을 최적화하면 기하적 모호성이 곧장 흉측한 형태로 드러난다. 절대 scale이 정해지지 않을뿐더러, 시점이 충분히 확보되지 않은 곳에서는 Gaussian이 카메라 시선 방향을 따라 가늘고 길게 찌그러지면서 렌더링과 tracking을 동시에 망가뜨리기 때문이다. Matsuki는 Gaussian의 세 축 길이가 한쪽으로 과도하게 늘어지지 않도록 다음과 같은 등방성(isotropy) 정규화 항을 손실에 추가했다.

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

여기서 $\mathbf{s}_k \in \mathbb{R}^3$는 $k$번째 Gaussian의 스케일 벡터이고 $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$는 세 축 스케일의 평균이다. 그럼에도 단안 궤적 자체의 절대 scale 모호성은 남는다.

tracking은 GS-SLAM·SplaTAM과 같이 렌더링 영상과 관측 영상의 photometric loss로 카메라 자세를 직접 최적화한다. 깊이 센서가 없는 상태에서 새 Gaussian을 심을 때는, 기존 지도가 렌더링해 낸 깊이가 있는 픽셀에서는 그 깊이 주변에 작은 분산으로 초기화하고, 아직 지도가 없는 픽셀에서는 렌더링된 깊이들의 중앙값 주변에 큰 분산으로 띄워 둔 뒤 여러 시점의 관측이 쌓이면서 제자리를 찾아가게 했다.

[MonoGS의 2023년 첫 arXiv 원고](https://arxiv.org/abs/2312.06741v1)에 따르면 TUM-RGBD 데이터셋의 평균 ATE RMSE는 단안 설정에서 4.44 cm(정답 궤적에 scale을 맞춘 뒤 측정), RGB-D 설정에서는 1.58 cm(rigid alignment만 적용)였다. RGB-D 설정의 Replica 렌더링 품질은 평균 PSNR(peak signal-to-noise ratio, 높을수록 원본에 가깝다) 37.50 dB를 기록했다.

> 🔗 **구분.** MonoGS는 외부에서 미리 학습한 monocular depth 예측기나 별도의 tracking 모듈을 빌려 쓰지 않고, 순수하게 3DGS 표현 안에서 단안 SLAM을 완결했다.

---

## 18.6 RTG-SLAM과 실시간 처리

표현 품질이 입증되자 다음 화두는 자연스레 속도와 메모리로 옮겨 갔다. Zhejiang University의 Zhexi Peng 등이 발표한 [RTG-SLAM (SIGGRAPH 2024)](https://arxiv.org/abs/2404.19706)은 거실 수준을 넘어선 큰 장면을 실시간으로 RGB-D 재구성하는 데 초점을 맞췄다.

해법은 Gaussian의 역할을 극단적으로 가르는 것이었다. 모든 Gaussian을 애매한 반투명 상태로 여럿 겹쳐 놓는 대신, 각 Gaussian이 완전히 불투명하거나 거의 투명하도록 강제했다. 불투명한 Gaussian 하나가 표면 한 조각과 대표 색상을 딱 덮어 버리면 그 뒤에 다른 Gaussian을 겹쳐 쌓을 필요가 없으므로, 전체 Gaussian 개수와 연산량이 뚝 떨어진다. 여기에 Gaussian을 안정 상태와 불안정 상태로 나누어 불안정한 것만 최적화하고 그들이 덮는 픽셀만 선택적으로 렌더링한 결과, 논문은 동시대 최신 NeRF 기반 RGB-D SLAM보다 약 2배 빠르면서 메모리 사용량은 절반 수준으로 줄였다고 보고했다.

---

## 18.7 3DGS 이후 달라진 표현 선택

2024년 이후 3DGS 기반 SLAM 논문이 쏟아지면서 SLAM 엔지니어가 고를 수 있는 지도 표현의 스펙트럼은 어느 때보다 넓어졌다. 물론 모든 로봇이 3DGS로 갈아탄 것은 아니다. 임베디드 보드 위에서 경로를 계획하고 충돌을 피해야 하는 실전 로보틱스에서는 지금도 TSDF와 occupancy grid가 주력이다. 예컨대 NVIDIA의 [nvblox](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html)는 Jetson 같은 소형 장치에서 실시간으로 TSDF 지도를 쌓고, 이를 곧바로 경로 계획용 거리장(ESDF)과 cost map으로 변환한다.

그럼에도 3DGS가 이토록 빠르게 번진 밑바탕에는 하드웨어 궁합이라는 현실적인 동력이 있었다. 현대 GPU는 수십 년간 삼각형과 스플랫을 화면에 뿌리는 래스터화 파이프라인에 맞춰 진화해 왔는데, ray marching 기반인 NeRF와 달리 3DGS는 바로 그 GPU의 장기인 래스터화를 그대로 탔기 때문이다.

> 🔗 **차용.** 장면 표현을 그래디언트로 최적화하고 photometric loss로 관측과 렌더링을 맞대는 패러다임 자체는 NeRF(Mildenhall et al. 2020)에서 물려받은 것이다. 3DGS는 그 최적화 문법은 그대로 유지한 채, 지도의 그릇만 implicit MLP에서 명시적 Gaussian 집합으로 바꿔 끼웠다.

> 📜 **예언 vs 실제.** Kerbl et al.은 3DGS(2023) 원 논문의 Limitations에서 관측이 빈약한 곳의 아티팩트, 길쭉하게 늘어지는(elongated) 아티팩트, 시점 이동 시 튀는 popping 현상, 별도의 정규화(regularization) 부재, 그리고 막대한 메모리 소비(큰 장면을 학습할 때 20 GB를 넘기고 렌더링에도 수백 MB가 듦)를 약점으로 꼽았다. 개선 방향으로는 antialiasing, 원칙적인 culling, point cloud 압축 기법의 도입을 들었다. 실제로 메모리 압축 문제는 2024년 [Compact 3DGS](https://arxiv.org/abs/2311.13681)와 [Niedermayr et al.](https://arxiv.org/abs/2401.02436) 등이 곧바로 파고들었다. 반면 원 논문이 아예 언급하지 않았던 동적 장면 확장([4DGS](https://arxiv.org/abs/2310.08528), [Deformable 3DGS](https://arxiv.org/abs/2309.13101))과 3D 생성·편집([DreamGaussian](https://arxiv.org/abs/2309.16653), [GaussianEditor](https://arxiv.org/abs/2311.14521))은 2024년을 전후해 아예 독립된 거대 연구 분야로 자라났다.

---

## 아직 열린 문제

**Memory scaling.** Gaussian 개수는 장면이 넓어지는 만큼 정직하게 늘어난다. 실내 Replica 벤치마크에서 도시 구역이나 대규모 캠퍼스로 나가면 필요한 Gaussian 수가 감당하기 어렵게 커진다. 불필요한 Gaussian을 쳐내는 pruning과 거리에 따라 해상도를 조절하는 level-of-detail 계층 구조가 활발히 연구되고 있지만, 넓은 환경에서 메모리 효율과 렌더링 화질을 동시에 잡는 일은 여전히 미완이다.

**Semantic 통합.** 2023년 [LERF](https://arxiv.org/abs/2303.09553)가 NeRF에 언어 feature를 심은 데 이어, [LangSplat](https://arxiv.org/abs/2312.16084)이 Gaussian 표현에 언어 feature를 얹으며 의미 이해와 3DGS가 만났다. 이를 SLAM으로 끌어온 연구도 잇따르고 있으나, 실시간 지도 갱신 속도와 tracking 안정성, semantic 정확도를 동일 조건에서 고르게 만족시키는 수준에는 아직 이르지 못했다.

**Dynamic scene.** 4DGS나 Deformable 3DGS처럼 Gaussian에 시간 축을 붙이는 오프라인 기법은 나왔지만, 정작 GS-SLAM(Yan et al. 2024), SplaTAM(Keetha et al. 2024), MonoGS(Matsuki et al. 2024)를 비롯한 초기 3DGS SLAM 시스템은 예외 없이 세계가 꼼짝 않고 정지해 있다고 가정한다(→ 19장).

여기에 더해 초기 Gaussian을 어디서 구할 것인가 하는 문제도 시스템마다 제각각이었다. RGB-D 방식은 센서가 찍어 준 깊이에 기대고, MonoGS는 시스템 내부에서 추정한 깊이로 더듬어 나갔으며, 뒷장에서 볼 DUSt3R 계열은 아예 이미지에서 기하를 직접 출력하는 네트워크를 학습했다. 그러나 표현이 explicit하든 implicit하든, 16장에서 18장까지 살펴본 시스템들은 하나같이 "카메라만 움직이고 세상은 멈춰 있다"는 오랜 가정 위에 서 있었다. 이제 그 가장 오래된 전제가 깨지는 현장을 들여다볼 차례다.
