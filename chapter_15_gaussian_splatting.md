# Ch.15 — Gaussian Splatting 시대: 3DGS에서 GS-SLAM까지

iMAP과 NICE-SLAM은 MLP로 공간을 기억하는 방법의 가능성을 보여주었다. 그러나 표현을 직접 편집하기는 어려웠다. iMAP에서는 새 관측에 대한 전역 MLP 갱신이 다른 영역에도 영향을 주었고, NICE-SLAM은 지역 feature grid로 이 문제를 완화했다. NICE-SLAM의 RTX 3090에서 1fps 아래 처리 속도는 "실시간 SLAM"이라는 말과 공존하기 어려운 수치였다. 장면은 네트워크 파라미터 안에 갇혀 있었고, 그 안을 들여다볼 방법이 없었다.

2023년 8월 SIGGRAPH에서 Bernhard Kerbl(INRIA), Georgios Kopanas, Thomas Leimkuhler, George Drettakis가 [논문](https://arxiv.org/abs/2308.04079)을 발표했다. Kerbl은 NeRF가 3년에 걸쳐 쌓은 implicit representation 패러다임을 버리지 않으면서, 다른 선택을 했다. iMAP·NICE-SLAM·Co-SLAM이 MLP와 voxel grid로 장면을 잠그는 동안, Kerbl은 수백만 개의 작은 타원체, 즉 Gaussian primitive를 공간에 흩뿌렸다. 이후 6개월 동안 이 표현을 활용한 SLAM 논문들이 잇따라 나왔다. Kerbl의 선택은 Matthias Zwicker의 EWA splatting(2001)이라는 20년 된 그래픽스 기법을 뿌리로 삼고, NeRF의 differentiable rendering 정신은 그대로 계승했다. 차이는 표현의 형식에 있었다.

---

## 3DGS의 구조

Kerbl은 장면을 explicit한 Gaussian 집합으로 표현했다. 각 Gaussian은 위치(mean) $\boldsymbol{\mu} \in \mathbb{R}^3$, 공분산 행렬 $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, 학습되는 불투명도 $o \in (0,1]$, 그리고 구면조화함수(spherical harmonics) 계수로 표현된 색상을 가진다. 공분산은 학습 안정성을 위해 스케일 벡터 $\mathbf{s}$와 단위 쿼터니언 $\mathbf{q}$로 분해한다:

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

렌더링은 projected 2D Gaussian을 깊이 순서대로 알파 블렌딩(alpha-blending)한다. 각 Gaussian의 유효 불투명도 $\alpha_i$는 학습되는 불투명도 $o_i$와 픽셀 위치에서 평가된 2D Gaussian $G_i(\mathbf{x})$의 곱이다. 이 장에서 학습되는 불투명도를 $o_i$로 쓰면 픽셀 색상 $C$는

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = o_i G_i(\mathbf{x})$$

Volume rendering 적분을 수치적으로 근사하는 NeRF와 달리, 3DGS는 GPU 래스터화(rasterization) 파이프라인에 직접 올라간다. 타일 기반 래스터라이저는 forward pass와 backward pass 모두 커스텀 CUDA 커널로 구현했다. RTX 3090 한 장에서 30fps 이상으로 동작했다. 이 수치는 학습 뒤 novel-view 렌더링의 처리량이다. 추적과 지도 갱신을 포함한 NICE-SLAM의 전체 SLAM 처리량과 직접 배율로 비교할 수는 없다.

초기화는 SfM에서 나온 sparse point cloud를 사용한다. 이후 학습 과정에서 Gaussian을 분열(splitting), 복제(cloning), 제거(pruning)하는 **densification** 절차를 반복한다. 뷰 공간(view-space) 위치 그래디언트가 임계치를 넘으면, scale이 큰 Gaussian은 두 자식으로 분열(split)하고 scale이 작은 Gaussian은 동일 위치에 복제(clone)한다. 불투명도가 낮은 Gaussian은 주기적으로 제거한다.

> 🔗 **차용.** 3DGS의 래스터화 기반 splatting은 Zwicker et al.의 [EWA splatting (2001)](https://www.cs.umd.edu/~zwicker/publications/EWAVolumeSplatting-VIS01.pdf)을 직접 계승한다. Zwicker는 점 구름을 렌더링하기 위해 각 점에 타원형 가중 평균 커널을 씌웠다. Kerbl은 그 커널을 learnable Gaussian으로 교체하고 GPU 타일 래스터라이저로 가속했다.

---

## 3DGS와 SLAM의 구조적 적합성

implicit representation은 SLAM에 적용할 때 여러 제약이 있었다. MLP 기반 NeRF는 새 관측이 들어올 때마다 전체 네트워크를 재학습해야 했고, catastrophic forgetting 탓에 incremental update가 어려웠다. 지도 확장은 네트워크 크기 재조정을 뜻했다. NICE-SLAM의 voxel grid는 이 문제를 완화했지만, 해상도와 메모리의 트레이드오프를 피할 수 없었다.

Gaussian은 공간에 명시적으로 있는 객체여서, 새 키프레임이 들어오면 해당 영역에 Gaussian을 추가하기만 하면 된다. Densification 절차가 keyframe 추가와 자연스럽게 맞물렸고, 렌더링 품질은 NeRF 수준을 유지했다. 실시간 처리도 가능했다. 2023년 후반부터 GS-SLAM 논문들이 잇따라 나왔다.

---

## GS-SLAM: 초기 시도

Chi Yan(홍콩대)과 공동 연구자들은 2023년 11월 [Yan et al. 2023. GS-SLAM](https://arxiv.org/abs/2311.11700)을 arXiv에 게시했다. 비슷한 시기에 여러 3DGS-SLAM 원고가 공개된 가운데, 3DGS를 tracking과 mapping에 결합한 초기 시스템 중 하나였다.

GS-SLAM의 구조는 전통 SLAM 프레임워크를 따른다. Tracking은 현재 프레임의 포즈를 추정하고, Mapping은 Gaussian 지도를 갱신한다. Yan의 기여는 두 가지였다. adaptive Gaussian expansion은 새 키프레임이 추가될 때 coverage가 낮은 영역에 Gaussian을 삽입한다. geometry-aware Gaussian selection은 렌더링 손실 역전파 시 기여가 큰 Gaussian만 골라 최적화해 속도를 확보한다.

Tracking은 포즈를 렌더링 포토메트릭 손실로 최적화한다. GS-SLAM의 tracking 손실은 sampled pixel에 대한 L1 색 손실이다:

$$\mathcal{L}_{track} = \sum_m \|\mathbf{C}_m - \hat{\mathbf{C}}_m\|_1$$

Mapping 단계에서 Yan은 color L1과 depth L1을 가중합해 쓴다. 한편 3DGS 원 논문의 training 손실인 $(1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$ ($\lambda=0.2$) 조합은 Gaussian 지도를 학습할 때 상속되는 기본형이다. 이 손실을 포즈에 대해 미분할 수 있는 이유가 3DGS의 differentiable rasterizer다.

Replica 데이터셋에서 NICE-SLAM 대비 PSNR을 유지하면서 처리 속도를 높였다. 한계도 명확했다. RGB-D 카메라를 전제했고, 실외 대규모 환경에서는 검증하지 않았다.

---

## SplaTAM: silhouette 기반 densification

Nikhil Keetha(카네기멜론대)의 [Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126)은 설계 철학에서 GS-SLAM과 달랐다. Keetha는 복잡한 선택 메커니즘 대신 실루엣 마스크 기반 단순한 densification을 택했다.

Keetha는 **silhouette mask**를 이용해 현재 뷰에서 기존 Gaussian으로 설명되지 않는 영역을 찾고, 렌더링된 마스크의 빈 부분에 새 Gaussian을 추가했다. Gaussian이 없는 위치를 찾아 채우는 규칙이다.

Tracking은 포즈, Mapping은 Gaussian 파라미터를 각각 최적화한다. 추적할 때는 지도를 고정하고, 지도 갱신 단계에서 Gaussian을 조정한다. GS-SLAM도 pose와 지도 변수를 나누므로, 분리 자체를 SplaTAM만의 차이로 볼 수는 없다.

> 🔗 **차용.** SplaTAM의 키프레임 기반 지도 관리 구조는 PTAM(Klein & Murray, 2007)의 아이디어가 새 표현 위에서 재동작하는 사례다. PTAM이 keyframe을 선택적으로 삽입해 지도를 유지하던 방식이, SplaTAM에서는 Gaussian densification의 트리거로 변환되었다.

Replica 데이터셋 기준으로 SplaTAM은 PSNR 34.11 dB를 기록했다. 같은 논문의 표에서 NICE-SLAM은 24.42 dB였다. 렌더링 품질 격차는 명확했다.

Keetha는 2024 CVPR 논문의 Limitations & Future Work에서 motion blur·depth noise·공격적 회전에 대한 민감성, 그리고 known intrinsics와 dense depth 의존을 제거하는 방향을 다음 과제로 들었다. 확장성 개선도 언급했다.

> 📜 **예언 vs 실제.** Keetha는 SplaTAM(2024) 한계 절에서 motion blur·depth noise·공격적 회전 민감성, 그리고 known intrinsics/dense depth 의존 제거를 명시적 과제로 제시했다. 같은 해 Matsuki의 MonoGS(2024 CVPR)는 별도의 단안 RGB 시스템을 제시했다. intrinsics-free와 대규모 스케일 이슈는 2024-2025년 현재도 진행 중이다.

---

## MonoGS: monocular RGB

Hidenobu Matsuki(Imperial College Dyson Robotics Lab)의 [Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741)는 depth sensor 없이 단안(monocular) RGB 카메라만으로 3DGS SLAM을 구동했다.

monocular 설정에서는 scale이 문제다. depth 없이 절대 메트릭 스케일을 복원하는 것은 SfM에서도 풀리지 않은 문제다. Matsuki는 Gaussian geometry를 직접 최적화하고 등방성 손실을 추가했지만, 단안 궤적 자체는 scale-free다.

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

여기서 $\mathbf{s}_k \in \mathbb{R}^3$는 k번째 Gaussian의 스케일 벡터, $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$는 세 축 스케일의 평균이다. 이 등방성(isotropy) 정규화는 Gaussian이 지나치게 얇은 판 형태로 퇴화하는 것을 막는다. monocular에서 depth 감독 없이 Gaussian이 카메라 평면에 달라붙는 현상을 억제한다.

Tracking에서 Matsuki는 포즈를 Gaussian 렌더링 photometric loss로 직접 최적화했다. 새 Gaussian을 만들 때 해당 픽셀에 렌더링된 depth가 있으면 이를 쓰고, 없으면 렌더링된 depth의 중앙값을 사용한다. 이 절차와 등방성 손실은 내부 지도의 상대적 스케일을 일관되게 유지하지만, 절대 메트릭 스케일을 복원하지는 않는다.

> 🔗 **구분.** MonoGS는 pretrained monocular depth predictor나 외부 tracking 모듈을 사용하지 않는다. 따라서 MonoDepth2를 직접 차용했다는 계보는 논문에서 확인되지 않는다.

Matsuki는 Imperial Dyson Robotics Lab 출신이다. Davison이 지도한 연구실에서 나온 Sucar(iMAP), Bloesch(CodeSLAM)와 같은 계보다. Lab의 관심이 implicit MLP → Gaussian explicit 표현으로 이동한 흐름을 MonoGS가 대표한다.

TUM-RGBD 데이터셋에서 MonoGS는 monocular 평균 ATE RMSE 4.44cm, RGB-D 1.58cm를 기록했다. 단안 ATE는 정답 궤적에 scale alignment를 한 뒤 평가한 값이고, RGB-D는 rigid alignment만 적용했다. 렌더링 품질은 RGB-D 설정 Replica 기준 평균 PSNR 37.50 dB로 동세대 GS-SLAM 계열과 견줄 만한 수준이었다.

---

## RTG-SLAM과 실시간 처리

GS-SLAM 계열이 풀어야 할 다음 문제는 속도였다. GS-SLAM과 SplaTAM은 실시간이라 부르기 어려웠다. Zhejiang University의 Peng Zhexi 연구팀이 2024년 발표한 [RTG-SLAM (SIGGRAPH 2024)](https://arxiv.org/abs/2404.19706)은 명시적으로 실시간을 목표로 삼았다.

RTG-SLAM의 전략은 Gaussian의 수를 제어하는 것이다. 현재 카메라 뷰에서 기여도가 큰 Gaussian만 골라 최적화한다. Gaussian을 surfel(표면 원소) 기반으로 초기화해 geometry를 유지하면서 수를 줄였다. Replica 데이터셋에서 실시간에 근접한 처리 속도를 냈다.

---

## 3DGS 이후 달라진 표현 선택

2024년 이후 3DGS 논문이 빠르게 늘면서 SLAM mapping 연구의 표현 선택지가 넓어졌다. TSDF·occupancy grid는 embedded·planning·안전 응용에서 계속 쓰이고, NeRF와 3DGS도 목표에 따라 선택된다. 3DGS 논문들은 렌더링 속도와 명시적 primitive 갱신을 장점으로 내세웠지만, 이 흐름만으로 다른 표현이 주류에서 사라졌다고 단정할 수는 없다.

이것은 표현의 전환이면서 하드웨어 친화성의 전환이다. GPU 래스터라이저는 GPU 레이마처(ray marcher)보다 훨씬 잘 최적화되어 있다. 3DGS가 기존 그래픽스 파이프라인 위에서 자연스럽게 돌아간다는 점이 NeRF 대비 채택 속도를 높였다.

> 🔗 **차용.** 3DGS의 differentiable rendering 정신은 NeRF에서 직접 계승한다. 장면 표현을 gradient로 최적화한다는 아이디어, photometric loss로 관측과 렌더링을 연결하는 방식은 Mildenhall et al.(2020)의 유산이다. Kerbl은 표현(implicit MLP → explicit Gaussian)을 교체하면서 패러다임은 계승했다.

> 📜 **예언 vs 실제.** Kerbl et al.은 3DGS(2023) §7.4 Limitations에서 관측이 부족한 영역의 elongated artifact와 popping, 정규화(regularization) 부재, 메모리 소비(훈련 중 20GB 초과, 대규모 씬 렌더링 시 수백 MB)를 한계로 꼽았다. Future work로는 antialiasing, 더 원칙적인 culling, point-cloud 압축 기법 차용을 제안했다. 메모리 축(압축)은 [Compact 3DGS](https://arxiv.org/abs/2311.13681) 계열과 [Niedermayr et al.](https://arxiv.org/abs/2401.02436)이 2024년에 직접 응답했다. dynamic scene 확장([4DGS](https://arxiv.org/abs/2310.08528), [Deformable 3DGS](https://arxiv.org/abs/2309.13101))과 생성·편집([DreamGaussian](https://arxiv.org/abs/2309.16653), [GaussianEditor](https://arxiv.org/abs/2311.14521))은 원 논문이 직접 거론하지 않은 영역이지만 2024년 전후로 별도 계통으로 갈라져 나왔다.

---

## 🧭 아직 열린 것

**Memory scaling.** Gaussian의 수는 장면 크기에 따라 선형으로 증가한다. 실내 Replica 데이터셋에서 수십만 개로 충분하던 것이 outdoor 도시 구역에서는 수천만 개로 늘어난다. Gaussian pruning과 level-of-detail 계층화가 연구되고 있지만, 대규모 환경에서 메모리와 렌더링 품질의 트레이드오프를 합리적으로 관리하는 방법은 아직 합의가 없다. Compact 3DGS 계열(Lee et al. 2024, Niedermayr et al. 2024)이 압축 방향을 탐색 중이다.

**Semantic 통합.** 2023년 [LERF](https://arxiv.org/abs/2303.09553)가 NeRF에 언어 feature를 결합했고, [LangSplat](https://arxiv.org/abs/2312.16084)은 Gaussian 표현에 언어 feature를 결합했다. 이후 semantic Gaussian을 SLAM에 결합한 연구도 이어졌지만, 서로 다른 장면과 장비에서 실시간 갱신·tracking 품질·semantic 정확도를 함께 비교할 공통 protocol은 정착되지 않았다. semantic과 geometry를 공동 최적화할 때의 interference가 핵심 평가 항목이다.

**Dynamic scene.** 4DGS와 Deformable 3DGS는 시간 차원을 Gaussian에 추가하는 방향을 제안했다. SLAM 설정에서 dynamic object는 배경과 다른 움직임을 가지므로 별도로 처리해야 한다. GS-SLAM(Yan et al. 2023), SplaTAM(Keetha et al. 2024), MonoGS(Matsuki et al. 2024) 모두 정적 세계 가정을 유지한다. Dynamic SLAM에서 Gaussian이 어떻게 이동하는 객체를 표현하고 추적할 것인가는 2025년 기준으로 열려 있다.

그러나 3DGS가 남긴 또 다른 질문이 있었다. Gaussian은 SfM point cloud에서 오는가, 아니면 depth sensor에서 오는가. 포즈를 알아야 Gaussian을 놓을 수 있고, Gaussian이 있어야 포즈를 추정할 수 있다. 이 닭-달걀 문제에 대응하는 초기화는 시스템마다 달랐다. RGB-D 방법은 측정 깊이를 활용했고, MonoGS는 외부 depth predictor 없이 내부에서 초기 깊이 가설을 만들었다. Ch.16에서 다루는 DUSt3R와 그 후계들은 다른 출발점을 선택했다. geometry 자체를 처음부터 학습하는 길이다.
