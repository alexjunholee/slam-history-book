# Ch.14 — NeRF를 SLAM 지도로: iMAP→NICE-SLAM

DROID-SLAM은 학습한 표현을 SLAM의 tracking에 넣었다. CNN과 GRU가 feature와 대응을 만들고, 그 결과로 자세를 최적화했다. Imperial College의 Edgar Sucar와 동료들은 2021년 iMAP에서 학습한 표현을 *지도*로 썼다. 그 표현은 SLAM 밖의 NeRF에서 왔다.

2020년 3월, Ben Mildenhall과 동료들이 arXiv에 올린 [Mildenhall et al. 2020. NeRF](https://arxiv.org/abs/2003.08934)는 자세를 아는 여러 입력 영상으로 연속 volumetric scene function을 최적화해, 보지 않은 시점의 영상을 합성하는(novel-view synthesis) 방법이었다. 2021년 ICCV에 나온 iMAP은 이런 표현으로 지도와 자세를 함께 추정했다. KinectFusion(Ch.9)이 풀던 dense mapping을 implicit neural field(신경망으로 표현한 연속 함수)로 다시 푼 초기 시스템이다.

NeRF 이전인 2019년에도 좌표를 입력받는 MLP(coordinate-based MLP)로 3D를 표현하는 연구가 잇달아 나왔다. [Park et al.의 DeepSDF](https://arxiv.org/abs/1901.05103)는 좌표를 넣으면 signed distance를 출력하는 MLP로 물체 표면을 암묵적으로 기술했다. [Mescheder et al.의 Occupancy Networks](https://arxiv.org/abs/1812.03828)는 같은 좌표 입력에서 occupancy 확률을 출력하게 했다. [Sitzmann et al.의 SRN](https://arxiv.org/abs/1906.01618)은 좌표를 넣으면 scene feature vector를 내는 MLP를 두고, 미분 가능한 ray marching으로 이미지를 합성했다. 이 연구들은 모두 좌표를 넣으면 field 값이 나오는 함수를 학습했다. NeRF는 여기에 volume rendering 적분과 positional encoding을 더해 view synthesis에 썼다. iMAP 논문은 DeepSDF와 Occupancy Networks를 참고문헌으로 인용한다.

---

## NeRF: MLP 기반 공간 표현

NeRF는 MLP 하나로 3D 공간을 표현한다. 공간 좌표 $(x, y, z)$와 시선 방향 $(\theta, \phi)$를 넣으면 그 위치의 색상 $(r, g, b)$와 밀도 $\sigma$를 출력한다. 이 출력을 광선마다 적분해 장면을 렌더링한다.

렌더링은 volume rendering 방정식으로 이루어진다. 카메라 원점 $\mathbf{o}$에서 방향 $\mathbf{d}$로 나간 광선을 $t$ 매개변수로 샘플링한다:

$$\hat{C}(\mathbf{r}) = \int_{t_n}^{t_f} T(t)\,\sigma\!\left(\mathbf{r}(t)\right) \mathbf{c}\!\left(\mathbf{r}(t), \mathbf{d}\right)\, dt$$

여기서 $T(t) = \exp\!\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s))\, ds\right)$는 광선이 $t$까지 도달할 확률, 즉 누적 투과율이다. 실제로는 광선 위 샘플들로 이 적분을 구간별 합으로 근사한다.

> 🔗 **차용.** Volume rendering 방정식은 [Kajiya & Von Herzen(1984)](https://courses.cs.duke.edu/cps296.8/spring03/papers/RayTracingVolumeDensities.pdf)의 그래픽스 논문에서 왔다. NeRF 논문은 이 식을 미분 가능하게 계산해, 렌더링한 색과 실제 픽셀의 차이로 MLP를 학습했다.

좌표를 그대로 넣은 MLP는 저주파 성분에 치우쳐 학습한다. NeRF 논문은 positional encoding으로 이 문제를 다뤘다. 좌표 $(x, y, z)$를 여러 주파수의 사인·코사인 값으로 바꿔 넣으면 네트워크가 세밀한 텍스처와 날카로운 경계까지 학습한다:

$$\gamma(p) = \left(\sin(2^0 \pi p),\, \cos(2^0 \pi p),\, \ldots,\, \sin(2^{L-1} \pi p),\, \cos(2^{L-1} \pi p)\right)$$

> 🔗 **차용.** positional encoding은 NeRF 원 논문에 이미 있었다. 같은 해 [Tancik et al.(2020)](https://arxiv.org/abs/2006.10739) "Fourier Features Let Networks Learn High Frequency Functions"가 NTK(neural tangent kernel) 이론으로 이 기법의 작동 원리를 설명했다.

NeRF는 자세를 아는 이미지와 같은 시점에서 렌더링한 결과를 비교해 픽셀 색의 제곱 오차 합을 줄인다. 최적화가 끝나면 장면의 geometry와 appearance가 MLP 가중치에 담긴다.

원래 NeRF는 장면 하나를 학습하는 데 V100 GPU 한 장으로 1~2일이 걸렸다. 장면마다 따로 학습해야 했고, 카메라 자세는 COLMAP 같은 SfM으로 미리 구해야 했다. SLAM에 쓰려면 자세 추정과 지도 학습을 실시간에 가깝게 함께 해야 한다.

---

## iMAP: 초기 neural implicit SLAM

Imperial College Dyson Robotics Lab의 Edgar Sucar는 2021년 ICCV에 [Sucar et al. 2021. iMAP](https://doi.org/10.1109/ICCV48922.2021.00617)을 발표했다. **iMAP**(Implicit MAP)은 손에 든 RGB-D 카메라 입력으로 MLP 하나를 실시간에 지도로 학습하면서 자세를 함께 최적화했다. 논문 저자는 Sucar, Shikun Liu, Joseph Ortiz, Andrew Davison이다.

최적화 루프 두 개가 병렬로 돈다. *mapping* 루프는 MLP와 keyframe 몇 장의 자세를 함께 업데이트한다. 라이브 시스템은 매 반복마다 과거 keyframe 3장을 손실 분포에 비례해 뽑고, 마지막 keyframe과 현재 프레임을 더해 5장으로 최적화했다. 픽셀도 이미지당 200개만 뽑아, 손실이 큰 영역에 더 많이 배정했다. *tracking* 루프는 MLP를 고정하고 현재 프레임의 자세를 렌더링 손실로 최적화한다. 두 루프는 같은 MLP를 쓴다. 논문은 tracking 10 Hz, 전역 지도 갱신 2 Hz를 보고했다.

손실 함수는 색상 손실 $\mathcal{L}_{\text{color}} = \|\hat{C} - C\|_1$과 깊이 손실 $\mathcal{L}_{\text{depth}} = \|\hat{D} - D\|_1$로 이루어진다. 깊이 손실은 광선을 따라 계산한 depth 분산으로 정규화해, 물체 경계처럼 불확실한 영역의 비중을 낮춘다.

iMAP은 MLP 하나를 실시간 SLAM의 유일한 장면 표현으로 썼다. 논문은 keyframe 10~20장과 1 MB 정도의 MLP로 방 하나를 지도화했다고 적었다. 새 영역을 학습하면 이전 영역의 표현이 망가질 수 있어서, 선택한 keyframe을 기억 저장소처럼 계속 다시 학습(replay)했다.

> 📜 **예언 vs 실제.** Sucar는 iMAP 논문 Conclusion에서 "future directions for iMAP include how to make more structured and compositional representations that reason explicitly about the self similarity in scenes"라고 적었다. iMAP이 arXiv에 올라온 지 9개월 뒤(2021년 12월) 나온 NICE-SLAM은 multi-resolution voxel feature grid로 공간을 계층적으로 나눴다. 2023년 Wang et al.의 Co-SLAM은 hash grid와 coordinate encoding을 함께 썼다.

---

## NICE-SLAM: 계층 격자와 scalability

iMAP의 단일 MLP 문제를 직접 다룬 연구는 ETH Zurich의 Zihan Zhu와 Songyou Peng이 2022년 CVPR에서 발표한 [Zhu et al. 2022. NICE-SLAM](https://arxiv.org/abs/2112.12130)이다. **NICE-SLAM**(Neural Implicit Scalable Encoding for SLAM)은 단일 MLP 대신 multi-resolution voxel feature grid와 작은 MLP decoder를 결합했다.

공간을 명시적 복셀 격자로 나누고 각 복셀에 학습 가능한 feature vector를 둔다. 렌더링 시 샘플 좌표 주변 복셀들의 feature를 trilinear interpolation으로 결합한 뒤 작은 MLP에 통과시켜 색상과 occupancy를 얻는다. 공간 정보 대부분이 격자에 담기므로 MLP는 작아도 된다.

NICE-SLAM은 세 단계 해상도 격자를 계층적으로 쌓았다. 거친 격자부터 세밀한 격자까지 geometry를 여러 수준으로 표현하고, 색상은 별도의 color feature grid와 decoder로 표현한다. 논문은 시야 안에 든 격자 feature만 최적화하는 이런 지역 갱신 덕분에 iMAP과 같은 forgetting 문제를 겪지 않는다고 적었다. geometry decoder 세 개는 ConvONet 방식으로 미리 학습해 고정했다.

tracking에서는 격자와 decoder를 고정하고 자세만 최적화했다. mapping에서는 격자 feature를 업데이트했다. 논문은 여러 방으로 된 아파트를 직접 촬영해 재구성했다. 같은 픽셀 수로 속도를 재면 tracking은 iMAP보다 2배 넘게, mapping은 3배 넘게 빨랐다(101→47 ms, 448→130 ms).

다만 3D 격자의 메모리는 장면 한 변 길이의 세제곱으로 늘었다. 이 점은 ESLAM 논문이 짚었다. 전체 시스템 속도는 Co-SLAM 논문 측정으로 Replica에서 0.91 Hz였다.

Thomas Müller와 동료들의 [Müller et al. 2022. Instant-NGP](https://nvlabs.github.io/instant-ngp/)는 2022년 SIGGRAPH에서 발표됐다. multi-resolution hash encoding과 GPU에 맞춘 구현으로 학습 시간을 크게 줄였고, 공개 구현은 지원되는 NVIDIA GPU의 일부 데모 장면에서 수 초 훈련을 시연했다. SLAM 논문은 아니지만 이후 여러 NeRF-SLAM 시스템이 이 hash encoding을 썼다.

> 🔗 **차용.** NICE-SLAM의 multi-resolution feature grid와 Instant-NGP의 hash encoding은 비슷한 시기에 따로 나왔다(arXiv 2021년 12월, 2022년 1월). Co-SLAM은 hash grid를 썼다. 격자에 TSDF 값 대신 feature를 저장한다는 점에서 KinectFusion(Ch.9)의 voxel 격자와도 이어진다.

---

## Co-SLAM과 NeRF-SLAM

2022년 말부터 나온 시스템 가운데 일부는 implicit 표현 자체를 더 효율적으로 만들었다. 다른 일부는 기존 SLAM의 추정기를 NeRF 지도와 결합했다.

UCL의 Wang et al.이 낸 [**Co-SLAM**](https://arxiv.org/abs/2304.14377)(CVPR 2023)은 표현을 효율화했다. multi-resolution hash grid(parametric encoding)와 one-blob coordinate encoding을 함께 썼다. hash grid는 빠르게 수렴하고 국소 세부를 담는다. one-blob encoding은 표면을 매끄럽게 잇고 관측하지 않은 영역을 채우게 돕는다. 논문은 10~17 Hz를 보고했다(Replica 15~17 Hz, ScanNet·TUM 12~13 Hz).

같은 해 같은 CVPR에서 Idiap/EPFL의 Johari et al.의 [**ESLAM**](https://arxiv.org/abs/2211.11704)은 3D feature grid 대신 multi-scale axis-aligned feature plane을 써 메모리 증가를 $O(n^3)$에서 $O(n^2)$로 낮췄다. volume density 대신 TSDF를 decoding 목표로 삼았고, NICE-SLAM보다 최대 10배 빠르면서 사전 학습도 필요 없다고 보고했다.

MIT의 Antoni Rosinol, John Leonard, Luca Carlone이 2022년에 공개한 [**NeRF-SLAM**](https://arxiv.org/abs/2210.13641)(IROS 2023)은 기존 추정기를 썼다. 단안 영상만 받아 DROID-SLAM 방식으로 자세와 dense depth를 추정하고, 그 최적화에서 주변 공분산(marginal covariance)을 추가로 계산했다. 별도의 Instant-NGP 기반 mapping 모듈은 이 추정값과 불확실성을 받아 radiance field를 학습했다.

> 🔗 **차용.** NeRF-SLAM은 DROID-SLAM의 recurrent update와 Dense Bundle Adjustment가 만든 pose·depth 추정을 가져오고, 지도 표현에 Instant-NGP를 결합했다. radiance field는 자세 추정에 쓰이지 않는다. 기존 추정기의 출력을 받아 실시간으로 dense 지도를 만든다.

---

## iMAP의 구조적 한계

단일 MLP에는 지역성(locality)이 없다. 공간의 어느 부분을 렌더링하든 MLP 전체를 거친다. 그래서 새 영역을 학습하면 가중치 전체가 바뀌어 기존 영역의 표현이 망가질 수 있다(catastrophic forgetting). NICE-SLAM 논문은 장면이 커질 때 단일 MLP의 용량 한계 때문에 파라미터와 연산량이 더 늘어날 수 있다고 적었다.

NICE-SLAM 논문은 자기 방식에서 점 하나를 질의하는 연산량이 장면이 아주 커져도 같다고 적었다.

---

## 🧭 아직 열린 것

**실시간 NeRF-SLAM.** 앞에서 본 Co-SLAM의 10~17 Hz는 데스크톱 GPU에서 잰 값이다. 모바일·임베디드 로봇에서 같은 속도를 내는 문제는 남아 있다. Gaussian Splatting(Ch.15)은 명시적 표현으로 렌더링 속도를 높였다.

**대규모 야외 환경.** [Block-NeRF](https://arxiv.org/abs/2202.05263)(Tancik et al., CVPR 2022)는 도시 구역을 여러 국소 NeRF로 나눠 재구성했다. 다만 이미 모은 영상으로 학습하는 오프라인 방법이어서, loop closure와 전역 일관성을 실시간으로 다루는 SLAM과는 문제 설정이 다르다.

**semantic·편집 가능한 implicit 지도.** NeRF 지도는 MLP 가중치나 feature에 장면을 담는다. 그래서 물체를 지도에서 지우거나 영역의 분류를 바꿀 때, TSDF나 point cloud처럼 해당 원소를 직접 고칠 수 없다. [LERF](https://arxiv.org/abs/2303.09553)는 언어 feature를 공간에 정렬해 의미로 질의할 수 있게 했지만 물체 삭제나 편집을 다루지는 않는다. [Nerfstudio](https://arxiv.org/abs/2302.04264) 같은 도구 위에서 의미 표현과 편집 연구가 이어지고 있고, SLAM 파이프라인과 실시간으로 묶는 일은 아직 풀리지 않았다.

---

implicit field 연구와 함께 명시적 표현으로 돌아가는 연구도 나왔다. 장면을 MLP 가중치나 feature grid 대신 공간에 놓인 작은 타원체 수백만 개로 표현하는 방법이다. splatting에는 2001년 EWA splatting 같은 선행 기법이 있었다. Bernhard Kerbl 등의 SIGGRAPH 2023 논문은 학습 가능한 3D Gaussian과 빠른 미분 가능 렌더러를 결합했다.
