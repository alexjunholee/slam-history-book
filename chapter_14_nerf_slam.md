# 17장 NeRF 기반 SLAM

DROID-SLAM이 학습된 신경망을 SLAM의 *tracking*에 투입해 feature와 픽셀 대응을 만들어 냈다면, 같은 Imperial College Dyson Robotics Lab의 Edgar Sucar와 동료들은 2021년 iMAP에서 학습된 연속 함수를 아예 *지도* 그 자체로 세웠다. 그 표현을 가져온 곳은 SLAM 밖의 컴퓨터 그래픽스·비전 분야, 바로 NeRF였다.

2020년 3월 Ben Mildenhall과 동료들이 arXiv에 공개한 [Mildenhall et al. 2020. NeRF](https://arxiv.org/abs/2003.08934)는 원래 카메라 자세를 아는 다수의 입력 영상으로부터 연속 volumetric scene function을 최적화해, 촬영하지 않은 새 시점의 영상을 합성(novel-view synthesis)하는 기법이었다. 2021년 ICCV에 실린 iMAP은 이 구조를 SLAM에 이식해 지도 학습과 자세 추정을 동시에 굴렸다. KinectFusion(12장)이 voxel grid로 풀던 dense mapping을, 신경망으로 짠 연속 함수 즉 implicit neural field로 다시 푼 첫 실시간 시스템이었다.

사실 좌표를 입력받아 3D 공간의 값을 내뱉는 MLP(coordinate-based MLP) 자체는 NeRF 직전인 2019년부터 3D 비전 분야에서 잇따라 시도되고 있었다. [Park et al.의 DeepSDF](https://arxiv.org/abs/1901.05103)는 공간 좌표를 넣으면 signed distance를 출력하는 MLP로 물체 표면을 암묵적으로 기술했고, [Mescheder et al.의 Occupancy Networks](https://arxiv.org/abs/1812.03828)는 같은 좌표 입력에서 점유 확률(occupancy)을 내놓았다. [Sitzmann et al.의 SRN](https://arxiv.org/abs/1906.01618)은 좌표마다 scene feature vector를 뽑아 미분 가능한 ray marching으로 이미지를 합성하는 길을 열었다. NeRF는 이 흐름 위에 volume rendering 적분과 positional encoding을 얹어 실사 수준의 시점 합성에 성공했고, iMAP 역시 DeepSDF와 Occupancy Networks를 직접적인 선행 연구로 인용한다.

---

## 17.1 NeRF: MLP 기반 공간 표현

NeRF의 핵심은 3D 공간 전체를 단 하나의 MLP 가중치에 압축해 넣는 데 있다. 3차원 공간 좌표 $(x, y, z)$와 시선 방향 $(\theta, \phi)$를 네트워크에 입력하면, 그 지점의 색상 $(r, g, b)$과 부피 밀도 $\sigma$가 나온다. 화면의 한 픽셀 색을 얻으려면 카메라 원점 $\mathbf{o}$에서 시선 방향 $\mathbf{d}$로 쏜 광선 $\mathbf{r}(t)$를 따라 다음 volume rendering 방정식을 적분한다.

$$\hat{C}(\mathbf{r}) = \int_{t_n}^{t_f} T(t)\,\sigma\!\left(\mathbf{r}(t)\right) \mathbf{c}\!\left(\mathbf{r}(t), \mathbf{d}\right)\, dt$$

여기서 $T(t) = \exp\!\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s))\, ds\right)$는 광선이 $t$ 지점까지 가로막히지 않고 도달할 확률, 곧 누적 투과율이다. 실제 구현에서는 광선 위에 여러 샘플 점을 찍어 이 연속 적분을 구간별 합으로 근사한다.

그런데 좌표를 날것 그대로 MLP에 집어넣으면 신경망이 저주파 성분만 따라가며 뭉개진 결과를 낸다. NeRF는 이 문제를 positional encoding으로 넘겼다. 입력 좌표 $(x, y, z)$를 여러 주파수의 사인·코사인 함수로 펼쳐 넣어, 네트워크가 세밀한 질감과 날카로운 경계까지 포착하도록 길을 터 준 것이다.

$$\gamma(p) = \left(\sin(2^0 \pi p),\, \cos(2^0 \pi p),\, \ldots,\, \sin(2^{L-1} \pi p),\, \cos(2^{L-1} \pi p)\right)$$

최적화는 단순하다. 자세가 알려진 실제 이미지와 같은 시점에서 위 식을 적분해 그린 렌더링 영상을 비교하고, 픽셀 색상의 제곱 오차 합을 줄여 나간다. 수렴이 끝나면 장면의 3차원 기하와 외관이 통째로 MLP 파라미터 속에 담긴다.

다만 원판 NeRF에는 SLAM에 바로 쓸 수 없는 무거운 제약이 있었다. 장면 하나를 학습하는 데 V100 GPU 한 장으로 꼬박 1~2일이 걸렸고, 장면마다 모델을 처음부터 다시 학습해야 했으며, 카메라 자세는 이미 COLMAP 같은 SfM으로 구해 두어야 했다. 실시간 SLAM으로 만들려면 자세 추정과 지도 학습을 동시에, 그것도 초당 몇 차례 이상 돌려야 했다.

> 🔗 **차용.** volume rendering 방정식은 [Kajiya & Von Herzen(1984)](https://courses.cs.duke.edu/cps296.8/spring03/papers/RayTracingVolumeDensities.pdf)의 컴퓨터 그래픽스 논문에서 가져온 것이다. NeRF는 이 고전 적분식을 미분 가능하게 구현해, 렌더링한 색과 실제 픽셀의 차이로 MLP를 역전파 학습했다.

> 🔗 **차용.** positional encoding은 NeRF 원 논문에 이미 포함되어 있었고, 같은 해 [Tancik et al.(2020)](https://arxiv.org/abs/2006.10739)의 "Fourier Features Let Networks Learn High Frequency Functions"가 NTK(neural tangent kernel) 이론을 동원해 이 기법이 왜 고주파 디테일을 살려 내는지 수학적으로 규명했다.

---

## 17.2 iMAP: 초기 neural implicit SLAM

이 시간의 벽을 처음 뚫은 시스템이 Imperial College Dyson Robotics Lab의 Edgar Sucar, Shikun Liu, Joseph Ortiz, Andrew Davison이 2021년 ICCV에 발표한 [Sucar et al. 2021. iMAP](https://doi.org/10.1109/ICCV48922.2021.00617)이다. **iMAP**(Implicit MAP)은 손에 든 RGB-D 카메라가 보내오는 영상으로 MLP 하나를 실시간으로 학습시키면서 동시에 카메라 자세까지 최적화했다.

실시간성을 확보하기 위해 iMAP은 하나의 MLP를 공유하는 두 개의 최적화 루프를 병렬로 돌렸다. *mapping* 루프는 MLP 가중치와 소수 keyframe의 자세를 함께 갱신한다. 라이브 시스템에서는 매 반복마다 과거 keyframe 가운데 손실 분포에 비례해 3장을 뽑고, 직전 keyframe과 현재 프레임을 더해 총 5장으로 최적화를 돌렸다. 픽셀 역시 전부를 렌더링하지 않고 이미지당 200개만 골라 오차가 큰 영역에 집중 배정했다. 한편 *tracking* 루프는 MLP 가중치를 얼려 둔 채 현재 프레임의 자세만 렌더링 손실로 맞췄다. 이렇게 연산을 쪼갠 덕분에 논문은 tracking 10 Hz, 전역 지도 갱신 2 Hz라는 수치를 보고했다.

학습 신호로는 색상 손실 $\mathcal{L}_{\text{color}} = \|\hat{C} - C\|_1$과 깊이 손실 $\mathcal{L}_{\text{depth}} = \|\hat{D} - D\|_1$을 함께 썼다. 이때 깊이 손실은 광선을 따라 계산한 depth 분산으로 정규화해, 물체 경계처럼 불확실성이 큰 지점의 영향력을 줄였다.

결과는 신선했다. keyframe 10~20장과 고작 1 MB 남짓한 MLP 하나로 방 한 칸의 dense 지도를 통째로 담아냈다. 다만 치명적인 약점이 하나 따라왔다. 새 구역을 학습하면 신경망 가중치 전체가 흔들리면서 앞서 배운 구역의 표현이 망가질 수 있었고, 이를 막기 위해 iMAP은 골라 둔 keyframe을 기억 저장소 삼아 반복해서 다시 학습(replay)해야 했다.

> 📜 **예언 vs 실제.** Sucar는 iMAP 논문 Conclusion에서 *"future directions for iMAP include how to make more structured and compositional representations that reason explicitly about the self similarity in scenes"*라고 적었다. iMAP이 arXiv에 올라온 지 불과 9개월 뒤(2021년 12월) 등장한 NICE-SLAM은 multi-resolution voxel feature grid로 공간을 계층적으로 쪼개 이 문제에 답했다. 2023년 Wang et al.의 Co-SLAM은 hash grid와 coordinate encoding을 결합해 한 걸음 더 나아갔다.

---

## 17.3 NICE-SLAM: 계층 격자와 scalability

iMAP이 부딪힌 단일 MLP의 확장성 한계를 정면으로 겨냥한 연구가 ETH Zurich의 Zihan Zhu와 Songyou Peng이 2022년 CVPR에서 내놓은 [Zhu et al. 2022. NICE-SLAM](https://arxiv.org/abs/2112.12130)이다. **NICE-SLAM**(Neural Implicit Scalable Encoding for SLAM)은 공간 전체를 큰 MLP 하나에 구겨 넣는 대신, multi-resolution voxel feature grid와 아주 작은 MLP decoder를 조합했다.

구조는 직관적이다. 3D 공간을 명시적인 복셀 격자로 나눈 뒤 각 꼭짓점에 학습 가능한 feature vector를 심어 둔다. 특정 좌표를 렌더링할 때는 주변 복셀의 feature를 trilinear interpolation으로 섞어 작은 MLP에 통과시키면 색상과 점유 확률이 나온다. 기하 정보의 대부분을 공간 격자가 나눠 맡으므로 decoder MLP는 작고 가벼워도 충분하다.

NICE-SLAM은 세 단계 해상도의 격자를 겹쳐 세워 거친 윤곽부터 세밀한 형상까지 계층적으로 표현하고, 색상은 별도의 color feature grid와 decoder로 분리했다. geometry decoder 세 개는 ConvONet 방식으로 사전 학습해 고정한 채 실행 중에는 격자 feature만 갱신했는데, 시야에 들어온 지역 격자만 건드리므로 iMAP처럼 과거 기억이 덮어써지는 망각(forgetting) 문제를 피할 수 있었다.

운용 방식 역시 tracking에서는 격자와 decoder를 고정하고 자세만 풀고, mapping에서는 격자 feature를 다듬는 이원 구조를 취했다. 덕분에 여러 방으로 이어진 아파트 전체를 직접 촬영해 재구성할 수 있었고, 같은 픽셀 수 기준으로 iMAP보다 tracking은 2배 이상(101→47 ms), mapping은 3배 이상(448→130 ms) 빨랐다.

하지만 3D 격자라는 그릇 자체의 무게가 발목을 잡았다. ESLAM 논문이 지적했듯 3D 격자 메모리는 장면 한 변 길이의 세제곱으로 불어났고, Co-SLAM 논문이 Replica에서 측정한 전체 시스템 속도 역시 0.91 Hz에 그쳐 진정한 실시간과는 거리가 있었다.

바로 그 즈음 그래픽스 쪽에서 속도의 판도를 바꾸는 돌파구가 터졌다. Thomas Müller와 동료들이 발표한 [Müller et al. 2022. Instant-NGP](https://nvlabs.github.io/instant-ngp/)(SIGGRAPH 2022)다. multi-resolution hash encoding과 GPU 친화적 커널 구현으로 학습 시간을 극적으로 줄여, 공개 구현이 지원되는 NVIDIA GPU에서 일부 데모 장면을 단 몇 초 만에 학습해 보였다. SLAM 논문은 아니었지만, 이 hash encoding은 곧바로 차세대 NeRF-SLAM 시스템들의 핵심 부품으로 흡수됐다.

> 🔗 **차용.** NICE-SLAM의 multi-resolution feature grid와 Instant-NGP의 hash encoding은 비슷한 시기에 독립적으로 등장했다(arXiv 2021년 12월, 2022년 1월). 이후 Co-SLAM이 hash grid를 직접 가져다 썼다. 격자 안에 TSDF 값 대신 학습된 feature를 담는다는 점에서, 이 발상은 KinectFusion(12장)의 voxel 격자와도 맥이 닿아 있다.

---

## 17.4 Co-SLAM과 NeRF-SLAM

2022년 말부터 후속 연구는 뚜렷하게 두 갈래로 갈렸다. 한쪽은 implicit 지도 표현 자체의 수렴 속도와 메모리 효율을 끌어올리는 데 매달렸고, 다른 한쪽은 이미 검증된 고전·학습 추적기 위에 NeRF 지도를 얹는 결합을 택했다.

전자를 대표하는 UCL의 Wang et al.의 [**Co-SLAM**](https://arxiv.org/abs/2304.14377)(CVPR 2023)은 multi-resolution hash grid(parametric encoding)와 one-blob coordinate encoding을 함께 엮었다. hash grid가 빠른 수렴과 국소 디테일을 책임지는 동안, one-blob encoding이 표면을 매끄럽게 이어 주며 관측되지 않은 빈틈까지 메우는 구조다. 그 결과 Replica 15~17 Hz, ScanNet·TUM 12~13 Hz로 10~17 Hz 수준의 속도를 끌어냈다. 같은 CVPR에 발표된 Idiap/EPFL의 Johari et al.의 [**ESLAM**](https://arxiv.org/abs/2211.11704)은 한발 더 나아가 3D feature grid를 multi-scale axis-aligned feature plane으로 대체해 메모리 복잡도를 $O(n^3)$에서 $O(n^2)$로 낮추고, volume density 대신 TSDF를 decoding 목표로 삼아 사전 학습 없이도 NICE-SLAM보다 최대 10배 빠른 속도를 기록했다.

후자의 길을 연 것은 MIT의 Antoni Rosinol, John Leonard, Luca Carlone이 2022년에 공개한 [**NeRF-SLAM**](https://arxiv.org/abs/2210.13641)(IROS 2023)이다. RGB-D 없이 단안 영상만 받아 DROID-SLAM 방식으로 카메라 자세와 dense depth를 먼저 풀고, 그 최적화 과정에서 주변 공분산(marginal covariance)까지 함께 뽑아낸다. 그러면 별도의 Instant-NGP 기반 mapping 모듈이 이 자세·깊이 추정치와 불확실성을 넘겨받아 실시간으로 radiance field를 학습한다.

> 🔗 **차용.** NeRF-SLAM은 DROID-SLAM의 recurrent update와 Dense Bundle Adjustment에서 나온 자세·깊이 추정을 그대로 끌어오고, 지도 그릇으로 Instant-NGP를 붙였다. radiance field가 카메라 자세 추정까지 떠맡지 않고, 검증된 추정기의 출력을 받아 실시간 dense 지도를 빚어내는 분업 구조다.

---

## 17.5 iMAP의 구조적 한계

iMAP에서 NICE-SLAM·Co-SLAM으로 이어진 세대교체의 뿌리에는 **지역성(locality)**이라는 구조적 차이가 있었다. 단일 MLP는 지역성이 없다. 공간의 어느 귀퉁이를 렌더링하든 네트워크 가중치 전체를 거쳐야 하므로, 새로운 구역을 학습할 때마다 기존 영역을 지탱하던 가중치까지 함께 뒤틀린다(catastrophic forgetting). 게다가 NICE-SLAM 논문이 지적했듯 장면이 커질수록 단일 MLP가 감당해야 할 표현 용량이 벅차져 파라미터와 연산량 자체가 늘어날 수밖에 없다.

반면 복셀이나 hash grid 기반의 지역 표현에서는 장면이 아무리 넓어져도 공간의 한 점을 질의할 때 건드리는 연산량이 일정하게 유지된다. 온라인으로 지도가 끝없이 자라나는 SLAM에서 지역성은 단순한 최적화 기교가 아니라 확장성을 좌우하는 전제 조건이었다.

---

## 아직 열린 문제

**실시간 NeRF-SLAM.** Co-SLAM이 기록한 10~17 Hz는 데스크톱 GPU 위에서 얻은 성적이다. 전력과 연산 자원이 빠듯한 모바일·임베디드 로봇에서 같은 속도를 내는 문제는 여전히 풀리지 않았고, 이 속도 한계는 곧 명시적 Gaussian 표현으로 렌더링을 가속하는 Gaussian Splatting(18장)의 부상으로 이어졌다.

**대규모 야외 환경.** [Block-NeRF](https://arxiv.org/abs/2202.05263)(Tancik et al., CVPR 2022)는 도시 구역을 여러 국소 NeRF로 쪼개 재구성하는 해법을 보였지만, 이미 모아 둔 영상으로 학습하는 오프라인 방식이었다. 주행 중에 실시간으로 loop closure를 처리하고 전역 일관성을 지켜야 하는 SLAM의 문제 설정은 아직 도시 규모에 닿지 못했다.

**semantic·편집 가능한 implicit 지도.** NeRF 지도는 장면을 MLP 가중치나 추상적인 feature grid 안에 녹여 저장한다. 이 때문에 TSDF나 point cloud처럼 특정 물체를 집어내 삭제하거나 영역의 속성을 직접 고치기가 까다롭다. [LERF](https://arxiv.org/abs/2303.09553)가 언어 feature를 공간에 정렬해 자연어로 장면을 질의할 수는 있게 만들었지만 물체 삭제나 편집까지는 다루지 않았고, [Nerfstudio](https://arxiv.org/abs/2302.04264) 생태계에서 의미 표현과 편집 연구가 이어지고 있으나 이를 실시간 SLAM 파이프라인 안에 통합하는 과제는 아직 남아 있다.

---

implicit field 연구가 격자와 hash를 덧대며 속도를 끌어올리는 사이, 아예 명시적(explicit) 표현으로 되돌아가는 또 하나의 물결이 일었다. 장면을 MLP 가중치나 복셀 격자 대신 공간에 흩뿌린 작은 타원체 수백만 개로 직접 빚는 방식이다. 2001년 EWA splatting 같은 그래픽스의 오래된 선행 기법 위에 학습 가능한 3D Gaussian과 초고속 미분 가능 렌더러를 얹은 Bernhard Kerbl 등의 SIGGRAPH 2023 논문이 나오자, SLAM 지도 표현의 판도는 다시 한번 급격히 움직이기 시작했다.
