# Ch.9 — Dense/RGB-D: KinectFusion부터 BundleFusion까지

2011년 10월, Imperial College London의 Richard Newcombe는 ISMAR에서 KinectFusion을 발표했다. 데모 영상에서는 손에 든 Kinect 센서 하나로 방 전체를 실시간으로 3D 메시로 만들었다. KinectFusion은 Curless와 Levoy가 1996년 그래픽스 분야에서 내놓은 부호 거리 함수 표현, Besl과 McKay가 1992년 발표한 ICP 정합, Microsoft의 Kinect 센서를 함께 썼다. 같은 해 Newcombe는 단안 카메라 영상의 밝기로 dense 재구성을 한 DTAM(Ch.8)도 발표했다.

---

## 9.1 Kinect 이전의 dense 재구성

2011년 이전에도 dense 3D 재구성은 있었다. [Pollefeys 등(IJCV 2008)](https://doi.org/10.1007/s11263-007-0086-4)처럼 차량에서 찍은 영상과 GPS/INS로 도시 3D 모델을 실시간으로 만든 시스템도 있었지만, 대부분은 오프라인으로 처리했다.

오프라인 파이프라인은 스테레오나 structured light(구조광: 정해진 빛 무늬를 쏘아 그 무늬가 휘는 정도로 깊이를 재는 방식) 스캐너로 얻은 point cloud를 시간을 들여 합쳤다. 스캔 장비가 비싸서 연구실 밖에서는 쓰기 어려웠다. SLAM 연구자들은 sparse landmark만으로도 쓸 만한 결과를 얻고 있었고, dense 재구성은 주로 그래픽스 연구자들이 다뤘다.

[Curless와 Levoy의 1996년 SIGGRAPH 논문 "A Volumetric Method for Building Complex Models from Range Images"](https://graphics.stanford.edu/papers/volrange/volrange.pdf)는 이 시기 그래픽스 쪽 방법을 대표한다. 논문은 3D 공간을 균일한 복셀 그리드로 나누고, 복셀마다 센서 시선 방향으로 잰 표면까지의 부호 있는 거리를 가중치와 함께 누적한다. 이 값은 표면 근처에서만 의미가 있으므로 논문은 거리와 가중치를 측정점 주변으로 잘라서 쓴다. 뒤에 KinectFusion은 이 표현을 **TSDF(Truncated Signed Distance Function, 잘라 낸 부호 거리 함수)**라고 불렀다. 부호는 센서 쪽에서 표면을 향해 갈 때 표면 앞(빈 공간)이 양수, 표면 뒤(물체 내부)가 음수다. 잘라 낸다는 것은 값을 일정 한계 $t$ 안으로 묶는다는 뜻이어서 $\text{TSDF}(x) = \text{clip}(d(x), -t, +t)$가 된다. 새 깊이 프레임이 들어올 때마다 이 값을 가중 평균으로 갱신하면 노이즈가 조금씩 평균되어 표면이 점점 선명해진다. 표면은 TSDF 값이 0이 되는 곳(zero-crossing)에 marching cubes를 적용해 뽑는다.

이 방법은 정확했지만 복셀 그리드가 메모리를 많이 썼고, 당시 하드웨어로는 실시간으로 갱신할 수 없었다. 그 뒤 GPU로 그래픽스 밖의 범용 계산을 하게 되었고(GPGPU), Kinect가 나왔다.

---

## 9.2 KinectFusion과 TSDF

Microsoft는 2010년 11월 4일 Xbox 360용 Kinect를 북미에서 149.99달러에 출시했다. Kinect는 구조광으로 깊이를 재서 640×480 깊이 맵을 30 Hz로 내보냈다. 연구용 ToF 카메라보다 정밀도는 낮았지만 값이 훨씬 쌌다. 출시 엿새 뒤인 11월 10일 Hector Martin이 뒤에 libfreenect가 되는 오픈소스 드라이버를 공개했고, 연구자들도 곧 Kinect를 쓰기 시작했다.

Newcombe는 박사과정 중 Microsoft Research Cambridge의 Shahram Izadi 팀과 함께 Kinect depth 스트림으로 GPU 기반 dense SLAM을 만들었다. 그 결과가 2011년 ISMAR에서 발표된 [Newcombe et al. 2011. KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378)이다.

> 🔗 **차용.** KinectFusion의 지도 표현인 TSDF는 Curless & Levoy(1996)가 오프라인 3D 스캐닝을 위해 고안한 표현을 이어받았다. Newcombe 팀은 복셀을 GPU에서 병렬로 갱신해 실시간으로 돌렸다.

파이프라인은 다음 단계로 이루어진다.

깊이 전처리 단계에서는 원시 깊이 맵에 bilateral filter(경계는 살리고 노이즈만 줄이는 필터)를 걸고 표면 법선을 계산한다.

ICP 추적 단계에서는 현재 프레임의 point cloud를, 지금까지 쌓은 TSDF에서 ray-cast로 그려 낸 가상 표면에 맞춘다. [Besl & McKay(1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)의 **ICP(Iterative Closest Point, 가장 가까운 점끼리 대응시켜 정렬을 반복하는 방법)**를 point-to-plane 형태로 바꿔 쓴다. 영상 피라미드 3단계에서 거친 단계부터 고운 단계로 각각 최대 4·5·10회 반복한다. 반복할 때마다 GPU가 수많은 vertex-normal 대응에서 나온 정규방정식 항을 병렬로 더한다. 결과는 카메라의 6-DoF 자세다.

point-to-plane ICP의 목적함수는 다음과 같다. 현재 프레임의 포인트 $\mathbf{p}_i$를 변환 $T = (R, \mathbf{t})$로 옮기고, 대응 점 $\hat{\mathbf{p}}_i$(ray-cast 표면)와 법선 $\hat{\mathbf{n}}_i$로

$$E(R, \mathbf{t}) = \sum_i \bigl(\hat{\mathbf{n}}_i^\top (R\,\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i)\bigr)^2$$

을 최소화한다. Besl-McKay의 원래 point-to-point 오차($\|R\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i\|^2$)는 점 사이 거리 전체를 잰다. point-to-plane 오차는 그 차이의 법선 방향 성분만 재므로, 점이 표면을 따라 미끄러지는 방향의 어긋남에는 덜 민감하다. 작은 회전을 $R \approx I + [\boldsymbol{\omega}]_\times$로 근사하면 $E$는 6차원 벡터 $(\boldsymbol{\omega}, \mathbf{t})$에 대한 선형 최소자승 문제가 되고, GPU에서 parallel reduction으로 한 번에 푼다.

> 🔗 **차용.** KinectFusion의 추적은 Besl & McKay(1992)의 ICP를 이어받아 GPU에서 dense하게 돌렸다. ICP는 IEEE PAMI에 실린 3D 형상 정합 기법이다.

TSDF 통합 단계에서는 추정한 자세로 깊이 맵을 복셀 그리드에 투영해 TSDF 값을 갱신한다. 논문은 3 m³ 볼륨을 64³부터 512³ 복셀까지 여러 해상도로 나눠 시간을 쟀다(Fig. 13).

표면 렌더링 단계에서는 TSDF의 zero-crossing을 ray marching으로 찾아 표면의 vertex map과 normal map을 만든다. 이 결과가 다음 프레임 ICP 추적의 기준 표면이 된다.

> 🔗 **차용.** KinectFusion과 DTAM은 Newcombe가 같은 해 발표한 dense 시스템이고 둘 다 GPU를 쓴다. DTAM은 단안 영상의 밝기 오차를 최적화한다. KinectFusion은 depth를 ICP로 정합해 TSDF에 통합하므로, 지도 표현과 목적함수가 DTAM과 다르다.

KinectFusion은 30 Hz로 추적하면서 방 한 칸을 dense mesh로 복원했다. 프레임을 직전 프레임에 맞추지 않고 지금까지 합친 전체 모델에 맞추므로(model-to-frame), 방 크기의 볼륨 안에서는 추적 drift가 작았다. 다만 그 모델도 과거에 추정한 자세로 쌓은 것이라 절대 기준은 아니었다.

복셀 그리드는 미리 정한 공간만 덮어서, 카메라가 그 밖으로 나가면 더 기록할 수 없었다. loop closure도 없었다. Kinect의 IR 구조광은 햇빛 아래에서 패턴이 묻혀 실외에서는 쓰기 어려웠다.

---

## 9.3 Kintinuous — rolling volume

KinectFusion이 발표된 직후 Maynooth 대학의 Thomas Whelan과 MIT의 Michael Kaess, John Leonard 등은 볼륨 크기 문제를 다뤘다. 이들은 TSDF 볼륨이 카메라를 따라 움직이게 했다.

[Whelan 등의 Kintinuous](https://www.cs.cmu.edu/~kaess/pub/Whelan12rssw.pdf)는 2012년 7월 Sydney에서 열린 RSS 워크숍 "RGB-D: Advanced Reasoning with Depth Cameras"에서 발표됐다. Kintinuous는 TSDF 배열을 순환 버퍼로 다룬다. 카메라가 볼륨 경계에 가까워지면 볼륨을 카메라 쪽으로 옮기고, 볼륨 밖으로 밀려난 slice에서 point cloud를 뽑아 삼각형 메시에 덧붙인다. 그래서 메모리는 일정하게 유지되고 카메라는 계속 이동할 수 있다.

이 방식으로 복도처럼 긴 공간도 재구성할 수 있었다. 다만 loop closure는 여전히 없어서, 긴 경로를 돌아 출발점에 왔을 때 두 끝이 맞지 않았다. 표면은 세밀해도 전역 지도는 어긋났다.

---

## 9.4 ElasticFusion: Surfel과 비강체 변형

Whelan은 Kintinuous 다음 시스템에서 TSDF 복셀 대신 surfel을 썼다.

**surfel(surface element, 표면 요소)**은 위치, 법선, 반지름, 색을 지닌 점이다. 컴퓨터 그래픽스에서 [Pfister 등(2000)](https://www.merl.com/publications/docs/TR2000-10.pdf)이 렌더링용으로 제안했다. surfel은 격자 없이 표면 위에만 불규칙하게 놓인다.

Whelan 등은 ElasticFusion을 RSS 2015에 처음 발표했고, 2016년 IJRR에 [확장판](https://doi.org/10.1177/0278364916669237)을 냈다. ElasticFusion은 surfel 기반 dense 지도에 *non-rigid deformation*(비강체 변형)을 적용해 loop closure를 처리했다.

dense SLAM에서 loop closure는 어려웠다. 전역 메시나 복셀 그리드를 loop closure 결과에 맞춰 고치려면 비용이 컸다. ElasticFusion은 surfel을 deformation graph(변형 그래프)에 연결하고, loop closure를 찾으면 그래프를 변형해 오차를 지도 전체에 나눴다.

deformation graph의 노드 $g_k$마다 위치 $\mathbf{v}_k$, 회전 $R_k$, 이동 $\mathbf{t}_k$가 있다. surfel $s$는 가장 가까운 $K$개 노드의 영향을 받고, 변형 뒤 위치는

$$\tilde{\mathbf{p}}_s = \sum_{k \in \mathcal{N}(s)} w_k \bigl(R_k (\mathbf{p}_s - \mathbf{v}_k) + \mathbf{v}_k + \mathbf{t}_k\bigr)$$

로 계산한다(가중치 $w_k$는 거리가 멀수록 줄어든다). loop closure 제약이 들어오면 노드들의 $(R_k, \mathbf{t}_k)$를 Gauss-Newton으로 최적화해 오차를 지도 전체에 나눈다. 그래서 지도를 처음부터 다시 쌓지 않고도 dense 지도 전체를 고칠 수 있었다.

ElasticFusion 논문은 ICL-NUIM 합성 데이터셋에서 좋은 결과를 보고했다. kt0·kt1·kt2 시퀀스의 ATE RMSE는 1.4 cm 이하였고, 그중 kt0·kt1은 0.9 cm였다. kt3에서는 오차가 이보다 훨씬 컸다.

---

## 9.5 BundleFusion

[Dai et al. 2017. BundleFusion](https://doi.org/10.1145/3072959.3054739)은 Dai, Nießner, Zollhöfer, Izadi, Theobalt가 2017년 ACM Transactions on Graphics에 발표한 시스템이다. 이들은 GPU 연산을 많이 써서, 온라인 시스템에서도 모든 카메라 자세를 bundle adjustment로 실시간으로 함께 최적화하려 했다.

BundleFusion은 새 프레임이 들어올 때마다 지금까지 들어온 모든 RGB-D 입력으로 카메라 자세 전체를 다시 최적화한다. 최적화는 두 층으로 나눠 한다. 연속한 11프레임을 chunk 하나로 묶어 chunk 안에서 먼저 최적화하고, 각 chunk의 keyframe들을 전역으로 정렬한다. 대응은 sparse SIFT feature와 dense한 기하·밝기 정합을 함께 쓴다. 새 프레임은 매번 전역 최적화된 프레임들에 위치를 다시 맞춘다. 그래서 추적이 크게 실패해도 relocalization(재위치 추정)으로 돌아온다. 자세가 바뀌면 해당 프레임을 TSDF에서 빼고 새 자세로 다시 넣어(re-integration) 지도도 함께 고친다.

논문은 TUM RGB-D 등에서 기존 온라인 시스템보다 나은 결과를 보고했다.

> 📜 **예언 vs 실제.** BundleFusion 초록은 기존 온라인 시스템보다 낫고 품질은 오프라인 방법과 대등하면서 "unprecedented speed and scan completeness"를 보인다고 썼다. GPU 성능은 그 뒤로도 빠르게 올랐다. 그런데 2021년 이후 연구 관심의 일부는 COLMAP으로 자세를 구하고 NeRF로 장면을 표현하는 파이프라인으로 옮겨 갔다. 이 흐름은 주로 novel-view synthesis(새 시점 영상 합성)와 장면 표현을 다뤘다.

---

## 9.6 하드웨어와 알고리즘

Kinect 1세대는 구조광 방식이었다. 깊이 정밀도는 미터 범위에서 수 밀리미터였지만, 햇빛 아래에서는 IR 패턴이 잡히지 않았다. 2013년 11월 Xbox One과 함께 나온 Kinect 2는 ToF(Time-of-Flight: 쏜 빛이 돌아오는 시간으로 거리를 재는 방식) 센서를 썼고, 정밀도와 시야가 나아졌다. 이어 Intel RealSense 시리즈가 나왔다. 센서 종류가 늘면서 깊이 품질도 센서마다 달라졌고, 연구자들은 센서별 노이즈 특성에 맞춰 방법을 조정했다.

GPU 쪽에서는 CUDA 도구와 라이브러리가 자리를 잡았고, 2011년부터 2017년 사이 GPU의 처리량과 메모리 성능도 올라갔다. ElasticFusion과 BundleFusion이 점점 무거운 최적화를 실시간으로 돌릴 수 있었던 데에는 이런 하드웨어 발전과, 연구실에서 쉽게 구할 수 있을 만큼 싼 Kinect가 한몫했다고 본다.

> 📜 **예언 vs 실제.** KinectFusion의 한계였던 공간 범위, drift, 실외 사용은 그 뒤 연구 과제가 됐다. 공간 범위와 drift는 Kintinuous, ElasticFusion, BundleFusion이 차례로 다뤘다. 실외에서는 햇빛이 IR 구조광 패턴을 가려서, 초기 Kinect 기반 dense SLAM은 주로 실내에서 쓰였다. 실외 dense mapping에서는 LiDAR가 주요 센서가 됐다. 햇빛의 영향은 센서 방식마다 정도가 다르다.

---

## 9.7 dense 방식이 주류가 되지 못한 이유

2011년부터 2017년 사이 dense RGB-D SLAM은 Visual SLAM의 주된 방향이 될 것처럼 보였다. 그러나 2015년 이후 나온 실용 SLAM 시스템인 [ORB-SLAM2](https://arxiv.org/abs/1610.06475)와 [VINS-Mono](https://arxiv.org/abs/1708.03852)는 dense 지도를 기본으로 만들지 않았다.

먼저 메모리가 문제였다. KinectFusion은 복셀마다 TSDF 값과 가중치를 16비트씩 저장하므로, 512³ 볼륨 하나에 512 MB가 든다. 모바일 플랫폼이나 임베디드 시스템에서는 감당하기 어려운 크기였다. 해시 블록에 TSDF를 저장하는 [Voxblox](https://arxiv.org/abs/1611.03631)와 octree에 점유 확률을 저장하는 [OctoMap](https://www.hrl.uni-bonn.de/papers/wurm10octomap.pdf)은 저마다 다른 지도 표현으로 메모리 부담을 줄였지만, sparse 방식만큼 가볍지는 않았다. 실시간 dense 처리에는 GPU가 필요했는데, 자율주행 차량의 임베디드 프로세서나 드론의 가벼운 보드에서는 KinectFusion 수준의 파이프라인을 돌리기 어려웠다. 게다가 Kinect의 IR depth는 실외에서 쓰기 어려웠는데, 자율주행과 드론처럼 상용화 요구가 큰 분야는 대부분 실외였다.

dense 지도의 자료구조는 KinectFusion의 고정 볼륨에서 여러 방향으로 발전했다. [Museth(2013)의 VDB](https://doi.org/10.1145/2487228.2487235)는 동적으로 커지는 sparse root와 얕고 넓은 B+ tree를 결합했다. 값이 있는 영역에만 노드를 할당하면서도 임의 접근이 빠르다. 이 구조는 OpenVDB로 공개되어 큰 sparse volumetric 데이터를 다루는 데 널리 쓰인다. [Reijgwart 등(2023)의 wavemap](https://arxiv.org/abs/2306.01279)은 Haar wavelet으로 occupancy를 압축해 여러 해상도를 한 구조에 담았다.

지도를 연속 함수로 표현한 연구도 있었다. [O'Callaghan과 Ramos(2012)의 GPOM(Gaussian Process Occupancy Map)](https://doi.org/10.1177/0278364911421039)은 점유 지도 작성을 분류 문제로 보고 변형한 Gaussian Process로 풀었다. 그래서 가려진 영역이나 센서 빔 사이처럼 직접 관측하지 못한 곳도 분산과 함께 추정했다. [Ramos와 Ott(2016)의 Hilbert Map](https://doi.org/10.1177/0278364916684382)은 Hilbert space 특징 위에 logistic regression을 학습시켜, 데이터가 들어오는 대로 stochastic gradient descent로 갱신하는 연속 occupancy 지도를 만들었다.

[Behley와 Stachniss(2018)의 SuMa](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/behley2018rss.pdf)는 surfel 지도를 회전식 3D LiDAR 데이터에 적용했다. 현재 스캔을 surfel 지도에서 렌더링한 모델 뷰에 맞춰 자세를 추정했고, 도시 규모 환경에서 loop closure까지 온라인으로 처리했다(→ Ch.17).

2020년 NeRF가 나온 뒤 고품질 dense 재구성 연구는 NeRF와 3D Gaussian Splatting으로 옮겨 갔다. 그래도 RGB-D로 잰 깊이는 추적과 형상 학습 양쪽에서 제약으로 계속 쓰였다. TSDF 같은 volumetric 표현은 로봇 지도에 쓰이고, ICP는 LiDAR SLAM의 주요 정합 방법이 됐다.

---

## 🧭 아직 열린 것

**대규모 실외 dense 재구성.** 햇빛에 약한 것은 IR 구조광을 포함한 active depth 센서 전반의 문제다. LiDAR는 더 먼 거리를 재지만 색과 세밀한 표면 정보가 부족하다. 학습 기반 stereo depth 추정은 빠르게 나아지고 있지만, 어두운 곳, 반사면, 먼 거리에서는 아직 한계가 있다.

**동적 장면의 dense 재구성.** KinectFusion부터 BundleFusion까지 모두 장면이 움직이지 않는다고 가정했다. 사람이 걸어 다니는 공간을 dense하게 재구성하려면 움직이는 물체를 semantic segmentation이나 기하 residual로 걸러 내야 한다. [DynaSLAM](https://arxiv.org/abs/1806.05620)과 [MaskFusion](https://arxiv.org/abs/1804.09194) 등이 이 방향을 시도했다. 계산 비용과 robustness(강건성)를 실제 사용 수준으로 맞추는 일은 남은 과제다.

**복셀 지도의 메모리.** Voxblox와 OctoMap으로 메모리를 줄였지만, 건물 한 층이나 도시 블록 규모로 넓히고 해상도를 높이면 dense 지도의 메모리는 빠르게 커진다. 영역마다 해상도를 달리하는 방법으로는 §9.7에서 본 wavemap 같은 다해상도 구조가 나왔다. [Instant-NGP](https://arxiv.org/abs/2201.05989) 같은 implicit neural representation도 이 문제를 다룬다. 실시간 갱신 속도와 조회 속도를 함께 맞추는 일은 아직 남아 있다.

KinectFusion에는 loop closure가 없었다. 전에 본 장소를 알아보는 place recognition은 Ch.10에서 다룬다.
