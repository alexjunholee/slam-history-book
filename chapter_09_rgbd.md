# Ch.9 — Dense/RGB-D: KinectFusion부터 BundleFusion까지

2011년 11월, Richard Newcombe(Imperial College London)가 ISMAR에서 KinectFusion을 발표했을 때 청중의 반응은 논문보다 데모 영상에 집중됐다. 손에 들린 Kinect 센서 하나가 실시간으로 방 전체를 3D 메시로 채워가는 장면이었다. 그것은 Newcombe 자신이 같은 해 발표한 DTAM이 단안 카메라로 꿈꾸던 것을 RGB-D 센서로 실제로 해낸 것이었다. 계보는 선명하다: 1996년 Curless와 Levoy가 그래픽스 커뮤니티를 위해 고안한 TSDF 표현, 1992년 Besl과 McKay가 로봇공학에 제공한 ICP 추적, 그리고 2010년 Microsoft가 $150에 출시한 Kinect 센서. 이 세 줄기가 교차한 지점에서 dense SLAM의 짧고 강렬한 시대가 열렸다. Davison의 MonoSLAM(Ch.5)이 단안 카메라로 sparse landmark를 추적하던 바로 그 프레임워크—실시간 추적, GPU 없이 CPU만으로—가 이제 Kinect의 깊이 스트림 앞에서 다른 결론에 도달했다. Newcombe의 DTAM(Ch.8)이 직접 광도 최적화로 dense 재구성을 시도하면서 GPU의 가능성을 열었고, KinectFusion은 그 가능성을 RGB-D 센서로 닫았다.

---

## 9.1 Kinect 이전의 dense 재구성

2011년 이전에도 dense 3D 재구성은 가능했다. 가능하지 않았던 것은 *실시간*이었다.

오프라인 파이프라인들은 스테레오 혹은 structured light 스캐너로 취득한 포인트 클라우드를 시간을 들여 병합했다. 실내 스캔 장비는 수십만 달러였다. 연구실 바깥에서 이 기술을 쓰는 사람은 없었다. SLAM 커뮤니티는 이미 sparse landmark로 충분히 실용적인 결과를 얻고 있었고, dense 재구성은 그래픽스 쪽 문제로 분류해 두고 있었다.

Curless와 Levoy의 1996년 SIGGRAPH 논문 "A Volumetric Method for Building Complex Models from Range Images"는 이 시기의 그래픽스 쪽 접근을 대표한다. 핵심 아이디어는 **TSDF(Truncated Signed Distance Function)**였다. 3D 공간을 균일한 복셀 그리드로 나누고, 각 복셀에 가장 가까운 표면까지의 부호 있는 거리를 누적한다. 부호 관행은 센서에서 표면 방향으로 진행할 때 표면 앞(free space)이 양수, 표면 뒤(solid 내부)가 음수다. Truncated란 이 값을 절댓값 기준 일정 한계 $t$ 이내로 잘라낸다는 뜻으로, $\text{TSDF}(x) = \text{clip}(d(x), -t, +t)$ 형태가 된다. 새 깊이 프레임이 들어올 때마다 이 값을 가중 평균으로 갱신하면, 노이즈가 점진적으로 평균화되면서 표면이 점점 선명해진다. 표면 추출은 TSDF의 zero-crossing에 marching cubes를 적용하면 된다.

이 방법은 정확했다. 그러나 복셀 그리드는 메모리를 많이 먹었고, 실시간 갱신은 당시 하드웨어로는 불가능했다. Curless-Levoy의 논문은 이후 15년 동안 그래픽스 교과서에 머물렀다.

그 15년 사이에 두 가지가 바뀌었다. GPU가 GPGPU 시대로 진입했고, Kinect가 등장했다.

---

## 9.2 KinectFusion: 15년 묵은 그래픽스 기법의 부활

Microsoft Research는 2010년 Xbox 360용 Kinect를 $150에 출시했다. 구조광(structured light) 방식으로 깊이를 측정하는 이 센서는 VGA 해상도의 깊이 맵을 30Hz로 스트리밍했다. 정밀도는 연구용 ToF 카메라보다 낮았지만, 가격은 100분의 1이었다. 해커들이 먼저 반응했다. 출시 몇 주 만에 오픈소스 드라이버가 공개됐고, 연구자들이 그 뒤를 따랐다.

Newcombe는 그 무렵 Microsoft Research Cambridge로 자리를 옮겼고, Shahram Izadi 팀과 함께 GPU 기반 dense SLAM을 준비하고 있었다. Kinect가 출시됐을 때 그들에게는 이미 파이프라인의 윤곽이 있었다. Kinect가 공급한 깊이 스트림이 나머지를 채웠다. 결과가 2011년 ISMAR에서 발표된 [Newcombe et al. 2011. KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378)이다.

> 🔗 **차용.** KinectFusion의 핵심 표현인 TSDF는 Curless & Levoy(1996)가 오프라인 3D 스캐닝을 위해 고안한 것이다. Newcombe 팀은 이를 GPU의 병렬 복셀 갱신으로 실시간화했다.

파이프라인은 네 단계로 구성된다.

깊이 전처리: 원시 깊이 맵에서 bilateral filter로 노이즈를 줄이고 표면 법선을 계산한다.

ICP 추적: 현재 프레임의 포인트 클라우드를 이전 TSDF에서 ray-cast한 가상 표면에 정렬한다. Besl & McKay(1992)의 **ICP(Iterative Closest Point)**를 point-to-plane 변형으로 GPU에서 수천 번 반복한다. 결과는 카메라의 6-DoF 포즈다.

point-to-plane ICP의 목적함수는 다음과 같다. 현재 프레임의 포인트 $\mathbf{p}_i$를 변환 $T = (R, \mathbf{t})$로 움직인 뒤 대응 점 $\hat{\mathbf{p}}_i$(ray-cast 표면)과 법선 $\hat{\mathbf{n}}_i$에 대해

$$E(R, \mathbf{t}) = \sum_i \bigl(\hat{\mathbf{n}}_i^\top (R\,\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i)\bigr)^2$$

을 최소화한다. 원래 Besl-McKay의 point-to-point($\|R\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i\|^2$)와 달리 법선 방향 오차만 측정하므로, 표면에 접하는 방향의 미끄러짐에 덜 민감하다. 소회전 근사 $R \approx I + [\boldsymbol{\omega}]_\times$ 를 적용하면 $E$는 6-DoF 벡터 $(\boldsymbol{\omega}, \mathbf{t})$에 대한 선형 최소제곱 문제로 바뀌어 GPU에서 병렬 감소(parallel reduction)로 한 번에 풀린다.

> 🔗 **차용.** KinectFusion의 추적 단계는 Besl & McKay(1992) ICP를 직접 계승한다. 고전 로봇공학 문헌의 기법을 GPU 밀도로 다시 꺼낸 것이다.

TSDF 통합: 추정된 포즈로 깊이 맵을 복셀 그리드에 투영해 TSDF 값을 갱신한다. 복셀은 512³ 크기, 물리적으로 약 3.5m 큐브에 해당한다.

표면 렌더링: TSDF의 zero-crossing을 ray marching으로 찾아 실시간 메시를 렌더링한다. 이 결과가 다음 ICP 추적의 참조 표면이 된다.

Newcombe는 같은 해 DTAM을 단안 카메라 dense SLAM으로 발표했다. KinectFusion은 그 자매 연구다. DTAM이 GPU를 써서 단안의 광도 일관성을 최적화했다면, KinectFusion은 같은 GPU를 깊이 통합에 투입했다. 두 논문의 저자 목록이 겹치는 이유다.

> 🔗 **차용.** KinectFusion과 DTAM은 같은 해 같은 연구자가 발표한 두 dense 시스템이다. DTAM의 GPU dense 파이프라인 철학이 KinectFusion으로 자연스럽게 이식됐고, 센서만 달랐다.

512³ TSDF가 30Hz로 갱신됐고, 실내 방 한 칸을 몇 분 안에 dense mesh로 복원했다. 추적 drift는 feature-based 방식보다 훨씬 작았다. ICP가 절대 표면에 수렴하는 구조이기 때문이다.

한계도 명확했다. 512³ 복셀 그리드는 고정된 공간 범위만 다룰 수 있었다. 방을 벗어나면 복셀이 포화되거나 기존 복셀을 덮어써야 했다. loop closure가 없었다. 그리고 Kinect의 IR 구조광은 햇빛 아래에서 작동하지 않았다. 실외는 처음부터 범위 밖이었다.

---

## 9.3 Kintinuous: 이동하는 지도

KinectFusion이 발표된 직후 Whelan은 Imperial College에서 이 한계에 달려들었다. 고정 크기 TSDF 볼륨이 문제라면, 카메라를 따라 이동하면 된다.

2013년 Whelan 등이 발표한 Kintinuous는 "rolling TSDF volume"을 도입했다. 카메라가 볼륨 경계에 가까워지면 반대쪽 슬라이스를 메시로 출력하고 해제한 뒤, 새 슬라이스를 앞에 붙인다. 메모리는 일정하게 유지되면서 카메라는 무한히 이동할 수 있다.

실내 복도 전체를 걷는 데모는 KinectFusion이 보여주지 못한 것이었다. 그러나 loop closure는 여전히 없었다. 긴 복도를 걸어서 원점으로 돌아왔을 때 두 끝이 맞지 않는 문제는 해결되지 않았다. 재구성 품질도 sparse SLAM이 쌓아온 submap 정합 방법에 비해 열위였다.

---

## 9.4 ElasticFusion: Surfel과 비강체 변형

Whelan은 Kintinuous 이후 방향을 바꿨다. TSDF 복셀 대신 surfel을 선택했다.

**surfel(surface element)**은 위치, 법선, 반경, 색상을 가진 점이다. 컴퓨터 그래픽스에서 Pfister 등(2000)이 렌더링 표현으로 제안한 개념이었다. 복셀 그리드에 비해 불규칙하고 표면에 밀착하는 구조다.

> 🔗 **차용.** ElasticFusion의 surfel 표현은 Pfister 등(2000)의 그래픽스 렌더링 기법을 SLAM의 맵 표현으로 이식한 것이다.

[Whelan et al. 2016. ElasticFusion](https://doi.org/10.1177/0278364916669237)의 핵심 기여는 두 가지다. 첫째, surfel 기반 dense map. 둘째, *non-rigid deformation*을 이용한 loop closure.

기존 dense SLAM의 loop closure는 어려웠다. 전역 메시나 복셀 그리드를 loop closure 정보에 맞춰 수정하려면 비용이 컸다. ElasticFusion은 surfel 집합을 변형 그래프(deformation graph)와 연결하고, loop closure가 감지되면 그래프를 변형해 전체 맵에 오차를 분산시켰다. 메시 수준에서의 비강체 변형이었다.

구체적으로, deformation graph의 각 노드 $g_k$는 위치 $\mathbf{v}_k$와 회전 $R_k$, 이동 $\mathbf{t}_k$를 가진다. surfel $s$는 가장 가까운 $K$개 노드의 영향권 안에 놓이고, surfel의 변형 후 위치는

$$\tilde{\mathbf{p}}_s = \sum_{k \in \mathcal{N}(s)} w_k \bigl(R_k (\mathbf{p}_s - \mathbf{v}_k) + \mathbf{v}_k + \mathbf{t}_k\bigr)$$

로 계산된다(가중치 $w_k$는 거리 기반 감쇠). loop closure 제약이 추가되면 그래프 노드들의 $(R_k, \mathbf{t}_k)$를 Gauss-Newton으로 최적화해 오차를 전역에 분산한다. TSDF를 통째로 다시 쌓지 않고도 dense 맵 전체를 일관되게 수정할 수 있었던 이유다.

KITTI나 TUM RGB-D 벤치마크가 아니라 실내 재구성 품질 자체로 평가하면 ElasticFusion은 당시 최고 수준이었다. ICL-NUIM 데이터셋에서 ATE RMSE 1cm 이하를 기록했다. 실시간성을 유지하면서 이 수준에 도달한 시스템은 그 이전에 없었다.

---

## 9.5 BundleFusion: 오프라인 SfM 품질을 온라인으로

2017년 Dai, Nießner, Zollhöfer, Izadi, Theobalt가 ACM Transactions on Graphics에 발표한 [Dai et al. 2017. BundleFusion](https://doi.org/10.1145/3072959.3054739)은 다른 방향에서 문제에 접근했다. KinectFusion 계열이 실시간성을 타협하지 않으면서 품질을 높이려 했다면, BundleFusion은 GPU 연산을 최대한 투입해 온라인 시스템에서도 SfM 수준의 번들 조정을 실행하겠다는 목표였다.

핵심 아이디어는 계층적 최적화다. 가장 빠른 층에서는 현재 프레임과 이전 프레임 사이의 dense depth alignment로 초기 포즈를 잡는다. 그 위 층에서는 SIFT feature를 이용한 sparse frame-to-frame alignment로 보정하고, 세 번째 층에서 sliding-window global bundle adjustment가 누적된 프레임들의 포즈를 재최적화한다. Bundle adjustment는 프레임이 누적될수록 과거 포즈도 재추정한다. "retroactive pose correction"이라고 불린 이 방식은 오프라인 SfM 파이프라인이 모든 데이터를 가진 뒤 정합하는 것과 유사한 효과를 온라인으로 달성하려 했다. 갱신된 포즈 시퀀스를 TSDF에 역투영해 재통합하므로, 추적 오류가 맵에 그대로 쌓이지 않는다.

Dai 팀이 TUM RGB-D 벤치마크에서 보고한 수치는 ElasticFusion을 능가했다. 시각적 재구성 품질도 당시 기준으로 오프라인 COLMAP 파이프라인에 근접했다.

> 📜 **예언 vs 실제.** Dai 등은 2017년 BundleFusion 논문에서 GPU 연산력의 지속적 발전이 real-time online global optimization을 조만간 대중화할 것이라 내다봤다. GPU 연산력은 예언대로 빠르게 증가했다. 그러나 관심은 dense SLAM이 아니라 NeRF(Neural Radiance Field)로 이동했다. 고품질 실내 재구성의 사실상 표준은 2021년 이후 COLMAP + NeRF 파이프라인이 됐다. BundleFusion이 열려고 했던 경로 자체가 다른 기술로 우회됐다. `[기술변화]`

---

## 9.6 하드웨어와 알고리즘의 공진화

KinectFusion에서 BundleFusion까지의 6년을 이해하는 한 가지 방법은 알고리즘의 발전으로 보는 것이다. 더 정확한 방법은 하드웨어와 알고리즘이 서로를 밀어붙인 과정으로 보는 것이다.

Kinect 1세대는 구조광 방식이었다. 깊이 정밀도는 미터 범위에서 수 밀리미터였지만 햇빛 아래에서는 IR 패턴이 잡히지 않았다. 2013년 출시된 Kinect 2는 ToF(Time-of-Flight) 방식으로 바꿨다. 정밀도가 올라갔고 동적 범위도 나아졌다. Intel의 RealSense 시리즈가 뒤를 이었다. 센서 선택지가 늘어날수록 알고리즘이 가정할 수 있는 깊이 품질이 달라졌고, 연구자들은 더 작은 노이즈를 활용하거나 더 큰 노이즈를 견디는 방식을 실험했다.

GPU 쪽에서는 CUDA 생태계가 성숙했다. KinectFusion이 나온 2011년의 Tesla 아키텍처와 BundleFusion이 나온 2017년의 Pascal 아키텍처 사이에 부동소수점 성능은 10배 이상 증가했다. Whelan이 ElasticFusion에서, Dai가 BundleFusion에서 점점 더 무거운 최적화를 실시간으로 실행할 수 있었던 것은 알고리즘만의 성과가 아니었다.

Kinect가 $150이 아니라 $15,000이었다면, 이 흐름은 5년 이상 늦게 시작됐을 것이다. 소비자 시장용 센서가 연구의 속도를 끌었다.

> 📜 **예언 vs 실제.** Newcombe 등은 2011년 KinectFusion 논문에서 scale-up, outdoor 환경, 더 큰 볼륨으로의 확장을 후속 과제로 꼽았다. scale-up과 더 큰 볼륨은 Kintinuous, ElasticFusion, BundleFusion이 차례로 해결했다. outdoor는 다른 결론에 도달했다. IR 구조광은 햇빛 아래에서 패턴이 잡히지 않는다. RGB-D 기반 dense SLAM은 그렇게 실내에 묶였고, outdoor는 LiDAR가 맡았다. `[기술변화]`

---

## 9.7 "모든 것을 dense로"의 퇴각

2011년부터 2017년 사이 dense RGB-D SLAM은 Visual SLAM의 주된 방향이 될 것처럼 보였다. 실제 전개는 그렇지 않았다.

sparse backend는 계속 지배했다. ORB-SLAM2와 VINS-Mono로 대표되는 2015년 이후의 실용 SLAM 시스템들은 dense 맵을 기본으로 삼지 않았다. 이유는 복합적이었다. 512³ TSDF는 512MB 이상이 필요해 모바일 플랫폼이나 임베디드 시스템에서는 감당하기 어려웠다. Octree나 해시맵 기반 변형(Voxblox, OctoMap)이 이를 완화하려 했지만 sparse 방식의 효율성과는 격차가 있었다. 실시간 dense 처리는 GPU를 전제했는데, 자율주행 차량의 임베디드 프로세서나 드론의 경량 플랫폼에서는 KinectFusion 수준의 파이프라인을 돌리기 어려웠다. Kinect의 IR depth가 실외에서 작동하지 않는다는 점도 발목을 잡았다. 자율주행과 드론처럼 상용화 요구가 큰 분야 대부분이 실외 환경이었다.

2020년을 전후해 NeRF가 등장하면서 고품질 dense 재구성을 원하는 수요는 NeRF와 3D Gaussian Splatting으로 이동했다. RGB-D SLAM은 localization과 mapping을 분리하는 구조 속에서 depth를 추적 보조로 쓰는 수준으로 좁아졌다.

dense 시대는 짧았지만 흔적은 남았다. TSDF 표현은 자율주행용 occupancy map으로 이어졌고, ICP는 LiDAR SLAM의 표준 추적 수단이 됐다. 접근 방식은 퇴각했지만 부품들은 다른 시스템 안으로 흩어졌다.

---

## 🧭 아직 열린 것

대규모 실외 dense 재구성. IR 구조광의 햇빛 취약성은 active depth 센서 전반의 문제다. LiDAR는 더 먼 거리를 다루지만 색상과 세밀한 표면 정보가 빈약하다. RGB-D 방식으로 실외 대규모 환경을 dense하게 처리하는 방법은 2026년 기준으로 아직 없다. Stereo depth estimation이 학습 기반으로 빠르게 발전하고 있어 일부 연구들이 대안을 탐색 중이지만, 어두운 영역·반사면·원거리에서의 한계가 해결되지 않았다.

동적 장면의 dense 재구성. KinectFusion부터 BundleFusion까지 모든 시스템이 정적 장면을 전제로 설계됐다. 사람이 걸어 다니는 공간을 dense하게 재구성하려면 동적 물체를 분리해야 하는데, 이는 실시간 semantic segmentation과 dense SLAM의 결합을 요구한다. DynaSLAM, MaskFusion 등이 시도했지만 계산 비용과 robustness 모두에서 실용 배포 수준에 미치지 못한다.

TSDF 계열의 메모리 효율. 복셀 그리드의 메모리 비용은 Voxblox의 해시 구조, OctoMap의 octree 압축으로 줄어들었다. 그러나 건물 층 단위, 도시 블록 단위의 dense 표현은 여전히 수십 기가바이트 수준이다. 어떤 해상도를 어느 영역에서 유지할지를 자동으로 결정하는 adaptive resolution dense map은 아직 범용 해법이 없다. Instant-NGP와 같은 implicit neural representation이 이 문제에 접근하고 있지만, 실시간 갱신과 쿼리 속도는 트레이드오프가 남아 있다.

dense SLAM이 실내 방 한 칸을 메시로 채우는 동안, 그 방으로 돌아오는 문제는 별도의 계보가 맡고 있었다. KinectFusion은 loop closure가 없었다. 있었다면 어떻게 됐을까. 이미 그 답을 가지고 있는 연구자들이 옥스퍼드에 있었다. 그들은 지도를 쌓는 것이 아니라 장소를 기억하는 문제를 붙잡고 있었다.
