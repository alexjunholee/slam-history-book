# 12장 RGB-D 조밀 재구성

2011년 10월 ISMAR 학회장, Imperial College London의 Richard Newcombe가 발표한 KinectFusion 데모 영상은 연구자들의 시선을 단숨에 사로잡았다. 손에 든 저렴한 Kinect 센서 하나로 방 안 이곳저곳을 훑고 지나가는 순간, 화면 속 3D 공간이 매끈하고 빈틈없는 메시로 실시간으로 빚어졌기 때문이다. 사실 이 기적 같은 파이프라인을 이루는 핵심 조각들은 모두 이미 존재하던 것들이었다. Curless와 Levoy가 1996년 컴퓨터 그래픽스 분야에서 고안한 부호 거리 함수(TSDF) 표현, Besl과 McKay가 1992년 발표한 ICP 정합 알고리즘, 그리고 Microsoft가 막 게임기용으로 내놓은 보급형 Kinect 센서다. 이 셋이 2011년 GPU 병렬 연산 위에서 마침내 한데 만났다. Newcombe가 단안 카메라의 밝기만으로 조밀 재구성을 시도한 DTAM(11장)을 내놓은 것과 정확히 같은 해의 일이었다.

---

## 12.1 Kinect 이전의 dense 재구성

2011년 이전이라고 해서 조밀한(dense) 3D 재구성 기술이 없었던 것은 아니다. [Pollefeys 등(IJCV 2008)](https://doi.org/10.1007/s11263-007-0086-4)처럼 차량에 실은 카메라와 GPS/INS를 결합해 도시 3D 모델을 실시간으로 구축한 시스템도 있었지만, 대부분의 dense 복원은 오프라인 후처리 작업이었다. 구조광(structured light: 정해진 빛 무늬를 투사해 그 왜곡 정도로 깊이를 재는 방식) 스캐너나 스테레오 매칭으로 긁어모은 point cloud를 몇 시간에 걸쳐 정합해 합치는 식이었고, 스캔 장비 자체가 고가여서 연구실 문턱을 넘기도 어려웠다. 당시 SLAM 주류는 소수의 희소한 랜드마크만으로도 궤적을 잘 뽑아내고 있었기에, 조밀한 표면 복원은 주로 컴퓨터 그래픽스 연구자들의 영역으로 남아 있었다.

그 그래픽스 진영의 고전이 [Curless와 Levoy의 1996년 SIGGRAPH 논문 "A Volumetric Method for Building Complex Models from Range Images"](https://graphics.stanford.edu/papers/volrange/volrange.pdf)다. 두 사람은 3D 공간을 균일한 복셀(voxel) 격자로 쪼갠 뒤, 각 복셀에 센서 시선 방향에서 잰 표면까지의 부호 있는 거리를 가중치와 함께 누적했다. 이 거리 값은 표면 바로 앞뒤에서만 기하학적 의미를 지니므로 측정점 주변의 일정 한계 $t$ 바깥은 잘라 내어 썼는데, 훗날 KinectFusion이 이를 **TSDF(Truncated Signed Distance Function, 잘라 낸 부호 거리 함수)**라고 명명했다. 부호는 센서에서 표면을 바라볼 때 표면 앞쪽(빈 공간)이 양수, 표면 뒤쪽(물체 내부)이 음수이며, 한계 $t$ 안으로 잘라 낸다는 것은 $\text{TSDF}(x) = \text{clip}(d(x), -t, +t)$로 묶는다는 뜻이다. 새 깊이 프레임이 들어올 때마다 복셀 값을 가중 평균으로 갱신하면 센서 잡음이 서서히 상쇄되며 표면이 또렷해지고, 최종 표면은 TSDF 값이 0이 되는 영점 교차면(zero-crossing)에 marching cubes를 돌려 3D 메시로 뽑아낸다.

원리는 명쾌했지만 1990년대 하드웨어로는 거대한 복셀 격자를 실시간으로 갱신할 엄두를 내지 못했다. 이 교착 상태를 깬 두 가지 변화가 바로 GPU를 범용 병렬 연산에 쓰는 GPGPU의 성숙과, 누구나 살 수 있는 가격의 Kinect 등장이었다.

---

## 12.2 KinectFusion과 TSDF

2010년 11월 4일 Microsoft는 Xbox 360용 Kinect를 북미 시장에서 149.99달러에 출시했다. 구조광 원리로 640×480 해상도의 깊이 맵을 30 Hz로 쏟아 내는 이 게임기 주변기기는 기존 연구용 ToF 카메라보다 정밀도는 다소 떨어져도 가격은 비교할 수 없이 쌌다. 출시 불과 엿새 뒤인 11월 10일, Hector Martin이 훗날 `libfreenect`로 이어진 오픈소스 드라이버를 공개하자 전 세계 로보틱스·비전 연구실이 즉각 Kinect를 사들이기 시작했다.

당시 박사과정생이던 Newcombe는 Microsoft Research Cambridge의 Shahram Izadi 팀과 손을 잡고 Kinect 깊이 스트림을 GPU 위에서 곧바로 처리하는 dense SLAM을 짰다. 그 결실이 2011년 ISMAR에 발표된 [Newcombe et al. 2011. KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378)이다.

KinectFusion의 실시간 루프는 네 단계가 맞물려 돌아간다. 먼저 **깊이 전처리** 단계에서 경계는 살리고 잡음만 죽이는 bilateral filter를 원시 깊이 맵에 씌운 뒤, 픽셀마다 표면 법선을 뽑는다. 이어 **ICP 추적** 단계에서는 현재 프레임의 point cloud를, 지금까지 누적한 TSDF 볼륨에서 ray-casting으로 그려 낸 가상 표면에 정합한다. 여기에는 [Besl & McKay(1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)의 **ICP**(Iterative Closest Point)를 point-to-plane 형태로 변형한 알고리즘이 쓰였다. 3단계 영상 피라미드에서 거친 해상도부터 고운 해상도 순으로 최대 4·5·10회 반복을 도는데, 매 반복마다 GPU가 수많은 정점–법선 대응에서 나온 정규방정식 항을 병렬로 합산해 카메라의 6-DoF 자세를 찾아낸다.

point-to-plane ICP에서 현재 프레임의 점 $\mathbf{p}_i$를 변환 $T = (R, \mathbf{t})$로 옮긴 뒤 ray-cast 표면의 대응점 $\hat{\mathbf{p}}_i$ 및 법선 $\hat{\mathbf{n}}_i$과 맞추는 목적함수는 다음과 같다.

$$E(R, \mathbf{t}) = \sum_i \bigl(\hat{\mathbf{n}}_i^\top (R\,\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i)\bigr)^2$$

Besl과 McKay의 원래 point-to-point 오차($\|R\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i\|^2$)가 두 점 사이의 유클리드 거리 전체를 줄이려 했다면, point-to-plane 오차는 법선 방향 성분만 재고, 표면을 따라 미끄러지는 접선 방향의 어긋남은 관대하게 넘기므로 수렴이 훨씬 빨라진다. 여기에 미소 회전 근사 $R \approx I + [\boldsymbol{\omega}]_\times$를 얹으면 $E$는 6차원 벡터 $(\boldsymbol{\omega}, \mathbf{t})$에 대한 선형 최소자승 문제가 되어 GPU에서 parallel reduction으로 한 번에 풀린다.

자세를 구한 뒤에는 **TSDF 통합** 단계로 넘어가 현재 깊이 맵을 복셀 격자에 투영해 TSDF 가중 평균을 갱신한다(논문 Fig. 13에서는 3 m³ 볼륨을 64³에서 512³ 해상도까지 달리하며 처리 시간을 측정했다). 마지막 **표면 렌더링** 단계에서는 TSDF의 zero-crossing을 ray marching으로 추적해 새 vertex map과 normal map을 그려 낸다. 이 가상 표면이 곧 다음 프레임 ICP 추적의 기준 모델이 된다.

이처럼 직전 프레임이 아니라 지금까지 누적한 전체 모델에 프레임을 맞추는 model-to-frame 추적 덕분에 KinectFusion은 30 Hz 실시간으로 방 한 칸을 깔끔하게 복원하면서도 좁은 공간 안에서 추적 drift를 억제할 수 있었다(물론 그 누적 모델 역시 과거 추정 자세에 기대고 있으므로 절대 오차에서 완전히 자유로운 것은 아니었다). 그러나 한계도 뚜렷했다. 미리 할당한 고정 복셀 격자를 카메라가 벗어나는 순간 더는 지도를 확장할 수 없었고, loop closure 기능도 없었다. 게다가 Kinect가 쏘는 적외선 구조광 패턴은 야외의 햇빛 아래에서 무력화됐다.

> 🔗 **차용.** KinectFusion의 체적 지도 표현은 Curless & Levoy(1996)가 오프라인 3D 스캐닝을 위해 고안한 TSDF를 그대로 물려받았다. Newcombe 팀의 돌파구는 GPU 병렬 연산으로 복셀 갱신을 30 Hz 실시간 영역으로 끌어내린 데 있다.

> 🔗 **차용.** 추적 엔진 역시 Besl & McKay(1992)가 IEEE PAMI에 발표한 고전 ICP를 GPU 위에서 조밀한 point-to-plane 형태로 병렬화한 결과물이다.

---

## 12.3 Kintinuous — rolling volume

KinectFusion이 발표되자마자 연구자들은 가장 답답한 병목이었던 고정 볼륨 크기 문제부터 공략했다. Maynooth 대학의 Thomas Whelan은 MIT의 Michael Kaess, John Leonard 등과 함께 TSDF 볼륨을 공간에 못 박아 두는 대신 카메라를 따라 굴러가게 만들었다.

2012년 7월 시드니 RSS 워크숍 "RGB-D: Advanced Reasoning with Depth Cameras"에서 발표된 [Whelan 등의 Kintinuous](https://www.cs.cmu.edu/~kaess/pub/Whelan12rssw.pdf)는 3차원 TSDF 배열을 순환 버퍼(cyclic buffer)로 다룬다. 카메라가 현재 볼륨 경계에 다가가면 볼륨을 카메라 진행 방향으로 한 칸씩 미끄러뜨리고, 반대편 경계 밖으로 밀려난 복셀 slice에서는 즉시 point cloud를 추출해 삼각형 메시에 덧붙여 저장한다. 덕분에 GPU 메모리는 일정하게 묶어 둔 채로 복도처럼 긴 공간을 끝없이 이어서 재구성할 수 있게 됐다.

그럼에도 한계는 절반만 해결됐다. 긴 복도를 한 바퀴 돌아 제자리로 돌아왔을 때 누적된 drift를 해소할 loop closure가 여전히 없었기 때문이다. 국소 표면은 촘촘하고 매끈했지만, 출발점과 끝점이 어긋난 전역 지도를 한꺼번에 펴 줄 방법이 필요했다.

---

## 12.4 ElasticFusion: Surfel과 비강체 변형

전역 복셀 격자나 삼각형 메시를 통째로 뜯어고치지 않고 dense 지도의 루프를 닫을 수는 없을까? Whelan은 Kintinuous 다음 프로젝트에서 아예 지도의 바탕을 TSDF 복셀이 아닌 **surfel**로 갈아 끼웠다.

**surfel(surface element, 표면 요소)**은 3D 위치와 법선, 반지름, 색상을 품은 납작한 원반 점으로, 컴퓨터 그래픽스에서 [Pfister 등(2000)](https://www.merl.com/publications/docs/TR2000-10.pdf)이 렌더링용으로 제안한 표현이다. 3차원 공간 전체를 균일 격자로 채우는 복셀과 달리, surfel은 실제 물체 표면 위에만 불규칙하게 놓인다.

Whelan 등이 RSS 2015에 처음 공개하고 2016년 IJRR에 [확장판](https://doi.org/10.1177/0278364916669237)을 실은 **ElasticFusion**은 바로 이 surfel 지도에 *비강체 변형*(non-rigid deformation)을 결합해 dense SLAM의 loop closure 난제를 풀었다. 거대한 복셀 볼륨은 한 번 틀어지면 처음부터 재통합해야 하지만, 점들의 집합인 surfel은 공간에 듬성듬성 배치된 변형 그래프(deformation graph)에 매달아 고무처럼 휘어 줄 수 있기 때문이다.

deformation graph의 각 노드 $g_k$는 공간 위치 $\mathbf{v}_k$와 회전 $R_k$, 이동 $\mathbf{t}_k$를 가진다. 모든 surfel $s$의 변형 후 위치는 인접한 $K$개 노드의 영향을 가중치에 따라 반영해 다음과 같이 계산된다(가중치 $w_k$는 노드와의 거리가 멀수록 작아진다).

$$\tilde{\mathbf{p}}_s = \sum_{k \in \mathcal{N}(s)} w_k \bigl(R_k (\mathbf{p}_s - \mathbf{v}_k) + \mathbf{v}_k + \mathbf{t}_k\bigr)$$

로봇이 이전에 지나간 장소를 다시 만나 loop closure 제약이 걸리면, ElasticFusion은 이 변형 그래프 노드들의 $(R_k, \mathbf{t}_k)$를 Gauss-Newton으로 최적화해 누적 오차를 지도 전체에 매끄럽게 분산시킨다. 지도를 처음부터 다시 쌓지 않고도 조밀한 전역 일관성을 되찾은 것이다.

합성 데이터셋 ICL-NUIM에서 ElasticFusion은 kt0·kt1·kt2 시퀀스 모두 1.4 cm 이하의 ATE RMSE를 기록했고, 그중 kt0와 kt1은 0.9 cm까지 떨어졌다(다만 kt3에서는 오차가 이보다 훨씬 컸다).

---

## 12.5 BundleFusion

비강체 변형이 사후적으로 지도를 구부려 오차를 나눈다면, 아예 GPU 자원을 아낌없이 투입해 모든 프레임의 자세를 처음부터 끝까지 온라인 bundle adjustment로 함께 풀 순 없을까? 2017년 ACM Transactions on Graphics에 실린 [Dai et al. 2017. BundleFusion](https://doi.org/10.1145/3072959.3054739)(Dai, Nießner, Zollhöfer, Izadi, Theobalt)이 바로 그 정공법이었다.

BundleFusion은 새 RGB-D 프레임이 들어올 때마다 지금까지 쌓인 전체 입력에 대해 카메라 자세를 전역적으로 다시 최적화한다. 계산량을 감당하기 위해 최적화는 두 층으로 나뉜다. 먼저 연속한 11개 프레임을 하나의 chunk로 묶어 국소 최적화를 돌리고, 각 chunk의 대표 keyframe들을 전역 계층에서 한꺼번에 정렬한다. 이때 대응점은 sparse SIFT 특징점과 dense 기하·밝기 정합을 함께 써서 튼튼하게 잡으며, 추적을 크게 잃더라도 곧장 전역 지도에서 relocalization으로 복귀한다. 더 놀라운 점은 지도 갱신 방식이다. 전역 최적화로 과거 프레임의 자세가 수정되면, BundleFusion은 그 프레임을 TSDF에서 빼낸(de-integration) 뒤 바뀐 자세로 실시간으로 다시 통합해(re-integration) 지도 자체를 항상 최신 전역 자세와 일치시켰다.

TUM RGB-D 등 주요 벤치마크에서 BundleFusion은 당시 온라인 시스템들을 능가하는 스캔 완성도와 정밀도를 입증했다.

> 📜 **예언 vs 실제.** BundleFusion 초록은 기존 온라인 기법을 넘어 오프라인 재구성에 필적하는 품질을 "전례 없는 속도와 스캔 완성도(unprecedented speed and scan completeness)"로 달성했다고 선언했다. 이후 GPU 성능은 계속 치솟았지만, 2021년 무렵부터 3D 비전 연구의 주된 관심은 RGB-D TSDF 통합 대신 COLMAP으로 자세를 구한 뒤 NeRF로 장면을 표현하는 쪽, 곧 고품질 novel-view synthesis와 신경망 기반 장면 표현으로 상당 부분 이동했다.

---

## 12.6 하드웨어와 알고리즘

돌이켜 보면 2011년부터 2017년까지 이어진 dense RGB-D SLAM의 황금기는 알고리즘과 하드웨어의 공진화였다. 1세대 Kinect는 구조광 덕분에 미터 거리에서 수 밀리미터 수준의 깊이 정밀도를 냈지만 햇빛 아래에서 무용지물이 됐다. 2013년 11월 Xbox One과 함께 출시된 Kinect 2는 빛의 비행 시간을 재는 ToF(Time-of-Flight) 방식으로 갈아타며 정밀도와 시야각을 끌어올렸고, 뒤이어 Intel RealSense 시리즈까지 가세해 연구자들에게 다양한 깊이 센서를 쥐여 주었다. 물론 센서마다 잡음 특성과 왜곡 양상이 달랐기에 알고리즘도 센서에 맞춰 진화해야 했다.

같은 기간 GPU 진영에서는 CUDA 생태계가 무르익고 연산량과 메모리 대역폭이 가파르게 성장했다. 연구실에서 누구나 손에 쥘 수 있던 싼 깊이 센서와 폭발하는 GPU 연산력이 맞물리지 않았다면, ElasticFusion의 비강체 변형이나 BundleFusion의 전역 실시간 re-integration은 2010년대 중반에 실현되기 어려웠을 것이다.

> 📜 **예언 vs 실제.** KinectFusion이 남긴 세 가지 숙제—공간 범위의 제한, 누적 drift, 실외 사용의 어려움—는 이후 연구의 로드맵이 됐다. 공간 범위와 drift는 Kintinuous, ElasticFusion, BundleFusion이 차례로 풀어냈지만, 햇빛이 적외선 패턴을 삼켜 버리는 실외 환경의 벽은 active IR 센서 자체의 물리적 제약 탓에 끝내 넘지 못했다(햇빛에 버티는 정도는 센서 방식마다 다르다). 결국 실외의 넓은 공간을 조밀하게 훑는 주력 센서의 자리는 LiDAR가 차지했다.

---

## 12.7 dense 방식이 주류가 되지 못한 이유

2011년부터 2017년 사이만 해도 dense RGB-D SLAM이 곧 모든 Visual SLAM의 표준이 될 것처럼 보였다. 하지만 정작 2015년 이후 실제 로봇과 드론 현장을 장악한 [ORB-SLAM2](https://arxiv.org/abs/1610.06475)와 [VINS-Mono](https://arxiv.org/abs/1708.03852)는 희소 특징점을 택했고, dense 지도를 기본 내장하지 않았다.

가장 현실적인 걸림돌은 메모리와 연산 예산이었다. KinectFusion처럼 복셀마다 16비트 TSDF 값과 16비트 가중치를 저장하면 $512^3$ 격자 하나에만 512 MB가 들어가, 모바일 기기나 임베디드 보드에서는 감당하기 버거웠다. 필요한 영역만 해시 블록으로 할당하는 [Voxblox](https://arxiv.org/abs/1611.03631)나 octree로 점유 확률을 쪼개 담는 [OctoMap](https://www.hrl.uni-bonn.de/papers/wurm10octomap.pdf)이 메모리 부담을 크게 덜어 주었지만 여전히 희소 랜드마크 지도만큼 가볍지는 못했다. 게다가 실시간 dense 추적에 필수였던 고성능 GPU는 드론이나 모바일 로봇에 싣기 무거웠고, 적외선 깊이 센서가 실외 햇빛에 취약하다는 점도 자율주행·야외 로보틱스의 요구와 어긋났다.

그렇다고 dense 지도 연구가 헛되이 사라진 것은 아니다. 자료구조 차원에서 KinectFusion의 고정 격자는 훨씬 영리한 계층 구조로 진화했다. 그래픽스와 시뮬레이션에서 출발한 [Museth(2013)의 VDB](https://doi.org/10.1145/2487228.2487235)(OpenVDB)는 동적으로 확장되는 희소 루트에 얕고 넓은 B+ tree를 엮어, 빈 공간은 건너뛰면서도 빠른 임의 접근을 보장했다. 최근에는 [Reijgwart 등(2023)의 wavemap](https://arxiv.org/abs/2306.01279)이 Haar wavelet으로 점유율(occupancy)을 압축해 다해상도 정보를 단일 구조에 담았다.

격자를 벗어나 공간을 연속 함수로 다루는 시도도 궤를 같이했다. [O'Callaghan과 Ramos(2012)의 GPOM(Gaussian Process Occupancy Map)](https://doi.org/10.1177/0278364911421039)은 점유 지도를 분류 문제로 삼아 변형된 Gaussian Process로 풀어, 가려진 사각지대나 센서 빔 사이 빈틈의 점유 확률을 분산과 함께 추정했다. [Ramos와 Ott(2016)의 Hilbert Map](https://doi.org/10.1177/0278364916684382)은 Hilbert 공간 특징 위에 로지스틱 회귀를 올려 데이터가 들어오는 즉시 stochastic gradient descent로 갱신하는 연속 점유 지도를 구현했다.

한편 ElasticFusion의 surfel 아이디어는 센서와 무대를 바꿔 살아남았다. [Behley와 Stachniss(2018)의 SuMa](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/behley2018rss.pdf)는 회전식 3D LiDAR 스캔을 surfel 지도에서 렌더링한 모델 뷰에 맞춰 추적함으로써 도시 규모 환경에서도 loop closure를 포함한 온라인 dense mapping을 해냈다(→ 21장).

2020년 NeRF의 등장 이후 고품질 시각 재구성의 무게중심은 NeRF와 3D Gaussian Splatting으로 넘어갔지만, RGB-D 깊이는 여전히 추적과 형상 학습의 튼튼한 제약으로 쓰인다. 무엇보다 TSDF 같은 체적 표현은 장애물 회피와 경로 계획을 위한 로봇 지도의 주류로 남았고, KinectFusion이 갈고닦은 GPU 기반 ICP는 LiDAR SLAM 정합의 핵심 엔진으로 자리 잡았다.

---

## 아직 열린 문제

**대규모 실외 dense 재구성.** 햇빛 아래에서 무력해지는 약점은 적외선 구조광을 비롯한 능동 깊이 센서 전반이 안고 있는 물리적 한계다. LiDAR는 멀리까지 정확히 재지만 색상과 조밀한 표면 질감이 빠지고, 학습 기반 스테레오 깊이 추정은 빠른 발전에도 불구하고 여전히 어두운 밤, 반사면, 먼 거리에서 흔들린다.

**동적 장면의 dense 재구성.** KinectFusion부터 BundleFusion까지 고전 dense SLAM은 세상이 정지해 있다고 가정했다. 사람과 차가 오가는 실제 공간에서 깔끔한 3D 지도를 쌓으려면, 움직이는 물체를 semantic segmentation이나 기하 residual로 실시간으로 가려내야 한다. [DynaSLAM](https://arxiv.org/abs/1806.05620)과 [MaskFusion](https://arxiv.org/abs/1804.09194)이 이 문을 두드렸지만, 계산 비용과 현실적인 강건성을 동시에 맞추는 일은 여전히 과제로 남아 있다.

**복셀 지도의 메모리.** Voxblox와 OctoMap이 메모리 장벽을 낮추긴 했어도, 건물 한 층이나 도시 블록을 높은 해상도로 채우려면 데이터 크기가 금세 감당할 수 없이 불어난다. wavemap 같은 다해상도 압축 구조나 [Instant-NGP](https://arxiv.org/abs/2201.05989) 같은 neural implicit 표현이 대안을 모색하고 있으나, 실시간 지도 갱신과 고속 공간 조회를 동시에 만족하는 표준 해법은 아직 완성되지 않았다.

---

3부에서 살펴본 직접법과 RGB-D 조밀 재구성은 픽셀 밝기와 깊이 값을 아낌없이 투입해 풍부한 3D 표면을 빚어냈지만, 정작 KinectFusion을 비롯한 초기 dense 시스템들이 가장 먼저 부딪힌 벽은 다름 아닌 loop closure의 부재였다. ElasticFusion과 BundleFusion이 지도를 구부리고 다시 합치는 기술을 마련하긴 했어도, 그에 앞서 "지금 내가 보는 이곳이 아까 지나온 그 장소"임을 알아보는 일은 전혀 다른 문제였다. 다음 4부에서는 수작업 특징 어휘에서 딥러닝 기반 전역 표현으로 진화하며 SLAM의 '기억'을 학습으로 풀기 시작한 장소 인식(place recognition)을 시작으로, 학습 기반 SLAM의 흐름을 따라간다(→ 13장).