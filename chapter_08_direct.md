# Ch.8 — Direct 계보: DTAM에서 DSO까지

Richard Newcombe는 Andrew Davison의 박사과정 학생이었다. Imperial College에서 MonoSLAM의 30-landmark 한계를 직접 목격한 그는 2011년 정반대의 선택을 했다—모든 픽셀을 쓰기로. Davison이 "몇 개의 점만 추적하면 충분하다"는 EKF의 논리에 기대어 실시간을 증명했다면, Newcombe는 GPU 한 장을 얹고 화면 전체를 써도 실시간이 가능하다는 것을 보여줬다. DTAM은 MonoSLAM의 직계지만, 그 방법론적 DNA는 완전히 뒤집혀 있다.

Ch.7에서 살펴본 ORB-SLAM 계보는 feature를 먼저 뽑고 그 feature만 추적하는 방식이었다. Harris 코너와 ORB 디스크립터가 걸러낸 수백 개의 점—나머지 픽셀은 버려진다. Direct 계보는 이 선택을 거부했다. 버릴 픽셀이 없다—이미지 자체가 측정값이다.

같은 해 뮌헨에서는 Daniel Cremers가 다른 경로를 걷고 있었다. Computer vision의 variational 방법론(Gauss-Newton image alignment, 광학 흐름의 수식 언어)을 SLAM 전체에 이식하는 작업이었다. Cremers의 제자 Jakob Engel은 2014년 LSD-SLAM을, 2016년 DSO를 내놓았다. 두 논문은 서로 다른 밀도에서 같은 질문을 던졌다. feature를 추출하는 대신 픽셀의 밝기를 직접 비교하면 어떤 일이 생기는가.

---

## 1. 모든 픽셀: DTAM

2011년 ICCV에서 Newcombe와 공동저자 Lovegrove, Davison이 발표한 [Newcombe, Lovegrove & Davison 2011. DTAM](https://doi.org/10.1109/ICCV.2011.6126513)은 "Dense Tracking and Mapping in Real-Time"의 약자다. 이름 그대로 추적과 지도 구축 양쪽을, 모든 픽셀을 사용해, 실시간으로 수행한다.

시스템의 핵심은 두 부분이다. 추적 단계에서는 현재 프레임 전체를 cost volume과 비교하는 photometric alignment를 수행한다. 특징점 추출 없이, 디스크립터 매칭 없이, 픽셀 intensity의 차이만 최소화한다. 지도 구축 단계에서는 multi-baseline stereo 방식으로 depth map을 추정하고, total variation regularization으로 smooth한 dense 3D 모델을 유지한다.

$$E(\mathbf{u}) = \sum_{i} \rho\left( I_i\bigl(\pi(KT_i\mathbf{p}(\mathbf{u}))\bigr) - I_r\bigl(\pi(\mathbf{p}(\mathbf{u}))\bigr) \right) + \lambda \,\text{TV}(\mathbf{u})$$

여기서 $\mathbf{u}$는 역 깊이(inverse depth) 맵, $\mathbf{p}(\mathbf{u})$는 $\mathbf{u}$로 역투영한 3D 점, $K$는 카메라 내부 행렬, $T_i$는 참조 프레임 기준 $i$번 프레임의 rigid body 변환, $\pi$는 원근 투영, $\rho$는 Huber loss, $\text{TV}(\mathbf{u}) = \|\nabla \mathbf{u}\|_1$은 total variation regularizer이다. 이 최적화를 실시간으로 돌리려면 GPU가 필요하다. DTAM은 그 전제를 숨기지 않았다. 당시 Nvidia GTX 480 한 장(논문 §3의 commodity 시스템 설정)에서 실행되었다.

> 🔗 **차용.** DTAM의 dense volumetric 접근은 depth camera 기반 연구, 특히 [Curless & Levoy 1996](https://doi.org/10.1145/237170.237269)의 TSDF 아이디어에서 부분 영감을 받았으나, 단안(monocular) 카메라에 적용했다는 점이 핵심 차이다. 이후 Newcombe 자신이 주도한 [KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378)(2011, ISMAR)이 오히려 depth sensor 버전으로 이 아이디어를 완성시키는 역방향 흐름이 나타난다.

결과는 충격적이었다. 실내 scene 전체가 실시간으로 복원되는 영상은 2011년 ICCV 발표 직후 YouTube에 공개되어 수만 회 조회를 기록했다. 그러나 약점도 명확했다. GPU 없이는 돌아가지 않았고, 조명 변화에 취약했으며, 실외 대규모 환경으로는 확장되지 않았다.

<!-- DEMO: dtam_photometric_residual.html -->

---

## 2. 엣지의 추적: LSD-SLAM

[Engel, Schöps & Cremers 2014. LSD-SLAM](https://doi.org/10.1007/978-3-319-10605-2_54)은 DTAM의 dense를 포기하는 대신 GPU 의존성도 함께 버렸다. "Large-Scale Direct Monocular SLAM"은 semi-dense 방식으로, 이미지에서 gradient magnitude가 임계값 이상인 픽셀만 추적한다. 벽의 평탄한 영역은 무시하고, gradient가 충분한 엣지 근방 픽셀만 살린다. 코너 detector는 쓰지 않으며, 오직 intensity gradient의 세기가 픽셀 선택 기준이다.

추적 단계는 SE(3)에서의 direct image alignment다. 현재 프레임을 키프레임에 direct로 warping하여 photometric residual을 Gauss-Newton으로 최소화한다. 지도는 키프레임 기반이며 각 키프레임마다 semi-dense depth map을 유지한다. 키프레임 간 연결은 pose graph로 관리하고, loop closure는 appearance-based relocalization으로 후보를 찾은 뒤 depth consistency check로 검증한다.

> 🔗 **차용.** Gauss-Newton photometric registration은 이미지 정렬 분야의 고전이다. [Lucas & Kanade 1981](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf) tracker와 그 역방향 합성([Baker & Matthews 2004](https://doi.org/10.1023/B:VISI.0000011205.11775.fd))이 LSD-SLAM frontend의 직접 조상이다. Cremers 그룹은 variational image processing 커뮤니티의 언어를 SLAM 파이프라인 전체로 이식했다.

CPU에서 실시간으로 동작한다는 점이 LSD-SLAM의 실용적 의미였다. 키프레임만 들고 pose graph를 최적화하는 구조는 PTAM의 tracking/mapping 분리와 표면적으로 닮았지만, 내부는 달랐다. ORB나 BRIEF 같은 binary descriptor가 없고, 픽셀 강도가 유일한 측정값이다.

LSD-SLAM은 실외 대규모 환경에서도 동작하는 장면을 공개했다. 자전거를 타고 수십 미터를 이동하는 동안 semi-dense map이 구축되는 데모는 direct 방식의 확장 가능성을 보여줬다. KITTI 벤치마크에서 당시 top-tier feature-based 방법과 비교 가능한 수준이었다.

그러나 조명 변화가 문제였다. 터널 진입, 창문 역광, 갑작스러운 플래시—photometric consistency를 가정하는 순간, 이런 상황은 시스템을 즉시 destabilize했다.

<!-- DEMO: lsd_slam_semidense.html -->

---

## 3. Sparse Direct의 완성: DSO

[Engel, Koltun & Cremers 2018. DSO (PAMI)](https://doi.org/10.1109/TPAMI.2017.2658577)는 2016년 arXiv에 먼저 공개되었다. "Direct Sparse Odometry"는 이름이 이미 포지셔닝을 담고 있다. LSD-SLAM보다 더 sparse하게, 그러나 DTAM보다 훨씬 적은 픽셀로, 대신 photometric calibration을 철저히 하겠다.

시스템은 각 키프레임에서 gradient가 높은 픽셀 약 2,000개를 선택한다. ORB-SLAM2의 기본 설정(nFeatures=1000)에 비해 많고, LSD-SLAM의 semi-dense(gradient 있는 픽셀 전체)보다 훨씬 적다. 이 픽셀들에 대해 sliding window bundle adjustment를 수행하는데, 최적화 변수가 camera pose뿐 아니라 inverse depth, affine brightness 파라미터 $(a_i, b_i)$까지 포함한다. 윈도우를 벗어난 프레임은 marginalization으로 제거되며, 이 과정에서 Schur complement를 이용해 계산 비용을 O(N)으로 유지한다.

DSO는 카메라 photometric 모델을 세 층으로 분리한다. 첫째, vignetting(렌즈 주변부로 갈수록 밝기가 감소하는 효과)은 사전 캘리브레이션으로 보정한다. 둘째, camera response function(gamma curve, 센서가 빛을 비선형으로 기록하는 특성)도 사전에 역함수를 추정해 linear intensity domain으로 변환한다. 셋째, 프레임마다 달라지는 노출 시간과 affine brightness 변화는 실시간 최적화 변수 $(t_i, a_i, b_i)$로 추정한다:

$$E_{pj} = \sum_{\mathbf{p} \in \mathcal{N}_p} w_{\mathbf{p}} \left\| \left( I_j\!\left[\mathbf{p}'\right] - \frac{t_j e^{a_j}}{t_i e^{a_i}} I_i[\mathbf{p}] - \left(b_j - \frac{t_j e^{a_j}}{t_i e^{a_i}} b_i\right) \right) \right\|_\gamma$$

여기서 $t_i, t_j$는 노출 시간, $(a_i, b_i)$와 $(a_j, b_j)$는 각 프레임의 affine brightness 파라미터(gain과 bias), $\|\cdot\|_\gamma$는 Huber loss이다. Vignetting은 전처리 단계에서 photometric calibration으로 보정되며, 위 잔차는 보정된 intensity에 적용된다. 카메라의 노출 변화·vignetting·response curve를 별도 캘리브레이션 단계와 실시간 최적화 변수로 분리해 처리한 것은 direct SLAM에서 DSO가 처음이었다.

> 🔗 **차용.** Photometric camera calibration의 형식적 기반은 [Debevec & Malik 1997](https://doi.org/10.1145/258734.258884)의 HDR 복원 작업에서 비롯된다. 그들이 여러 장의 사진에서 camera response function을 복원하기 위해 세운 photometric 모델을 DSO는 실시간 SLAM의 최적화 변수로 가져왔다.

결과는 인상적이었다. TUM monocular dataset에서 DSO는 ORB-SLAM2를 여러 시퀀스에서 능가한다고 보고했다. 특히 feature가 희박한 환경(평탄한 벽이 많은 실내 복도)에서 DSO가 ORB-SLAM2보다 낮은 ATE를 기록했다. photometric 정보를 쓰는 것이 원론적으로 더 많은 정보를 활용한다는 주장의 경험적 근거였다.

> 📜 **예언 vs 실제.** DSO는 사전 photometric calibration을 요구했고, 그 의존성은 곧 후속 연구의 표적이 되었다. 2018년 Bergmann, Wang, Cremers의 [online photometric calibration](https://doi.org/10.1109/LRA.2017.2777002)이 한 방향이었다—캘리브레이션을 사전에 하지 않고 SLAM 실행 중 노출·response·vignetting을 동시에 추정한다. 그럼에도 end-user 관점의 배포 장벽은 2026년 기준 여전히 남아 있다. consumer 카메라에서 photometric 파라미터를 안정적으로 추출하는 과정이 완전히 자동화되지 못한 채 카메라별 사전 세팅을 요구한다. `[진행형]`

> 📜 **예언 vs 실제.** DTAM은 GPU 한 장에 의존한 실시간 dense SLAM이었고, dense 재구성의 접근성 확대는 자연스러운 다음 과제로 놓였다. 그 실현 경로는 직진이 아니었다. 순수 mono dense는 NeRF와 3DGS가 등장하는 2020년대까지 실시간 배포 가능한 형태로 나오지 않았다. 대신 Newcombe 자신이 주도한 KinectFusion이 RGB-D depth sensor를 사용해 GPU dense 재구성을 2011년에 바로 완성했다—sensor 교체로 문제를 우회한 것이다. `[기술변화]`

---

## 4. VI-DSO와 계보의 확장

2018년 von Stumberg, Usenko, Cremers는 DSO에 IMU를 결합한 [VI-DSO](https://doi.org/10.1109/ICRA.2018.8462905)를 ICRA 2018에서 발표했다. 동기는 단순했다. photometric direct method의 가장 큰 실패 모드인 조명 급변 상황에서 IMU의 관성 측정이 pose 추적을 보조할 수 있다. 또한 mono 카메라의 scale ambiguity를 IMU로 해소할 수 있다.

VI-DSO는 DSO의 windowed photometric bundle adjustment에 IMU preintegration factor를 추가한다. IMU preintegration 방식은 [Forster et al.의 2017년 논문](https://doi.org/10.1109/TRO.2016.2597321)에서 차용했다. 결과적으로 scale이 복원되고 극단적 조명 조건에서 robustness가 향상되었다.

Cremers 그룹의 후속 작업들, [Basalt](https://arxiv.org/abs/1904.06504)(2019)와 [DM-VIO](https://doi.org/10.1109/LRA.2021.3140129)(2022)도 같은 방향을 이었다. direct photometric frontend에 tightly coupled inertial backend를 붙이는 구조다. 이 계보는 feature-based VIO(VINS-Mono, OpenVINS)와 병렬로 진행되면서 각자의 생태계를 형성했다.

> 🔗 **차용.** VI-DSO의 IMU preintegration은 [Forster et al. 2017. On-Manifold Preintegration (IEEE TRO)](https://doi.org/10.1109/TRO.2016.2597321)의 manifold preintegration 공식을 그대로 사용한다. DSO의 photometric layer 위에 Forster의 inertial layer가 올라간 적층 구조다.

---

## 5. direct method의 한계

Direct method는 이론적으로 더 많은 정보를 쓴다. feature detector가 버리는 픽셀들, 즉 gradient가 낮아도 consistent한 영역을 추적에 활용한다. photometric residual은 feature descriptor의 discretization 없이 연속적인 최적화 landscape를 제공한다.

그럼에도 2026년 기준 대다수 배포 시스템은 feature-based다. 이유는 여러 층에 걸쳐 있다.

첫째, photometric calibration 의존성이다. DSO가 가정하는 vignetting 보정, response curve 보정, 노출 제어는 consumer camera에서 그냥 얻어지지 않는다. 스마트폰 카메라는 HDR 합성, auto-exposure, 실시간 화이트 밸런스를 자체적으로 적용하며, 그 파이프라인은 사용자에게 공개되지 않는다. DSO의 photometric 모델은 이런 카메라에서 기본 가정이 깨진다.

둘째, 조명 변화다. 자동 노출, 역광, 플리커처럼 프레임 간 밝기가 급변하는 상황에서는 direct 방식의 핵심 가정인 photometric consistency가 바로 깨진다. DSO의 affine brightness 모델은 이러한 완만한 변동만 흡수할 수 있어, 실외에서 구름이 지나가거나 실내에서 형광등이 깜빡이는 장면은 여전히 추적 실패의 주 원인으로 남았다.

셋째, DSO가 controlled dataset에서 ORB-SLAM2를 이기는 시퀀스가 있어도, 실제 로봇에 얹는 엔지니어는 ORB-SLAM 쪽을 골랐다. ORB-SLAM은 여러 카메라 모델에서 별도 photometric calibration 없이 동작한다. 카메라를 교체해도 바로 돌아간다. DSO는 카메라마다 vignetting·response curve를 따로 캘리브레이션해야 했다.

넷째, [SuperPoint](https://arxiv.org/abs/1712.07629)(2018)·[LightGlue](https://arxiv.org/abs/2306.13643)(2023) 같은 학습 기반 feature가 "feature는 정보를 버린다"는 direct method의 핵심 비판을 약화시켰다. 기존 handcrafted descriptor보다 훨씬 많은 정보를 보존하면서도 descriptor matching의 실용적 장점을 유지한다. direct method가 feature-based를 공격하던 그 지점에, learned feature가 자리를 메운 것이다.

<!-- DEMO: photometric_calibration_demo.html -->

---

## 🧭 아직 열린 것

**조명 급변 환경에서의 direct tracking.** Direct method의 근본 전제, 장면의 밝기 분포가 프레임 간 보존된다는 가정은 자동 노출 카메라, 강한 역광, 터널-야외 전환 상황에서 즉각 붕괴한다. VI-DSO의 IMU 보조가 부분적으로 완화하지만, 조명 모델 자체를 동적으로 추정하는 완전한 해법은 아직 없다. 학습 기반 photometric 보정이 대안으로 탐색 중이지만, 실시간 배포 가능한 형태로 나오지 않았다.

**Textureless + direct의 이중 약점.** Feature-based는 코너가 없는 벽 앞에서 실패한다. Direct는 gradient가 없는 면에서 residual이 사라진다. 두 방식 모두 실내 복도, 대규모 창고, 균질한 실외 지형 같은 환경에서 약하다. Semi-dense LSD-SLAM은 gradient 있는 픽셀을 선택적으로 쓰는 방식으로 절충했지만, 그 픽셀이 충분히 분포하지 않는 상황의 degeneracy는 해결하지 못했다.

**Learned photometric model로의 이행 가능성.** 현재 direct SLAM의 photometric 모델은 단순 affine brightness 보정이나 고정 camera response function으로 표현된다. Neural radiance field 계열의 연구들은 장면의 appearance를 neural network로 모델링하는 방식을 탐색하고 있다. 이것이 실시간 direct SLAM의 photometric layer로 들어올 수 있는지, 들어온다면 direct와 learned의 경계가 어디에 그어지는지는 2026년 현재 열린 질문이다.

한편 direct 계보와 나란히, 다른 방향의 탈출구가 이미 2011년에 열려 있었다. Newcombe 자신이 KinectFusion을 통해 보여줬다. 단안 카메라의 photometric 가정을 지키는 대신, 센서 자체를 바꾸면 된다. depth 정보를 직접 측정하는 RGB-D 카메라는 밝기 변화에 무관하게 dense 재구성을 가능하게 했다. direct method가 photometric consistency를 수식으로 지키려 했다면, RGB-D는 그 가정 자체를 질문 목록에서 지웠다.
