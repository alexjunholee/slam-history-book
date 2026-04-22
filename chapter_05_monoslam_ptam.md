# Ch.5 — MonoSLAM → PTAM: 실시간의 몽상과 분리 혁명

앞 챕터는 EKF-SLAM이 어떻게 확률론적으로 일관된 지도 구축 방법을 완성했는지, 그리고 그 공분산 행렬이 landmark 수 $N$에 대해 $O(N^2)$로 커지는 구조적 벽에 부딪혔는지를 보였다. 그 벽은 이론적 결함이 아니었다. 설계 선택이었다. 그 선택을 다른 방향에서 다시 시작한 사람들이 Davison과 Klein이었다.

2003년 Andrew Davison은 Imperial College London의 연구실에서 웹캠 한 대와 노트북으로 무언가를 시도하고 있었다. Smith와 Cheeseman이 1988년에 확률적 공간관계의 수학을 세웠고, Leonard와 Durrant-Whyte가 그 위에 EKF-SLAM의 틀을 쌓았다. Davison은 그 유산을 물려받았다. 그러나 그가 추가한 것은 카메라 하나였다. IMU도 없고, 스테레오 쌍도 없고, 레이저도 없었다. 단안 카메라만으로 실시간 3D 추적을 하겠다는 것은 당시 기준으로 무모했다. 선배들의 EKF 수학을 기반으로, Shi-Tomasi(1994)의 코너 검출기를 시각 특징으로, Kalman의 예측-갱신 루프를 실시간 추적기로 — 이것이 MonoSLAM의 계보였다.

그리고 4년 뒤, Oxford의 Georg Klein과 David Murray는 같은 해(2007)에 Davison의 정식 논문과 나란히 다른 논문을 발표했다. 아키텍처 자체를 분리했다. tracking과 mapping을 두 스레드로 쪼갰다. 그 단순한 결정이 이후 10년간 Visual SLAM의 표준 구조를 결정했다.

---

## 1. 2003년의 데모

2003년 ICCV에서 Davison이 공개한 [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654)는 분야를 뒤흔들었다. 결과가 놀라웠기 때문이 아니다. *그것이 가능하다는 것*이 충격이었다.

당시 SLAM 분야의 주류는 레이저 센서였다. LiDAR는 2D 거리를 직접 제공했고, 스테레오 카메라는 픽셀 수준에서 깊이를 복원했다. 단안 카메라는 깊이 정보 자체가 없었다. 단안으로 3D 구조를 추정하려면 최소 두 프레임이 필요했고, 초기 깊이 추정의 불확실성이 EKF 상태 벡터 전체로 전파되었다. 이론적으로 가능했지만 실시간으로 돌린다는 것은 별개의 문제였다.

Davison의 선택은 실용적 제약에서 왔다. IMU는 추가 하드웨어였고, 스테레오는 캘리브레이션 부담이 있었다. 그가 원한 것은 "카메라 하나로 증명하는 것"이었다. 증명에 성공하면 나머지는 추가할 수 있었다. 그 논리는 맞았다. 틀린 것은 EKF가 그 "나머지"를 실제로 수용할 수 있는 구조인지였다.

---

## 2. EKF의 아름다움과 벽

2007년 IEEE PAMI에 실린 [MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049)는 Davison, Ian Reid, Nicholas Molton, Olivier Stasse의 공동 저자로, ICCV 2003 데모의 완성된 논문 형태였다.

MonoSLAM의 상태 벡터는 Smith-Cheeseman(1988)과 Leonard-Durrant-Whyte(1991)의 정식(Ch.4)을 단안 카메라에 직접 이식했다. 카메라 상태 $\mathbf{x}_v \in \mathbb{R}^{13}$ — 위치 3, 사원수 방향 4, 속도 3, 각속도 3 — 과 landmark 집합 $\mathbf{y}_i \in \mathbb{R}^3$를 하나의 벡터 $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$에 담고, 그 전체 공분산 $(13+3N)\times(13+3N)$ 행렬 $\mathbf{P}$를 매 프레임 predict-update 루프로 유지했다. predict 단계에서는 카메라 운동 모델 $f$의 자코비안 $\mathbf{F}$로 공분산을 전파했고($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$), update 단계에서는 투영 함수의 자코비안 $\mathbf{H}_i$로 칼만 이득을 계산해 상태와 공분산을 갱신했다. EKF predict-update 수식 자체는 Ch.4 §4.3의 것과 동일하다. 달라진 것은 상태 벡터 안에 카메라 속도·각속도가 함께 들어간 점이었다(이동 물체인 카메라의 동역학 모델이 필요했기 때문이다).

공분산 갱신 $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$의 지배 비용은 $(13+3N)^2$ 행렬 곱셈으로, landmark 수 $N$에 대해 $O(N^2)$였다. 30개의 landmark는 실시간(15 fps)에서 간신히 돌아갔다. 100개면 이미 느렸다.

> 🔗 **차용.** MonoSLAM의 EKF 상태 벡터 구조는 Smith-Cheeseman-Durrant-Whyte(1988-1991)의 확률적 공간관계 표현을 단안 카메라에 직접 이식한 것이다. Kalman 필터 자체는 1960년부터 있었지만, 로봇 pose와 landmark를 같은 벡터에 넣는 "augmented state vector" 관행이 확립된 것은 Leonard-Durrant-Whyte 1991의 스타일이었다.

30이라는 숫자는 시스템 한계를 드러냈다. Davison은 이를 알고 있었다. 논문 §6의 Future Work에는 "계층적 맵으로의 확장"이 명시되어 있었다. 그러나 EKF 내부에서 계층적 구조를 만드는 것은 근본적으로 어려웠다. 공분산 행렬이 모든 landmark 간의 상관관계를 전제로 설계되어 있었기 때문이다.

Shi-Tomasi(1994) 코너가 MonoSLAM의 시각 특징으로 선택된 것도 이 맥락에서 읽힌다. "Good Features to Track"의 선택 기준은 추적하기 좋은 점을 고르는 것이었다. 애초에 추적이 실패할 가능성이 낮은 코너만 상태 벡터에 넣으면 EKF의 갱신이 더 안정적이었다. 30개의 특징점이 모두 잘 추적되는 한, EKF는 돌아갔다.

> 🔗 **차용.** PTAM이 아니라 MonoSLAM에서 이미 Shi-Tomasi 1994의 코너 검출기가 쓰였다. "좋은 특징을 선택해서 추적한다"는 설계 철학은 Shi-Tomasi → MonoSLAM → PTAM의 직접적인 계보다.

---

## 3. 2007년, 같은 해

30이라는 landmark 상한은 숫자가 아니라 EKF의 천장이 드러난 지점이었다. 그것을 보고 있던 사람이 Oxford에 있었다.

2007년은 이상했다. 같은 해에 두 논문이 나왔다. Davison의 MonoSLAM PAMI 논문과, Klein & Murray의 [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852)가 모두 2007년에 발표되었다.

PTAM을 발표한 Georg Klein은 당시 David Murray 그룹의 Oxford 박사과정 학생이었다. Murray 그룹은 Oxford Active Vision Laboratory의 계보를 이었다. Davison 역시 Murray 그룹 출신이었다 — Davison의 박사과정 지도교수가 Murray였다. Klein이 Davison의 MonoSLAM을 알고 있었던 것은 당연했다. 그리고 Klein이 본 것은 MonoSLAM의 EKF가 아니라, MonoSLAM이 증명한 가능성이었다.

단안 카메라로 실시간 3D 추적을 할 수 있다. 그것은 확실했다. 문제는 *어떻게*였다. Klein의 답은 아키텍처를 바꾸는 것이었다.

---

## 4. 분리

PTAM의 핵심 아이디어는 하나였다. Tracking(카메라 pose 추적)과 Mapping(3D 지도 구축)을 분리해서 두 개의 병렬 스레드로 실행한다.

EKF에서 이 둘은 같은 루프 안에 있었다. 매 프레임마다 예측-갱신 한 사이클을 돌렸다. 카메라가 움직이면 상태가 예측되고, 이미지에서 landmark를 찾으면 상태가 갱신되었다. Tracking과 Mapping은 같은 연산 안에서 뒤섞여 있었다.

PTAM은 이것을 풀었다. Tracking 스레드는 매 프레임 카메라 pose를 추정하는 일만 한다. 현재 keyframe 집합에서 보이는 3D 점들의 2D 투영과 실제 관측을 매칭해서 pose를 계산한다. 빠르게, 실시간으로. Mapping 스레드는 새 keyframe이 추가될 때마다 bundle adjustment를 실행한다. 느려도 된다. Tracking 스레드가 독립적으로 돌아가기 때문이다.

Mapping 스레드의 bundle adjustment는 keyframe 집합 $\mathcal{K}$와 3D 점 집합 $\mathcal{P}$에 대해 재투영 오차의 합을 최소화했다:
$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$
여기서 $\mathbf{T}_k \in SE(3)$는 keyframe $k$의 pose, $\mathbf{p}_j \in \mathbb{R}^3$는 3D 점, $\pi$는 카메라 투영 함수, $\mathbf{z}_{kj}$는 keyframe $k$에서 점 $j$의 관측 픽셀 좌표, $\mathbf{\Sigma}_{kj}$는 측정 공분산, $\rho$는 Huber 함수 등의 robust kernel이다. 이 최적화는 Levenberg–Marquardt로 반복적으로 풀리며, Mapping 스레드가 비동기로 실행하기 때문에 Tracking 스레드의 실시간성에 영향을 주지 않았다.

> 🔗 **차용.** PTAM의 Mapping 스레드에서 실행되는 bundle adjustment는 Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"의 직접 적용이다. 1부에서 다룬 사진측량의 100년 전통이 SLAM backend에 처음으로 제대로 자리를 잡은 지점이 여기다. EKF에서는 공분산 행렬의 크기 제약 때문에 전체 BA가 불가능했다. 스레드 분리로 그 제약이 사라졌다.

이 분리는 단순해 보이지만 결과는 달랐다. Mapping 스레드가 비동기로 bundle adjustment를 실행하기 때문에, 지도에 들어갈 수 있는 landmark 수가 EKF의 $O(N^2)$ 제약을 벗어났다. PTAM이 사용한 keyframe의 수는 수백 개였다. 각 keyframe에는 수백 개의 patch feature가 있었다. MonoSLAM의 30 landmark와는 다른 세계였다.

초기 맵 구축 방법도 달랐다. PTAM은 사용자가 카메라를 천천히 움직이는 초기화 단계에서 Nistér 2004의 5-point 알고리즘으로 essential matrix를 추정하고, 첫 keyframe 쌍에서 초기 3D 구조를 복원했다. 이것 역시 차용이었다.

Essential matrix $\mathbf{E}$는 두 카메라 좌표계 사이의 순수 기하관계를 담는 $3\times 3$ 행렬로, 대응점 쌍 $(\mathbf{p}, \mathbf{p}')$에 대해 ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$을 만족한다. $\mathbf{E}$는 내부적으로 $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$ ($\mathbf{t}_\times$는 병진의 반대칭 행렬, $\mathbf{R}$은 회전)으로 분해되므로 자유도가 5이다. 따라서 최소 5쌍의 대응점으로 유일해(최대 10개 실수 해)를 구할 수 있다. Nistér의 기여는 이 5-point 연립방정식을 Gröbner basis를 이용해 효율적으로 풀어 RANSAC 루프 안에서 실시간으로 돌릴 수 있게 한 것이다. PTAM은 이 solver를 초기화 단계에서 RANSAC과 함께 사용해 첫 두 keyframe 사이의 상대 pose를 추정하고 초기 3D 점군을 삼각측량으로 복원했다.

> 🔗 **차용.** PTAM의 5-point essential matrix 초기화는 David Nistér 2004 "An Efficient Solution to the Five-Point Relative Pose Problem"에서 직접 가져왔다. Nistér의 5-point solver는 단안 카메라의 초기 맵 구축에 필요한 최소 대응쌍을 사용하는 minimal solver였다.

> 🔗 **차용.** PTAM의 keyframe 구조는 Leonard-Durrant-Whyte의 submap 아이디어에서 맥이 닿는다. "전체 맵을 한 번에 최적화하기 어려우면 지역 단위로 나눈다"는 발상이 PTAM에서는 keyframe 집합으로 표현되었다. 후속 ORB-SLAM의 covisibility graph는 이 keyframe 관리를 더 정교하게 만든 버전이다.

---

## 5. 새 아키텍처의 확산

PTAM은 AR(증강현실) 워크스페이스를 대상으로 설계되었다. 논문 제목에도 "Small AR Workspaces"가 명시되어 있다. Tracking 스레드의 재현성이 좋았고, 실시간성이 확실했기 때문에 AR 응용에 바로 쓸 수 있었다.

상업적 흡수는 빠르게 일어났다. 2010년대 초 Metaio(독일 AR 스타트업, 2015년 Apple에 인수)와 Qualcomm의 Vuforia SDK는 PTAM과 유사한 tracking/mapping 분리 구조를 채용했다. 소비자 기기에서 최초로 안정적인 planar AR을 구현한 기반이 PTAM의 아키텍처였다.

학계에서의 영향은 더 직접적이었다. 2015년 Raul Mur-Artal, J.M.M. Montiel, Juan D. Tardós가 발표한 ORB-SLAM은 PTAM의 구조를 계승했다. 특징점은 patch에서 ORB 디스크립터로 바꾸고, keyframe 관리는 covisibility graph로 정교화했으며, loop closure를 새로 얹었다. PTAM이 없었으면 ORB-SLAM의 설계도가 달랐을 것이다.

2018년 Qin, Li, Shen의 VINS-Mono 역시 sliding window 최적화 + loop closure의 이중 스레드 구조를 갖는다. tracking/mapping 분리의 계보가 VIO로 확장된 사례다.

---

## 6. Davison vs Klein & Murray: 같은 곳을 바라봤나

2007년에 두 논문이 나왔다. MonoSLAM PAMI는 2003년 데모의 완성판이었다. PTAM은 같은 해에 MonoSLAM의 한계를 돌파하는 새 구조로 나왔다.

Davison이 EKF를 버리지 못한 이유는 확률론적 일관성(consistency) 때문이었다. EKF는 상태의 불확실성을 공분산 행렬로 명시적으로 관리했다. 지도의 각 landmark가 얼마나 불확실한지, landmark 간의 공분산이 어떻게 되는지가 수학적으로 추적되었다. Davison의 입장에서 bundle adjustment는 최소자승 최적화였고, 이 불확실성 표현을 포기하는 대가로 확장성을 얻는 것이었다.

Klein & Murray는 그 대가를 기꺼이 치렀다. AR 응용에서 중요한 것은 카메라 pose의 실시간 추적이었다. 지도의 불확실성을 센티미터 단위로 추적할 필요는 없었다. Bundle adjustment로 지도를 주기적으로 refine하면 충분했다.

두 접근의 철학적 차이는 SLAM 분야가 이후 선택한 방향에서 결론이 났다. 2010년대 이후 graph-based 최적화와 bundle adjustment가 주류가 되었다. EKF-SLAM은 계산 자원이 극도로 제한된 응용 외에서는 대부분 물러났다.

---

## 📜 예언 vs 실제

> **Davison 2007 PAMI §6 "Future Work"**: "larger environments를 다루기 위한 hierarchical map 구조; IMU 통합을 통한 더 빠른 동작 추정; 동적 물체가 있는 장면에서의 강건성이 다음 과제"라고 적었다.
>
> 세 가지 예측의 운명은 달랐다. Hierarchical map 아이디어는 PTAM의 keyframe 구조와 ORB-SLAM의 covisibility graph를 거쳐 부분적으로 흡수되었다. 그러나 EKF를 유지하면서 계층적 확장을 달성한 시스템은 나오지 않았다 — 계층화는 BA 기반 아키텍처 전환과 함께 왔다. IMU 통합은 2010년대 Visual-Inertial Odometry(VIO) 연구 붐으로 크게 이행했다. Davison 자신도 이후 VIO 방향 연구를 계속했다. 동적 장면 강건성은 2026년 기준 여전히 열려 있다. DynaSLAM, FlowSLAM 등 여러 시도가 있었지만 "기본 파이프라인에 포함된 해법"은 아직 없다. `[진행형]`

> **Klein & Murray 2007 §8**: "map size를 확장하려면 keyframe selection이 더 정교해야 하며, 이를 위해 appearance-based loop closure가 필요하다"고 적었다.
>
> 이 예측은 정확히 맞았다. 2012년 Mur-Artal 그룹의 전신 작업들과 2015년 ORB-SLAM이 DBoW2 기반 appearance-based loop closure와 covisibility graph 기반 keyframe 관리를 조합해서 PTAM이 명시한 확장 방향을 실현했다. 단, 도구는 PTAM의 patch feature가 아닌 ORB 디스크립터였다. 경로는 달랐지만 목적지는 맞았다. `[적중]`

---

## 🧭 아직 열린 것

**Monocular scale 복원.** MonoSLAM부터 PTAM까지, 단안 카메라 시스템은 모두 scale ambiguity를 안고 있다. 이미지 한 장에서 절대 거리를 알 수 없다는 것은 기하학적 사실이다. IMU를 추가하면 중력 방향과 가속도계 판독값으로 scale이 observability를 갖는다. 그러나 IMU 없는 순수 monocular 시스템에서 scale 복원은 2026년에도 근본적으로 해결되지 않았다. 학습 기반 monocular depth estimation(MiDaS, Depth Anything)이 단일 이미지에서 상대적 깊이를 추정하지만, 이것을 metric scale로 변환하려면 여전히 외부 참조(지면 가정, 사전 알려진 물체 크기 등)가 필요하다.

**단일 VO 시스템의 환경 범용성.** MonoSLAM은 실내 데스크탑 환경만 다루었다. PTAM은 "Small AR Workspaces"라고 스스로 범위를 제한했다. 이후 ORB-SLAM2가 실내·실외·RGB-D를 아우르려 했지만, 조명 변화가 극단적인 환경이나 low-texture 공간에서는 여전히 tracking failure가 발생한다. 단일 파이프라인이 실내 복도, 야외 도심, 야간 환경, 텍스처 없는 흰 벽 전부를 견고하게 처리하는 시스템은 2026년 기준 아직 없다. Multi-modal fusion(카메라 + LiDAR + IMU)이 일부 커버하지만, 카메라 단독 시스템의 범용성은 여전히 미결이다.

**저조도·동적 환경에서의 특징점 추적.** MonoSLAM이 요구했던 것은 충분한 조명과 정적인 장면이었다. 2007년의 PTAM도 마찬가지였다. 2026년 현재 이 두 가정은 여전히 대부분의 feature-based SLAM 시스템에서 암묵적으로 유지된다. 저조도에서 ORB feature는 검출 자체가 실패하고, 움직이는 사람이 많은 장면에서는 dynamic point가 static point로 잘못 분류된다. 이 문제를 학습 기반 optical flow나 semantic segmentation으로 우회하는 시도가 있지만, 실시간 범용 해법으로 자리잡은 시스템은 아직 없다.

---

PTAM이 확립한 tracking/mapping 분리는 한 가지를 해결하지 못했다. keyframe이 쌓일수록 누적 오차가 loop에서 폭발했다. 그 답은 PTAM과 같은 해에 나온 것이 아니었다. 1997년, CMU 지하 복도에서 Feng Lu와 Evangelos Milios가 레이저 스캔 문제를 붙들고 있던 바로 그 시점에 이미 형태를 갖추고 있었다.
