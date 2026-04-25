# Ch.11 — 깊이 추정의 부활: Eigen에서 Depth Anything까지

3부(Ch.7-10)에서 feature-based, direct, RGB-D, place recognition 계통은 각자의 방식으로 성숙 단계에 올랐다. 기하학이 전부였다. ORB-SLAM은 epipolar 기하로 세계를 재구성했고, DSO는 photometric consistency로, KinectFusion은 ICP로 표면을 쌓아올렸다. 학습이 끼어들 자리는 없었다. 혹은 그렇다고 여겼다. 4부는 그 경계가 무너지는 이야기인데, 균열은 엉뚱한 곳에서 왔다. SLAM 연구자가 아니라 컴퓨터 비전 쪽, 더 구체적으로는 NYU의 한 대학원생이 낸 논문 한 편에서.

Monocular depth 추정은 컴퓨터 비전에서 가장 오래된 ill-posed 문제 중 하나였다. 한 장의 이미지에서 깊이를 복원한다는 것은 원론적으로 불가능하다. 카메라는 3D 세계를 2D로 투영하면서 깊이 정보를 버리기 때문이다. 그러나 인간은 단안으로도 깊이를 판단한다. 원근감, 폐색, 텍스처 기울기, 표면의 음영. 이것들을 통계적으로 학습할 수 있다면? 2014년, NYU의 David Eigen은 이 질문에 CNN을 들이밀었다. 그 실험 하나가 10년 뒤 SLAM 파이프라인을 다시 쓰게 될 계보의 출발점이었다.

---

## 1. Eigen 2014 — 첫 CNN depth

2014년 이전에도 monocular depth 추정 연구는 있었다. Ashutosh Saxena(Make3D, Stanford)가 [2005년 SVM과 Markov Random Field를 결합해 단일 이미지에서 depth map을 예측하는 시스템](https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images)을 발표했다. 결과는 거칠었고 실내 구조화 환경에서만 겨우 작동했다. 

Eigen, Puhrsch, Fergus의 [Eigen et al. 2014](https://arxiv.org/abs/1406.2283)는 접근 자체를 바꿨다. coarse network가 전역 구조를 예측하고, fine network가 지역 세부를 보정하는 두 단계 CNN. 학습 데이터는 NYU Depth v2 — Kinect RGB-D 카메라로 수집된 실내 장면 120,000장. 수치는 당시 기준 Make3D보다 개선되었으나 더 중요한 것은 개념의 증명이었다. 깊이는 학습 가능하다.

그러나 결정적 약점이 하나 남았다. **scale ambiguity**다. 네트워크는 상대적인 깊이 구조를 배우지만, 절대 스케일은 학습 데이터의 분포에 묶여 있다. NYU 실내에서 학습한 모델을 야외에 들이대면 스케일이 틀린다. 이 한계는 2024년까지 분야 전체의 숙제로 남는다.

> 🔗 **차용.** Eigen 2014는 깊이 추정 task 자체를 Make3D(Saxena 2005)에서 물려받았다. SVM과 MRF를 CNN으로 교체한 것이 핵심 교체였고, task 정의와 평가 지표(RMSE, threshold accuracy)는 이어받았다.

---

## 2. Garg → Godard — self-supervised depth

supervised depth 학습의 병목은 데이터였다. Kinect는 실내에서 잘 작동하지만 야외 환경, 특히 일조 하에서는 적외선 패턴이 날아간다. 대규모 실외 RGB-D 데이터셋 구축은 비용이 크다.

2016년, [Ravi Garg(UCL)는 다른 길을 열었다](https://arxiv.org/abs/1603.04992). stereo 이미지 쌍을 학습 신호로 쓰는 것이다. left 이미지를 보고 depth를 예측한 뒤, 그 depth와 카메라 baseline을 이용해 right 이미지를 reconstruction한다. right 이미지는 이미 존재하므로 photometric loss로 supervision이 가능하다. 라벨이 필요 없다.

Clément Godard(UCL)는 2017년 이 아이디어를 [Godard et al. 2017](https://doi.org/10.1109/CVPR.2017.699)에서 **MonoDepth**로 체계화했다. left-right consistency: left로 예측한 depth와 right로 예측한 depth가 서로 일치해야 한다는 양방향 제약. 구조적 유사도(SSIM)를 photometric loss에 포함해 텍스처 없는 영역에서의 안정성을 높였다. 핵심은 학습 시에만 stereo 쌍이 필요하다는 것이다. 추론은 단일 이미지만으로 가능하다. KITTI 벤치마크에서 당시 self-supervised 방법 중 최고였다.

> 🔗 **차용.** Garg와 Godard의 photometric loss는 stereo matching 문헌에서 온다. [Scharstein과 Szeliski가 정리한(2002)](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf) disparity estimation의 intensity consistency 제약을 depth network의 학습 신호로 전용한 것이다.

2019년 Godard의 *MonoDepth2* ([Godard et al. 2019, ICCV](https://arxiv.org/abs/1806.01260))는 stereo 쌍 대신 monocular video를 쓰는 self-supervised로 나아갔다. depth network와 pose network를 동시에 학습한다. 연속 프레임 사이의 카메라 운동을 pose network가 예측하면, depth network의 출력으로 이전 프레임을 현재로 warping한다. warping 오차가 줄어드는 방향으로 두 네트워크가 함께 최적화된다. 두 가지 핵심 장치가 추가됐다. 첫째, **minimum reprojection loss**: 여러 소스 프레임 중 photometric error가 가장 낮은 것을 선택해 occluded 영역 오류를 줄인다. 둘째, **auto-masking**: 카메라와 같은 속도로 움직이는 픽셀(정지 카메라 + 정지 물체 포함)을 자동으로 제외한다.

깔끔한 구조였다. 그러나 여전히 문제가 있었다. 움직이는 물체와 반사 표면이 걸렸고, 하늘처럼 텍스처가 없는 영역에서는 더 심했다. 이 영역에서 photometric consistency 가정이 무너진다. 그리고 스케일은 여전히 모호하다. video supervision은 스케일을 프레임 간 상대적으로만 풀어준다.

---

## 3. MiDaS — 데이터셋 혼합

Intel의 René Ranftl이 이끈 팀이 2020년 발표한 [Ranftl et al. 2020](https://doi.org/10.1109/TPAMI.2020.3019967) **MiDaS**(Mixing Datasets for Zero-shot Cross-dataset Transfer)는 다른 질문을 던졌다. 한 데이터셋이 아니라 여러 데이터셋을 동시에 학습하면 어떻게 될까?

문제는 데이터셋마다 depth의 단위와 스케일이 다르다는 것이다. NYU는 미터 단위 실내, KITTI는 LiDAR 포인트 실외, ReDWeb은 stereo 영화, MegaDepth는 SfM 재구성. 이것들을 그대로 섞으면 네트워크가 혼란스러워진다.

Ranftl의 해법은 **affine-invariant loss**였다. 각 이미지의 depth prediction을 학습 전에 affine transformation(스케일 + 시프트)으로 정규화한다. 구체적으로, 예측과 정답 각각에서 중앙값을 빼 shift를 제거하고, 중앙값 절대편차(MAD)로 나눠 scale을 제거한 뒤 비교한다. 이 scale-and-shift invariant 정규화 덕분에 데이터셋 간 단위 불일치가 사라진다. 이렇게 하면 네트워크는 "얼마나 멀리"가 아니라 "상대적으로 어느 것이 더 멀리"를 배운다.

12개 데이터셋, 190만 장 이상의 이미지로 학습한 MiDaS는 처음으로 실용적인 cross-dataset generalization을 보여줬다. 야외, 실내, 역사 사진, 영화 프레임 모두에서 그럴듯한 relative depth를 내놨다. 절대 스케일은 없지만, 깊이 순서와 구조는 맞았다.

이후 Ranftl 팀은 2021년 [**DPT**(Dense Prediction Transformer)](https://arxiv.org/abs/2103.13413)를 별도 발표해 MiDaS backbone을 ViT 기반으로 교체했다. MiDaS v3부터 DPT가 기본 backbone이 됐고, v3.1(2022)은 그 개선판이었다. 성능이 크게 올랐다.

> 🔗 **차용.** MiDaS v3와 이후 Depth Anything은 CLIP·DINOv2·ViT 계열 backbone을 그대로 전용했다. backbone 교체만으로 성능이 점프하는 현상은 foundation model 시대의 일반적 패턴이지만, depth estimation에서 그 효과가 처음 대규모로 확인된 것은 DPT(Ranftl 2021)에서였다.

---

## 4. Depth Anything — foundation 규모

2024년 1월, TikTok Research의 Lihe Yang 팀이 발표한 [Yang et al. 2024](https://arxiv.org/abs/2401.10891) **Depth Anything**은 규모로 문제를 풀었다. 1.5M개의 labeled 이미지(기존 데이터셋 통합)와 62M개의 unlabeled 이미지를 썼고, unlabeled 이미지에는 pseudo-label을 생성해 학습에 포함했다. pseudo-label 품질을 높이기 위해 semantic segmentation feature를 auxiliary supervision으로 썼다.

결과는 MiDaS를 포함한 이전 방법들을 KITTI, NYU, ScanNet, DIODE 등 모든 주요 벤치마크에서 앞질렀다. 모델 크기는 ViT-L 기반 335M 파라미터. 추론 속도는 실시간과 거리가 있었으나, 품질이 먼저였다.

같은 해 나온 [**Depth Anything v2**](https://arxiv.org/abs/2406.09414)는 합성 데이터(Unreal Engine 기반 Virtual KITTI, Hypersim 등)를 대거 추가했다. 합성 데이터는 반사·투명 표면처럼 실제 데이터에서 annotation이 어려운 영역을 커버한다. v2는 v1보다 edge 세부와 얇은 구조 표현에서 눈에 띄게 개선되었다.

그러나 Depth Anything도 여전히 relative depth로, scale은 없다.

[**ZoeDepth**(Shariq Farooq Bhat et al. 2023)](https://arxiv.org/abs/2302.12288)와 [**Metric3D v2**(2024)](https://arxiv.org/abs/2404.15506)는 이 마지막 문제를 다른 방향에서 공략했다. camera intrinsic(초점거리·센서 크기)을 네트워크 입력으로 명시적으로 주입한다. 같은 장면이라도 초점거리가 다르면 depth 분포가 달라지는 것을 네트워크가 배우도록 한다. in-the-wild 데이터에서의 metric depth 결과는 이전과 질적으로 달랐다. 완벽하지는 않지만 많은 실용 시나리오에서 쓸 수 있는 수준이 됐다.

---

## 5. SLAM으로의 역수입

2021년쯤부터 SLAM 연구자들이 monocular depth 모델을 파이프라인 안으로 끌어들이기 시작했다. 진입로는 초기화였다. Monocular SLAM은 구조상 초기화가 까다롭다. 두 프레임에서 triangulation을 하려면 baseline이 충분해야 하고, scale은 첫 단계부터 모호하다.

depth prior를 첫 프레임에 주입하면 초기화가 빨라지고 scale을 대략 고정할 수 있다. [Teed와 Deng이 2021년 발표한 DROID-SLAM](https://arxiv.org/abs/2108.10869)은 recurrent optical flow와 BA를 묶은 구조인데, 이 계통에서 나온 후속 연구들이 monocular depth prior를 geometric initialization에 붙이는 방식을 실험했다.

scale recovery 쪽은 더 직접적이었다. monocular visual odometry(VO)는 달리면서 scale drift가 쌓인다. depth network 예측을 주기적인 scale anchor로 쓰면 이 drift를 억제할 수 있다. 완벽한 해법이 아니라 실용적 패치지만, 순수 VO보다 훨씬 긴 거리에서 버텼다.

> 📜 **예언 vs 실제.** Eigen은 2014년 논문에서 surface normal 등 3D geometry 정보와의 결합을 자연스러운 확장 방향으로 언급했다. joint multi-task learning은 이후 PAD-Net·VPD 등으로 부분 실현됐다. 그러나 2024년 시점 실질적 영향은 task를 합친 것보다 ViT backbone 공유로 왔다고 볼 여지가 크다. 예측한 방향과 실제 경로는 달랐다. `[기술변화]`

> 📜 **예언 vs 실제.** MiDaS(2020)는 scale-and-shift invariant loss로 절대 스케일을 포기하고 상대 깊이에만 집중하는 우회를 택했고, 이는 metric 복원이 카메라 파라미터 없이는 본질적으로 어렵다는 인식과 맞닿아 있다. 2024년 Depth Anything v2와 Metric3D v2가 camera intrinsic을 입력으로 받는 방식으로 이 방향을 직접 공략했고, in-the-wild metric이 실용 수준에 가까워졌으나 완전한 카메라 독립은 아직 아니다. `[진행형]`

---

## 🧭 아직 열린 것

**반사·투명 표면의 depth.** 유리, 물, 금속 반사면은 카메라가 포착하는 것이 실제 표면이 아니다. 물리 광학 수준의 문제다. 합성 데이터로 학습을 늘려도 real-world 반사 장면에서의 일반화는 여전히 불안정하다. [ClearGrasp(Sajjan et al. 2020)](https://arxiv.org/abs/1910.02550) 같은 specialized 접근이 있으나 general solution은 없다. Foundation 규모 모델에서도 이 영역의 오차는 구조적으로 크다.

**Dynamic scene에서 ego-depth와 object-depth의 분리.** 자동차·사람·자전거가 움직이는 장면에서 photometric consistency는 근본적으로 위반된다. self-supervised 방법들은 moving object를 masking해 우회하는데, 이는 문제를 푸는 것이 아니라 피하는 것이다. 움직이는 물체의 depth를 에고모션과 분리해 동시에 풀어야 하는 문제는 [Ranjan et al.(2019)](https://arxiv.org/abs/1805.09806)을 비롯한 여러 후속 연구가 시도했으나 실용 수준에서는 여전히 난제다.

**Metric scale의 일반화.** Metric3D v2와 Depth Anything v2가 camera intrinsic 조건부로 metric depth를 내놓기 시작했다. 그러나 intrinsic을 모르는 상황은 흔하다. 스마트폰 수백 종이 있고, CCTV와 역사 아카이브 사진에는 exif조차 없다. 카메라 독립적 metric depth는 foundation model 규모에서도 쉽지 않다. 이것이 2025년 시점 monocular depth의 남은 핵심 질문이다.

---

2024년, Depth Anything이 벤치마크를 갈아엎고 있던 같은 시기, Cambridge의 한 논문이 이미 9년째 SLAM 커뮤니티의 미완성 숙제로 남아 있었다. 한 장의 이미지에서 절대 pose를 바로 꺼낸다. 특징 추출도 최적화도 없이. 지도는 처음부터 존재하지 않는다. [PoseNet](https://arxiv.org/abs/1505.07427)이 그 꿈의 이름이었다.
