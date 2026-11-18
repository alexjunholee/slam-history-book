# 14장 단안 깊이 추정: Eigen에서 Depth Anything까지

단안 깊이 추정(monocular depth estimation)은 컴퓨터 비전에서 손꼽히는 오래된 ill-posed 문제다. 3차원 세계가 2차원 이미지 평면으로 투영되는 순간 거리 정보는 소실되므로, 기하학만 놓고 보면 똑같은 사진 한 장을 만들어 낼 수 있는 3차원 장면의 조합은 무한히 많다. 그런데도 사람은 원근감과 가림(occlusion), 텍스처의 성글고 촘촘한 기울기, 표면의 음영 같은 시각 단서를 엮어 눈 한쪽만으로도 깊이를 가늠한다. 2014년 뉴욕대학교(NYU) 대학원생 David Eigen이 CNN을 이용해 단일 이미지에서 깊이 맵을 직접 예측해 내자, 이 학습된 깊이 예측을 단안 SLAM의 빈틈에 끼워 넣으려는 시도가 물밀듯 이어졌다.

---

## 14.1 Eigen 2014 — 초기 CNN depth

딥러닝 이전에도 사진 한 장에서 깊이를 배우려는 시도는 있었다. 2005년 NIPS에서 Stanford의 Ashutosh Saxena 등은 [Markov Random Field(MRF)로 단일 이미지의 depth map을 예측하는 연구](https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images)를 발표했고, 뒤이은 Make3D 시스템은 실외 사진 속 superpixel마다 작은 평면을 배치해 거친 3차원 구조를 복원했다. Make3D는 CNN 시대 이전에 수작업 시각 특징으로 단안 깊이를 추정한 대표적 이정표다.

판도를 바꾼 것은 Eigen, Puhrsch, Fergus가 NIPS 2014에 발표한 [Eigen et al. 2014](https://arxiv.org/abs/1406.2283)였다. 이 네트워크는 두 단계로 작동한다. 먼저 coarse network가 이미지 전체의 굵직한 원근 구조와 바닥·벽 배치를 훑어 거친 깊이를 잡으면, 뒤따르는 fine network가 원본 영상의 국소 세부와 물체 경계를 살려 깊이를 날카롭게 다듬는다. 저자들은 NYU Depth v2의 실내 464개 장면 중 학습용 249개 장면에서 뽑은 약 120,000장의 영상과, 실외 주행 데이터인 KITTI로 각각 모델을 학습시켰고, 두 벤치마크 모두에서 같은 데이터로 학습한 Make3D를 확실히 앞섰다.

하지만 **scale ambiguity**(척도 모호성)라는 벽은 그대로 남았다. 신경망이 뱉어 내는 절대 미터 값은 학습 데이터셋의 방 크기와 카메라 시야각 분포에 암묵적으로 묶여 있었다. 가령 NYU 실내 영상으로 학습한 모델을 야외 풍경에 들이대면 스케일이 크게 어긋나기 일쑤였다. 절대 미터 단위의 단안 metric depth를 낯선 환경에서도 일관되게 예측하는 일은 그로부터 10년이 지난 2024년에도 핵심 과제로 남았다.

> 🔗 **차용.** Eigen 2014는 Saxena 등이 개척한 '단일 이미지에서 직접 depth map을 회귀한다'는 문제 정의를 고스란히 이어받되, 수작업 특징과 MRF 추론을 다중 스케일 CNN으로 대체해 정확도를 끌어올렸다.

---

## 14.2 Garg → Godard — self-supervised depth

Eigen 방식의 supervised 학습에는 치명적인 병목이 있었다. 픽셀마다 정답 깊이(ground truth)가 달린 대량의 데이터가 필요하다는 점이다. Kinect는 실내에서는 쓸 만했지만 햇빛이 비치는 실외에서는 무력했고, 고가 LiDAR로 대규모 야외 RGB-D 데이터셋을 구축하는 데는 막대한 비용이 들었다.

2016년 호주 애들레이드 대학의 [Ravi Garg 등](https://arxiv.org/abs/1603.04992)은 정답 깊이 레이블 대신 스테레오 카메라가 찍은 좌우 이미지 쌍 자체를 학습 신호로 쓰는 영리한 우회로를 열었다. 왼쪽 이미지만 신경망에 넣어 깊이를 예측한 뒤, 그 깊이와 이미 알고 있는 카메라 baseline을 이용해 왼쪽 이미지를 오른쪽 카메라 시점으로 옮겨 그렸을 때, 실제 오른쪽 이미지와 밝기가 같아야 한다는 점을 이용한 것이다. 재구성한 영상과 실제 영상의 밝기 차이(photometric loss)만 줄이면 되니 비싼 깊이 정답이 필요 없었다.

2017년 UCL의 Clément Godard는 이 아이디어를 한층 정교하게 다듬어 [Godard et al. 2017](https://doi.org/10.1109/CVPR.2017.699)의 **MonoDepth**를 발표했다. 왼쪽에서 예측한 disparity와 오른쪽에서 예측한 disparity가 서로 일관되도록 묶는 left-right consistency 제약을 얹고, 단순 L1 밝기 차이에 구조적 유사도(SSIM)를 결합해 경계 품질을 크게 높였다. 스테레오 쌍은 학습 중에만 쓰이고 실제 추론에는 이미지 한 장만 들어가는데도, 당시 KITTI 평가 기준에서 기존 self-supervised 기법들을 모두 앞질렀다.

한 걸음 더 나아간 작품이 2019년 ICCV에 나온 Godard의 **MonoDepth2**([Godard et al. 2019, ICCV](https://arxiv.org/abs/1806.01260))다. 이제는 학습 때조차 스테레오 카메라 없이 일반 단안 비디오 한 줄만 있으면 됐다. 별도의 pose network가 연속 프레임 사이의 카메라 움직임을 예측하면, depth network가 뽑은 깊이와 합쳐 이웃 프레임을 현재 시점으로 warping하고 그 오차가 줄어들도록 두 망을 동시에 학습시키는 구조다. 단안 비디오 학습의 고질적인 오류를 막기 위해 두 가지 정교한 장치도 고안됐다. 한 프레임에서 물체에 가려진(occluded) 영역이 평균 오차를 오염시키지 않도록 여러 소스 프레임 중 가장 오차가 작은 값만 취하는 **minimum reprojection loss**를 썼고, **auto-masking**을 통해 warping 없이 그대로 비교한 identity reprojection error가 warping 후 오차 이하로 떨어지는 픽셀—가령 카메라가 정지해 있거나 카메라와 나란히 같은 속도로 달리는 앞차—은 학습 신호에서 자동으로 걸러 냈다.

그럼에도 한계는 분명했다. 움직이는 물체나 거울·유리 같은 반사면에서는 밝기 일치(photometric consistency) 가정 자체가 성립하지 않았고, 하늘처럼 무늬 없는 영역에서는 밝기 기울기가 없어 깊이가 모호해졌으며, 단안 비디오만으로는 절대 미터 스케일까지 배울 수 없었다.

> 🔗 **차용.** Garg와 Godard가 신경망의 지도 신호로 삼은 photometric loss는 사실 고전 스테레오 매칭 문헌에서 오래 써 온 밝기 일치 제약의 변주다. [Scharstein과 Szeliski(2002)](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf)가 집대성한 disparity 추정 알고리즘의 핵심 가정이 미분 가능한 image warping을 통해 딥러닝 손실 함수로 옮겨 온 셈이다.

---

## 14.3 MiDaS — 데이터셋 혼합

단안 깊이 모델이 연구실 데이터셋 울타리를 넘어 어디서나 통하려면 거대한 데이터 규모가 필요했지만, 세상에 흩어진 깊이 데이터셋들은 저마다 단위와 성질이 제각각이었다. Intel의 René Ranftl 등이 발표한 **MiDaS**([Ranftl et al. 2022](https://doi.org/10.1109/TPAMI.2020.3019967), "Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer", 2020년 온라인 공개 후 2022년 TPAMI 44권 3호 게재)는 바로 이 이질적인 데이터셋들을 하나의 모델로 녹여 낸 전환점이었다.

실제로 MiDaS가 끌어모은 데이터는 웹 스테레오 영상(ReDWeb), 인터넷 관광 사진의 SfM 재구성(MegaDepth), 3D 영화에서 추출한 disparity(3D Movies) 등 스케일도 다르고 원점 시프트(shift)도 제멋대로인 자료들이었다. 이를 그대로 섞어 학습하면 데이터셋마다 손실 함수의 기준이 충돌해 학습이 망가진다.

Ranftl 팀의 해법은 **affine-invariant loss**였다. 손실을 계산하기 직전, 매 이미지마다 예측 깊이 맵과 정답 깊이 맵에서 각각 중앙값을 빼서 시프트를 지우고, 중앙값으로부터의 절대편차 평균으로 나누어 스케일까지 지워 버린 뒤 오차를 재는 방식이다. 이 정규화 덕분에 서로 다른 단위의 데이터셋을 거리낌 없이 섞어 학습할 수 있었고, 그 대가로 네트워크는 절대 미터가 아닌 상대적인 앞뒤 깊이(relative depth)를 배우게 됐다.

결과는 인상적이었다. 초기 MiDaS부터 학습에 한 번도 넣지 않은 NYU나 KITTI에서도 곧바로 뛰어난 성능을 보이는 zero-shot cross-dataset transfer를 입증했고, 이후 버전을 거듭하며 학습 데이터셋을 최대 12개까지 불렸다. 2021년에는 Ranftl 팀이 ViT(Vision Transformer) 기반의 [DPT(Dense Prediction Transformer)](https://arxiv.org/abs/2103.13413)를 발표해 MiDaS v3부터 기본 backbone을 CNN에서 Transformer로 교체했고, v3.1(2022)에서는 다양한 최신 Transformer backbone으로 선택지를 넓혔다.

> 🔗 **차용.** 뒤이어 등장한 Depth Anything 역시 DINOv2로 사전 학습한 ViT encoder 위에 바로 이 MiDaS 계보의 DPT decoder를 얹어 구조를 짰다.

---

## 14.4 Depth Anything — 데이터 규모 확대

2024년 1월 홍콩대(HKU)와 TikTok의 Lihe Yang 등이 내놓은 **Depth Anything**([Yang et al. 2024](https://arxiv.org/abs/2401.10891))은 데이터 규모를 한 차원 더 끌어올리며 범용 단안 깊이의 새 기준을 세웠다. 이들은 6개 공개 데이터셋의 레이블 이미지 150만 장에 더해, 8개 대규모 데이터셋에서 모은 레이블 없는(unlabeled) 이미지 6,200만 장을 학습에 쏟아부었다. 먼저 레이블 데이터로 학습한 교사 모델이 6,200만 장의 일반 이미지에 가짜 정답(pseudo-label)을 붙이면 학생 모델이 이를 강하게 증강된 영상으로 따라 배우는 동시에, 사전 학습된 DINOv2 encoder의 semantic 특징 공간과 정렬하는 보조 손실을 더해 기하와 의미 정보를 함께 붙잡았다. ViT-S(24.8M), ViT-B(97.5M), ViT-L(335.3M) 세 규모로 공개된 Depth Anything은 KITTI, NYUv2, Sintel, DDAD, ETH3D, DIODE 등 6개 zero-shot relative depth 벤치마크에서 MiDaS v3.1을 일제히 앞섰다.

같은 해 여름 공개된 [Depth Anything v2](https://arxiv.org/abs/2406.09414)는 한 걸음 더 대담한 전략을 취했다. 실제 데이터셋의 센서 깊이에는 잡음과 누락이 끼고 유리·반사면이 망가지기 때문에, 아예 Virtual KITTI 2와 Hypersim 같은 100% 정밀한 **합성 데이터**만으로 교사 모델을 학습시킨 뒤, 그 교사가 실제 영상에 달아 준 깔끔한 pseudo-label로 학생 모델을 훈련한 것이다. 그 덕분에 v2는 v1보다 훨씬 날카로운 물체 경계와 머리카락·전선 같은 얇은 구조를 잡아냈다.

한편 상대 깊이만으로는 부족한 로보틱스와 SLAM의 요구에 맞춰 절대 미터(metric) 깊이를 되찾으려는 노력도 병행됐다. [ZoeDepth(Shariq Farooq Bhat et al. 2023)](https://arxiv.org/abs/2302.12288)는 MiDaS류 상대 깊이 backbone 위에 실내·실외 도메인을 자동 판별해 연결하는 metric-bin head를 얹었고, [Depth Anything v2(2024)](https://arxiv.org/abs/2406.09414) 역시 사전 학습 backbone을 metric 데이터로 따로 fine-tuning한 모델을 함께 공개했다. 또 [Metric3D v2(2024)](https://arxiv.org/abs/2404.15506)는 카메라마다 초점거리가 달라 생기는 크기 착시를 없애기 위해 입력 영상을 가상의 표준 카메라 공간(canonical camera space)으로 변환해 깊이를 추론한 뒤 다시 원래 카메라 파라미터로 되돌리는 기법으로 도메인 간 metric 일관성을 높였다.

---

## 14.5 SLAM에서 쓰기

2017년 CNN-SLAM 무렵부터 연구자들은 이렇게 똑똑해진 단안 깊이 모델을 SLAM 파이프라인에 직접 꽂아 넣기 시작했다. 학습된 깊이가 가장 즉각적으로 빛을 발하는 순간은 단안 SLAM의 **초기화**다. 순수 기하적 단안 SLAM은 카메라가 충분히 옆으로 움직여 삼각측량할 baseline이 확보될 때까지 지도를 띄우지 못하고 scale도 정할 수 없다. 반면 첫 프레임부터 단안 깊이 모델의 예측을 prior로 쥐여 주면 지도를 즉시 세울 수 있고, metric depth 모델이거나 스케일이 보정된 모델이라면 미터 단위의 초기 규모까지 곧바로 잡을 수 있다. 2021년 발표된 [Teed와 Deng의 DROID-SLAM](https://arxiv.org/abs/2108.10869)(반복적 optical flow 갱신과 미분 가능한 BA를 결합한 시스템) 기반 후속 연구들도 초기 기하 설정에 단안 깊이 prior를 결합하는 방식을 적극 실험했다.

주행 중 쌓이는 **scale drift 억제**에도 요긴하다. 단안 visual odometry는 거리가 길어질수록 지도의 축척이 서서히 줄거나 늘어나는 drift를 피하기 어려운데, metric으로 보정된 단안 깊이 예측을 주기적인 스케일 닻(anchor)으로 삼으면 drift를 붙잡을 수 있다(물론 MiDaS 같은 relative depth 모델만 단독으로 쓸 때는 여전히 외부 metric 기준이 필요하다).

> 📜 **예언 vs 실제.** Eigen은 2014년 논문 말미에서 표면 법선(surface normal) 같은 다른 3차원 기하 단서를 함께 예측하는 방향을 다음 과제로 꼽았다. 그 예언은 이듬해 Eigen과 Fergus가 depth, surface normal, semantic label을 단일 multi-scale CNN으로 동시에 예측한 [ICCV 2015 논문](https://arxiv.org/abs/1411.4734)으로 곧바로 실현됐고, 이후 depth와 scene parsing을 엮은 [PAD-Net](https://arxiv.org/abs/1805.04409)(CVPR 2018) 같은 다중 작업(multi-task) 학습 흐름으로 이어졌다.

---

## 아직 열린 문제

**반사·투명 표면의 depth.** 유리창, 물웅덩이, 번들거리는 금속 표면에서는 카메라에 맺힌 상이 표면 자체가 아니라 그 너머나 반사된 풍경을 비추기 때문에 기하와 외관 단서가 정면으로 충돌한다. Depth Anything v2가 합성 데이터로 이 공백을 메우려 했고(§14.4), 로봇 파지를 겨냥한 [ClearGrasp(Sajjan et al. 2020)](https://arxiv.org/abs/1910.02550)처럼 특정 환경에 맞춘 해법도 나왔지만, 일상 속 투명·반사 물체를 완벽히 풀어내는 일은 여전히 까다롭다.

**동적 장면에서 카메라 운동과 물체 운동의 분리.** 자동차와 보행자가 뒤섞여 움직이는 거리에서는 정적인 장면을 전제로 한 photometric consistency가 깨진다. self-supervised 학습들은 주로 움직이는 객체를 마스킹해 학습에서 제외하는 소극적 방식을 택했고, [Ranjan 등(2019)](https://arxiv.org/abs/1805.09806)처럼 카메라의 자체 운동(ego-motion)과 개별 물체의 운동, 그리고 깊이를 한꺼번에 풀어내려는 연구들이 시도됐으나 복잡한 실제 도심 장면을 깔끔히 분해하기까지는 아직 갈 길이 남았다.

**Metric scale의 일반화.** ZoeDepth나 Depth Anything v2의 metric 모델은 레이블 학습 분포에 기대고, Metric3D v2는 카메라의 초점거리(intrinsic) 입력을 전제로 스케일을 맞춘다. 하지만 메타데이터조차 없는 인터넷 영상이나 CCTV 기록처럼 카메라 내장 파라미터를 전혀 모르는 상황도 허다하다. 최근 [UniDepth](https://arxiv.org/abs/2403.18913)(Piccinelli et al., CVPR 2024)와 Apple의 [Depth Pro](https://arxiv.org/abs/2410.02073)(Bochkovskii et al., ICLR 2025)가 내장 파라미터 없이 사진 한 장만으로 metric depth를 낸다고 발표했으나, UniDepth 저자들 스스로도 ETH3D나 IBims-1 같은 데이터셋에서는 여전히 장면 스케일을 놓치는 경우가 있다고 시인했다.

---

신경망이 사진 한 장에서 조밀한 깊이 맵까지 읽어 내기 시작하자, 연구자들의 상상력은 자연스럽게 한 걸음 더 나아갔다. 깊이뿐 아니라 카메라의 위치와 움직임, 나아가 SLAM의 최적화 과정 전체를 아예 하나의 신경망으로 종단간(end-to-end) 학습해 버리면 어떨까? 실제로 2015년 케임브리지에서 발표된 [PoseNet](https://arxiv.org/abs/1505.07427)은 특징점 추출도, 기하학적 최적화도 모두 건너뛴 채 이미지 한 장에서 곧바로 6-DoF 카메라 자세를 출력하는 대담한 실험을 시작했다. 다음 장에서는 그 종단간 학습의 도전과 한계를 살펴본다(→ 15장).