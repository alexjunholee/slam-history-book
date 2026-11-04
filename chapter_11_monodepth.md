# Ch.11 — 학습 기반 단안 깊이 추정: Eigen에서 Depth Anything까지

monocular depth(단안 깊이) 추정은 컴퓨터 비전에서 오래된 ill-posed 문제(답이 하나로 정해지지 않는 문제)다. 카메라가 3D 세계를 2D로 투영할 때 깊이 정보를 잃으므로, 이미지 한 장을 만들어 낼 수 있는 3D 장면은 기하적으로 무수히 많다. 사람은 원근감, 폐색, 텍스처 기울기, 표면의 음영으로 단안 깊이를 판단한다. 2014년 NYU 대학원생 David Eigen은 CNN으로 이미지 한 장에서 깊이를 예측했다. 그 뒤 SLAM 파이프라인에 학습한 깊이를 넣는 연구가 이어졌다.

---

## 1. Eigen 2014 — 초기 CNN depth

2014년 이전에도 학습 기반 monocular depth 연구는 있었다. Stanford의 Ashutosh Saxena 등은 2005년 NIPS에서 [Markov Random Field(MRF)로 이미지 한 장의 depth map을 예측하는 방법](https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images)을 발표했다. 후속 시스템 Make3D는 주로 실외 장면에서 superpixel마다 평면을 두어 거친 3D 구조를 예측했다. Make3D는 CNN 이전 학습 기반 단안 깊이 추정의 대표 사례다.

Eigen, Puhrsch, Fergus의 [Eigen et al. 2014](https://arxiv.org/abs/1406.2283)는 NIPS에서 발표됐다. 네트워크는 두 단계로 되어 있어, coarse network가 장면 전체의 깊이 구조를 예측하면 fine network가 국소 세부를 다듬는다. 저자들은 NYU Depth v2의 실내 장면 464개 가운데 학습용 249개 장면의 원시 영상에서 서로 다른 이미지 약 120,000장을 뽑아 학습했고, KITTI로도 따로 학습했다. 두 데이터셋에서 모두 같은 데이터로 학습한 Make3D보다 오차가 작았다.

> 🔗 **차용.** Eigen 2014는 Saxena 등이 정의한 문제, 곧 이미지 한 장에서 depth map을 예측하는 문제를 이어받았고, MRF 대신 CNN을 썼다.

**scale ambiguity**(척도 모호성)는 남았다. 네트워크가 내는 절대 스케일은 학습 데이터의 분포와 카메라 설정에 묶였다. NYU 실내에서 학습한 모델을 야외 영상에 쓰면 스케일이 틀리기 쉽다. 이 문제는 2024년에도 단안 metric depth(실제 미터 단위 깊이) 추정의 핵심 과제였다.

---

## 2. Garg → Godard — self-supervised depth

supervised depth 학습에는 정답 깊이가 붙은 데이터가 많이 필요했다. Kinect는 실내에서는 잘 작동했지만 햇빛 아래에서는 적외선 패턴이 보이지 않았다. 큰 실외 RGB-D 데이터셋을 만드는 데는 비용이 많이 들었다.

2016년 University of Adelaide의 [Ravi Garg 등](https://arxiv.org/abs/1603.04992)은 stereo 이미지 쌍을 학습 신호로 썼다. 왼쪽 이미지로 depth를 예측하고, 그 depth와 카메라 baseline으로 오른쪽 이미지를 재구성한다. 실제 오른쪽 이미지는 이미 있으므로, 재구성한 이미지와 실제 오른쪽 이미지의 밝기 차이(photometric loss)를 줄이도록 학습하면 정답 깊이가 필요 없다.

Clément Godard(UCL)는 2017년 이 아이디어를 [Godard et al. 2017](https://doi.org/10.1109/CVPR.2017.699)의 **MonoDepth**로 발전시켰다. 왼쪽 이미지 기준 disparity와 오른쪽 이미지 기준 disparity가 서로 맞아야 한다는 제약(left-right consistency)을 더했고, photometric loss에 구조적 유사도(SSIM)를 L1과 함께 넣었다. stereo 쌍은 학습할 때만 필요하고 추론에는 이미지 한 장만 쓴다. 원 논문은 KITTI에서 당시 비교 방식으로 평가해, 표에 실린 기존 self-supervised 방법보다 오차가 낮다고 보고했다.

> 🔗 **차용.** Garg와 Godard의 photometric loss는 stereo matching 문헌의 밝기 일치 제약을 depth network의 학습 신호로 쓴 것이다. 이 제약은 [Scharstein과 Szeliski(2002)](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf)가 정리한 disparity 추정 방법들에 공통으로 들어 있다.

2019년 Godard의 *MonoDepth2* ([Godard et al. 2019, ICCV](https://arxiv.org/abs/1806.01260))는 stereo 쌍 없이 단안 video만으로도 self-supervised 학습을 한다. pose network가 연속 프레임 사이의 카메라 운동을 예측하면, 그 운동과 depth network의 출력으로 이웃 프레임을 현재 프레임 시점으로 warping한다. 두 네트워크는 이 warping 오차가 줄도록 함께 학습한다. MonoDepth2는 여기에 **minimum reprojection loss**와 **auto-masking** 등을 더했다. **minimum reprojection loss**는 픽셀마다 여러 소스 프레임 가운데 photometric error가 가장 낮은 것만 써서, 한 프레임에서 가려진(occluded) 영역의 오류를 줄인다. **auto-masking**은 픽셀마다 두 오차를 비교한다. 하나는 source frame을 warping 없이 그대로 비교한 identity reprojection error이고, 다른 하나는 예측 depth·pose로 warping한 오차다. 앞의 오차가 뒤의 오차 이하인 픽셀은 학습에서 뺀다. 이 조건으로 카메라가 멈춘 프레임이나 카메라와 비슷한 속도로 움직여 인접 프레임에서 겉보기 위치가 거의 변하지 않는 물체를 거른다.

움직이는 물체와 반사 표면에서는 photometric consistency(같은 점은 어느 프레임에서나 밝기가 같다는 가정)가 깨진다. 하늘처럼 texture가 없는 영역에서는 밝기 차이로 대응을 정할 수 없다. 단안 video로 학습하면 스케일도 정해지지 않는다.

---

## 3. MiDaS — 데이터셋 혼합

Intel의 René Ranftl 등은 [Ranftl et al. 2022](https://doi.org/10.1109/TPAMI.2020.3019967)에서 **MiDaS**를 발표했다. 논문 제목은 "Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer"이고, 2020년 IEEE TPAMI에 온라인으로 먼저 공개된 뒤 2022년 44권 3호에 실렸다. MiDaS는 여러 데이터셋을 섞어 모델 하나를 학습했다.

데이터셋마다 depth의 단위와 스케일이 다르다. 학습에 쓴 ReDWeb은 웹의 stereo 이미지, MegaDepth는 인터넷 사진의 SfM 재구성, 3D Movies는 3D 영화에서 얻은 disparity다. 그대로 섞으면 데이터셋마다 loss가 재는 기준이 달라진다.

Ranftl 등은 **affine-invariant loss**를 썼다. loss를 계산할 때 이미지마다 예측과 정답에서 스케일과 시프트를 걷어 낸 뒤 비교한다. 예측과 정답 각각에서 중앙값을 빼 shift를 없애고, 중앙값에서 떨어진 절대편차의 평균으로 나눠 scale을 없앤다. 이렇게 정규화하면 데이터셋마다 단위가 달라도 함께 학습할 수 있다. 그 대가로 네트워크는 scale과 shift가 정해지지 않은 상대 깊이를 배운다.

초기 MiDaS는 NYU나 KITTI처럼 학습에 쓰지 않은 데이터셋에서도 잘 동작했다(zero-shot cross-dataset transfer). 이후 MiDaS 버전들은 학습 데이터셋을 최대 12개로 늘렸다. 야외와 실내처럼 분포가 다른 영상에서도 relative depth(상대 깊이)를 추정했지만, 절대 스케일은 복원하지 못했다.

Ranftl 팀은 2021년 [DPT(Dense Prediction Transformer)](https://arxiv.org/abs/2103.13413)를 따로 발표해 MiDaS의 backbone을 ViT 기반으로 바꿨다. MiDaS v3부터 DPT가 기본 구조가 됐고, v3.1(2022)은 여러 transformer backbone을 더한 판이다.

> 🔗 **차용.** Depth Anything은 DINOv2로 사전학습한 encoder에 DPT decoder를 붙였다.

---

## 4. Depth Anything — 데이터 규모 확대

2024년 1월 HKU와 TikTok의 Lihe Yang 등은 [Yang et al. 2024](https://arxiv.org/abs/2401.10891)에서 **Depth Anything**을 발표했다. 이들은 공개 데이터셋 6개에서 모은 labeled 이미지 150만 장에, 공개 대규모 데이터셋 8개에서 모은 unlabeled 이미지 6,200만 장을 더했다. unlabeled 이미지에는 모델이 만든 pseudo-label(가짜 정답)을 붙여 학습에 넣었다. 사전학습한 encoder(DINOv2)의 semantic feature와 맞추는 auxiliary loss도 더했다.

Depth Anything은 KITTI, NYUv2, Sintel, DDAD, ETH3D, DIODE의 zero-shot relative depth 평가에서 MiDaS v3.1보다 나은 결과를 보고했다. 모델은 ViT-S(24.8M), ViT-B(97.5M), ViT-L(335.3M) 세 크기로 공개됐다.

같은 해 나온 [Depth Anything v2](https://arxiv.org/abs/2406.09414)는 Virtual KITTI 2, Hypersim 같은 합성 데이터로만 teacher 모델을 학습했다. 그리고 이 teacher가 실제 영상에 붙인 pseudo-label로 student 모델을 학습했다. 합성 데이터에는 반사·투명 표면처럼 실제 데이터에서 정답을 달기 어려운 부분도 정확한 depth가 있다. 저자들은 v2가 v1보다 경계와 얇은 구조를 더 세밀하게 예측한다고 보고했다.

기본 Depth Anything 모델은 relative depth만 내고 절대 스케일은 내지 않는다. [ZoeDepth(Shariq Farooq Bhat et al. 2023)](https://arxiv.org/abs/2302.12288)는 relative depth로 사전학습한 모델에 도메인별 metric-bin head를 붙이고, 입력 영상의 도메인을 자동으로 판별해 맞는 head로 보내게 했다. [Depth Anything v2(2024)](https://arxiv.org/abs/2406.09414)는 relative-depth backbone을 metric label로 따로 fine-tuning한 모델도 공개했다. [Metric3D v2(2024)](https://arxiv.org/abs/2404.15506)는 입력을 canonical camera space로 변환해 초점거리 차이에서 오는 모호성을 줄인 뒤, 추론 결과를 camera parameter로 되돌려 metric depth를 얻는다. 이렇게 단안 모델이 여러 도메인에서 metric 깊이를 내게 됐다.

---

## 5. SLAM에서 쓰기

2017년 CNN-SLAM 무렵부터 monocular depth 모델을 SLAM에 넣으려는 시도가 있었다. 학습한 깊이가 특히 쓸모 있는 곳은 초기화다. Monocular SLAM은 두 프레임에서 triangulation을 하려면 카메라가 충분히 움직여 baseline이 생겨야 하고, scale도 처음부터 정해지지 않는다.

첫 프레임에 depth prior를 주면 초기화가 빨라지고 장면 구조의 초기값도 생긴다. metric depth 모델이거나 알려진 기준으로 scale을 보정한 prior라면 metric scale의 초기값도 줄 수 있다. [Teed와 Deng이 2021년 발표한 DROID-SLAM](https://arxiv.org/abs/2108.10869)은 recurrent optical flow 갱신과 BA를 한 네트워크 안에서 반복한다. DROID-SLAM을 바탕으로 한 후속 연구들은 monocular depth prior를 기하 초기화에 쓰는 방식을 실험했다.

scale 복원에도 쓰였다. 단안 visual odometry(VO)는 이동할수록 scale drift가 쌓인다. metric으로 보정한 depth 예측을 주기적으로 scale 기준으로 삼으면 이 drift를 줄일 수 있다. MiDaS 같은 relative depth 출력을 쓰려면 외부 metric 기준이 따로 있어야 한다.

> 📜 **예언 vs 실제.** Eigen은 2014년 논문 끝에서 surface normal 같은 3D 기하 정보를 함께 다루는 것을 다음 연구로 들었다. 이듬해 Eigen과 Fergus는 [depth, surface normal, semantic label을 한 multi-scale 네트워크로 예측하는 논문](https://arxiv.org/abs/1411.4734)(ICCV 2015)을 냈다. depth와 scene parsing을 함께 학습한 [PAD-Net](https://arxiv.org/abs/1805.04409)(CVPR 2018) 같은 multi-task 연구도 이어졌다.

---

## 🧭 아직 열린 것

**반사·투명 표면의 depth.** 유리, 물, 금속 반사면에서는 카메라에 찍힌 상이 그 표면의 위치를 알려 주지 않는다. Depth Anything v2는 합성 데이터로 이런 표면의 정답을 보충했다(§4). 투명 물체를 집는 로봇용 [ClearGrasp(Sajjan et al. 2020)](https://arxiv.org/abs/1910.02550) 같은 특정 상황용 방법도 있다.

**동적 장면에서 카메라 운동과 물체 운동의 분리.** 자동차, 사람, 자전거가 움직이는 장면에서는 photometric consistency가 성립하지 않는다. self-supervised 방법들은 대개 움직이는 물체를 masking해서 학습에서 뺐다. 움직이는 물체의 depth와 카메라 자체의 운동(ego-motion)을 함께 푸는 문제는 [Ranjan et al.(2019)](https://arxiv.org/abs/1805.09806)을 비롯한 여러 연구가 시도했다.

**Metric scale의 일반화.** ZoeDepth와 Depth Anything v2의 metric 모델은 metric label로 scale을 학습한다. Metric3D v2는 camera parameter를 입력으로 받아 보정한다. 그런데 intrinsic(초점거리 같은 카메라 내부 파라미터)이나 믿을 만한 메타데이터가 없는 CCTV·기록 영상도 많다. [UniDepth](https://arxiv.org/abs/2403.18913)(Piccinelli et al., CVPR 2024)와 Apple의 [Depth Pro](https://arxiv.org/abs/2410.02073)(Bochkovskii et al., ICLR 2025)는 카메라 intrinsic 없이 이미지 한 장에서 metric depth를 낸다고 발표했다. UniDepth 저자들은 ETH3D와 IBims-1처럼 장면 scale을 제대로 잡지 못하는 경우가 있다고 함께 적었다.

---

자세도 네트워크로 바로 추정하려는 연구가 있었다. 2015년 Cambridge의 [PoseNet](https://arxiv.org/abs/1505.07427)은 feature 추출이나 최적화 없이 이미지 한 장에서 절대 자세를 바로 출력했다. Ch.12는 이런 end-to-end 방법을 다룬다.
