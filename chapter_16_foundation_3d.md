# Ch.16 — Foundation 3D: DUSt3R에서 VGGT까지

Naver Labs Europe의 Philippe Weinzaepfel과 Jérôme Revaud 등은 2022년 CroCo를 발표했다. 같은 장면을 찍은 두 이미지를 단서로 visual representation을 학습하는 cross-view self-supervised pretraining(두 시점 영상을 이용한 자기지도 사전학습) 방법이었다. CroCo는 feature learning 연구였다. 1년 남짓 뒤 같은 연구소가 CroCo의 구조를 바탕으로, calibration 없이 pointmap을 바로 출력하는 DUSt3R를 내놓았다. 2025년에는 Oxford VGG 그룹과 Meta AI의 VGGT가 나왔다.

---

## 16.1 DUSt3R — learned pointmap

2010년대 내내 3D 재구성은 같은 절차를 따랐다. 특징점을 찾아 매칭하고, 카메라 내부·외부 파라미터를 추정하고, triangulation으로 점군을 만들고, bundle adjustment로 전체를 다듬는다. [COLMAP(Schönberger & Frahm, CVPR 2016)](https://openaccess.thecvf.com/content_cvpr_2016/html/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.html)은 이 파이프라인을 구현한 대표적인 공개 소프트웨어다.

[Shuzhe Wang et al. 2024. DUSt3R: Geometric 3D Vision Made Easy (CVPR)](https://arxiv.org/abs/2312.14132)는 이 절차를 거치지 않는다. 이미지 두 장을 받아 각 픽셀의 3D 좌표를 바로 출력하고, 내부 파라미터(focal length, principal point)를 입력으로 받지 않는다. pointmap이라 부르는 이 출력은 두 이미지의 픽셀을 모두 첫 번째 카메라 좌표계로 나타낸 3D 좌표다.

DUSt3R의 transformer는 CroCo에서 물려받은 encoder-decoder 구조를 쓴다. 각 이미지는 따로 encoding되고, 두 decoder가 정보를 주고받는다. decoder 블록마다 먼저 self-attention으로 같은 이미지의 token끼리 참조하고, 이어서 cross-attention으로 다른 이미지의 token을 모두 참조한다. 어느 픽셀이 다른 이미지의 어느 픽셀과 같은 3D 점을 보는지는 이 cross-attention이 대규모 데이터에서 암묵적으로 배운다. 논문은 Habitat, MegaDepth, ARKitScenes, Static Scenes 3D, BlendedMVS, ScanNet++, CO3D-v2, Waymo 등 여덟 개 데이터셋에서 이미지 쌍 850만 개를 뽑았다. 정답은 합성 데이터, SfM 소프트웨어의 재구성 결과, 센서 측정에서 얻었으므로, 정답의 일부는 고전 SfM이 만들었다.

> 🔗 **차용.** DUSt3R의 backbone은 ViT([Dosovitskiy et al., ICLR 2021](https://arxiv.org/abs/2010.11929))에서 가져왔다. 더 직접적인 선행 연구는 Naver Labs Europe의 CroCo([Weinzaepfel et al. 2022](https://arxiv.org/abs/2210.10716))다. CroCo는 한 이미지에서 가린 영역을 다른 이미지의 정보로 복원하는 cross-view completion으로 사전학습을 했다. DUSt3R는 CroCo의 encoder-decoder 구조를 그대로 쓰고 과제만 pointmap 예측으로 바꿨다.

두 이미지의 pointmap은 이미 첫 카메라의 좌표계에 함께 놓여 있다. 그래서 상대 카메라 자세는 pointmap에서 바로 구할 수 있다. 논문은 두 pointmap을 Procrustes alignment로 맞추는 방법을 먼저 들고, 잡음과 outlier에 더 강한 방법으로 PnP-RANSAC을 쓴다.

이미지가 세 장 이상이면 DUSt3R는 global alignment를 푼다. 모든 이미지 쌍의 pointmap을 하나의 좌표계로 정합하는 최적화다. bundle adjustment와 비슷하지만 feature matching이나 카메라 모델 없이 진행한다.

---

## 16.2 매칭까지 하는 MASt3R

DUSt3R는 재구성의 핵심 단계인 픽셀 대응(feature matching)을 암묵적으로만 다룬다. SuperPoint+SuperGlue나 LightGlue 같은 명시적 매칭을 대신하기에는 부족했다.

[Vincent Leroy et al. 2024. Grounding Image Matching in 3D with MASt3R (ECCV)](https://arxiv.org/abs/2406.09756)는 DUSt3R에 matching head를 더한다. pointmap과 함께 픽셀마다 서술자(descriptor)를 출력하고, 3D 위치와 서술자를 함께 학습한다. 매칭은 두 이미지에서 서로가 서로의 nearest neighbor인 짝을 찾는 reciprocal matching으로 하며, 논문은 이를 빠르게 하는 알고리즘도 함께 냈다.

> 🔗 **차용.** MASt3R의 3D-anchored matching은 SuperGlue([Sarlin et al. 2020](https://arxiv.org/abs/1911.11763))가 풀려던 2D 서술자의 모호성 문제를 다른 방법으로 다룬다. SuperGlue는 그래프 신경망으로 주변 맥락을 써서 모호성을 줄였다. MASt3R는 3D 구조를 함께 학습해서 줄인다.

2024년 12월 Imperial College London Davison 그룹의 Riku Murai, Eric Dexheimer, Andrew Davison이 [MASt3R-SLAM](https://arxiv.org/abs/2412.12392)을 공개했다(CVPR 2025). 이 시스템은 MASt3R의 매칭을 frontend로, 그래프 기반 global optimization을 backend로 쓰고, 논문 실험에서는 RTX 4090 한 장으로 약 15 fps를 냈다.

MASt3R는 정확한 calibration 없이도 dense 매칭을 한다. foundation 모델의 출력을 COLMAP 형식의 SfM 파이프라인에 잇는 구성도 나왔다. 2025년 6월, §16.4에서 다룰 VGGT의 공개 저장소에 예측을 COLMAP 형식으로 저장하고 bundle adjustment를 선택해 돌리는 스크립트가 들어갔다.

> 📜 **예언 vs 실제.** DUSt3R는 이미지 쌍을 처리한 뒤 global alignment로 묶는 구조여서, 순차 처리와 실시간 동작은 후속 연구가 맡았다. DUSt3R 공개 후 8개월 만인 2024년 8월에 Spann3R가 global alignment 없이 이미지를 차례로 처리했고, 2024년 12월에 MASt3R-SLAM이 SLAM 시스템으로 묶었다.

---

## 16.3 Spann3R — sequential 처리

DUSt3R와 MASt3R는 이미지 집합을 한꺼번에 받아 처리한다. SLAM에서는 이미지가 시간 순서로 한 장씩 들어오고, 프레임마다 지도를 갱신해야 한다.

[Hengyi Wang & Lourdes Agapito 2025. 3D Reconstruction with Spatial Memory (Spann3R, 3DV)](https://arxiv.org/abs/2408.16061)는 DUSt3R의 구조를 sequential 처리에 맞게 고쳤다. 이미 처리한 프레임의 정보를 외부 spatial memory에 저장하고, 새 프레임이 들어오면 이 memory를 조회해 공통 좌표계의 pointmap을 바로 예측한다. global alignment 최적화가 필요 없다.

> 🔗 **차용.** Spann3R는 DUSt3R의 ViT encoder와 두 decoder 구조를 그대로 쓴다. 이전에 예측한 pointmap과 decoder의 기하 feature를 가벼운 memory encoder로 memory key와 value로 만들고, decoder의 기하 feature로 query를 만들어 memory를 조회한다. memory 관리 방식은 사람의 기억 모델을 본뜬 X-Mem을 참고했다.

Spann3R도 DUSt3R처럼 calibration 없이 동작하고, 이미지가 들어올 때마다 지도를 갱신한다. 논문은 RTX 4090 한 장에서 test-time 최적화 없이 50 fps 넘게 점진적으로 재구성했다고 보고했다.

---

## 16.4 VGGT — multi-view joint inference

Spann3R는 프레임을 하나씩 처리하면서 memory로 과거 프레임을 참조한다. Oxford VGG 그룹과 Meta AI의 Jianyuan Wang, Minghao Chen, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, David Novotny는 2025년 초 새 방법을 내놓았다. 여러 이미지를 한꺼번에 받아 카메라 파라미터, depth map, point map, 3D point track을 forward pass 한 번으로 출력한다.

[Jianyuan Wang et al. 2025. VGGT: Visual Geometry Grounded Transformer (CVPR)](https://arxiv.org/abs/2503.11651)는 DUSt3R의 쌍 단위 처리를 여러 시점 동시 추론으로 바꿨다. DUSt3R로 N장을 다루려면 이미지 쌍마다 pointmap을 구하고(최대 N(N-1)/2쌍) global alignment를 풀어야 한다. VGGT는 N장을 한꺼번에 transformer에 넣고 attention으로 모든 이미지 사이의 관계를 함께 처리한다. 논문은 수백 장까지 한꺼번에 입력으로 받는다고 적었다. 논문이 H100 한 장에서 잰 feature backbone 처리 시간은 20장에 0.31초, 100장에 3.12초였다.

논문의 정량 비교에서 VGGT는 후처리 없이도 카메라 자세 정확도와 점군 품질에서 DUSt3R보다 나았고, global alignment 최적화가 없어 더 빨랐다.

---

## 16.5 pose estimation과 reconstruction의 경계

컴퓨터 비전은 지도 기반 localization(알려진 지도에서 현재 위치 찾기)과 3D 재구성(처음 보는 환경의 기하 복원)을 따로 다뤘다. SLAM은 둘을 동시에 풀어야 해서 어려웠다.

DUSt3R부터 VGGT까지의 시스템은 기하와 카메라를 추정할 때 같은 학습 표현을 쓴다. DUSt3R는 pointmap에서 따로 계산해 pose를 구하고, 여러 뷰를 global alignment로 묶는다. VGGT는 forward pass 한 번으로 카메라와 기하를 함께 출력하므로 그런 후처리가 없다.

multi-view geometry는 학습 신호로 남아 있다. DUSt3R, MASt3R, VGGT는 SfM 재구성과 센서 데이터에서 얻은 정답으로 자세, depth, pointmap을 학습하면서 여러 시점의 기하 규칙을 이용한다. 다만 epipolar constraint, triangulation, bundle adjustment 같은 알고리즘이 transformer 가중치 안에 그대로 학습됐는지는 어느 논문도 보이지 않았다. 없어진 것은 명시적 파이프라인의 일부다. DUSt3R가 실패하면 그 원인이 네트워크 가중치 안에 있어서, 어느 단계에서 틀렸는지 짚기 어렵다.

> 📜 **예언 vs 실제.** MASt3R 논문의 결론은 여러 벤치마크에서 카메라 자세와 localization 성능을 끌어올렸다는 점과, 이미지가 적을 때도 동작하므로 localization의 쓰임새를 크게 넓힐 것이라는 기대를 적었다. 같은 해 MASt3R의 매칭은 MASt3R-SLAM의 frontend로 쓰였다(§16.2).

---

## 16.6 Semantic foundation model을 지도에 쓰기

DUSt3R, MASt3R, VGGT는 pointmap, 카메라 자세, 기하 구조를 다루는 geometric foundation model이다. 같은 시기 SLAM 연구에서는 CLIP, DINO, SAM 같은 모델을 지도에 넣는 semantic 연구도 늘었다. semantic 모델을 쓰면 미리 정한 class 목록(dictionary) 밖의 개념도 지도에 담을 수 있다.

계층적 semantic 지도를 실시간으로 만든 예로는 MIT Luca Carlone 그룹의 연구가 있다. [Nathan Hughes et al. 2022. Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization (RSS)](https://arxiv.org/abs/2201.13360)는 Kimera(Rosinol et al., ICRA 2020)의 metric-semantic mesh 위에 objects → places → rooms → buildings의 계층적 scene graph를 online으로 만들었다. 정해진 class만 아는 분류기(closed-set)를 썼으므로, Handbook 16장이 말하는 "relatively small (e.g., 100-1000 labels) and predefined dictionary"의 제약 안에 있었다. 논문은 Hydra를 처음으로 실시간 동작하는 spatial perception system이라고 소개했다.

이 제약은 foundation model을 쓰는 연구가 다뤘다. ETH Zurich와 Google의 Songyou Peng, Marc Pollefeys 등이 [Songyou Peng et al. 2023. OpenScene: 3D Scene Understanding with Open Vocabularies (CVPR)](https://arxiv.org/abs/2211.15654)를 냈다. 이어 Toronto, Montréal, MIT 등의 공동 연구로 [Qiao Gu et al. 2024. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA)](https://arxiv.org/abs/2309.16650)가 발표됐다. OpenScene은 CLIP feature를 3D 점군에 distillation해, 사용자가 넣은 임의의 단어와 장면의 각 점이 얼마나 맞는지 보여 주게 했다. ConceptGraphs는 class label 대신 VLM이 만든 설명문을 노드 속성으로 달고, 객체 사이의 관계를 LLM이 서술하게 했다. 두 연구로 미리 정한 class 밖의 개념도 3D 표현에 연결할 수 있게 됐다.

[Dominic Maggio et al. 2024. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs (RA-L)](https://arxiv.org/abs/2404.13696)는 로봇이 받은 자연어 과제(task)를 기준으로 scene graph를 만든다. information bottleneck 원리로 과제에 필요한 추상화 수준만 남기고, 과제와 상관없는 세부는 묶는다.

> 🔗 **차용.** Clio는 Carlone 그룹이 Hydra에 과제 기반 추상화를 더한 후속 연구다. ConceptGraphs는 open-vocabulary 객체 그래프를 따로 만든 연구다.

Ch.18 §18.4는 2017–2019년 object-as-landmark 연구가 줄어든 과정을 다룬다. semantic SLAM은 그 뒤 계층적 scene graph의 형태로 돌아왔다. DUSt3R 같은 geometric foundation model이 예측한 기하에 semantic feature를 함께 싣는 연구도 나왔다. UT Austin·NVIDIA 등의 [Fan et al. 2024. Large Spatial Model (LSM, NeurIPS)](https://arxiv.org/abs/2410.18956)은 DUSt3R로 초기화한 pointmap 예측에 LSeg feature를 더해, pose를 모르는 이미지 두 장에서 기하·외관·open-vocabulary semantic을 forward pass 한 번으로 출력했다.

---

## 16.7 SLAM에서 무엇이 남는가

MASt3R-SLAM은 고전 SLAM의 구조를 빌려 쓴다. keyframe 선택, loop closure, 지도 관리는 새 표현에서도 그대로 필요했다. DUSt3R 계열이 feature matching과 재구성의 내부를 바꿨어도 시스템 수준의 결정은 고전 방법을 따른다.

5부의 다른 시스템도 마찬가지다. NeRF-SLAM은 NeRF를 지도 표현으로 쓰면서 keyframe 기반 tracking을 유지했다. 초기 3DGS-SLAM 시스템인 MonoGS는 명시적 loop closure를 논문의 future work로 남겼다(Ch.15).

2025년 MIT의 Dominic Maggio·Hyungtae Lim·Luca Carlone이 [VGGT-SLAM](https://arxiv.org/abs/2505.12549)을 공개했다(NeurIPS 2025). VGGT가 local submap을 재구성하면, 시스템은 이어지는 submap 사이의 15자유도 projective transform을 \(\mathrm{SL}(4)\) 위에서 최적화하고 loop-closure 제약까지 함께 반영한다. transformer가 local 기하를 만들어도 global graph 최적화는 필요하다. *The SLAM Handbook* 13장(Teed, Deng, Chidlovskii, Revaud, Wimbauer, Cremers)도 DUSt3R를 다룬 절에서 "a form of factor graph is still necessary"라고 적었다(p.386).

---

## 🧭 아직 열린 것

**대규모 sequence 처리.** DUSt3R는 이미지 쌍마다 pointmap을 구하므로 쌍의 수가 이미지 수의 제곱으로 늘어난다. VGGT 논문의 비교에서 DUSt3R는 32장을 넘기면 GPU 메모리가 모자랐다. VGGT의 feature backbone은 H100 한 장에서 200장을 8.75초, 최대 메모리 40.63 GB로 처리했지만, 1,000장이나 10,000장은 그대로 넣기 어렵다. Spann3R는 점진적 방식으로 이 문제의 일부를 다뤘다. sparse attention, 계층적 global alignment 등이 시도되고 있고, 대규모 실외 환경을 처리하는 일은 남은 과제다.

**loop closure의 정의.** 고전 SLAM에서 loop closure는 전에 방문한 장소를 알아보고 누적 오차를 고치는 일이다. DUSt3R 계열에서 방문한 장소를 어떻게 표현하고, pointmap 기반 지도에 교정을 어떻게 퍼뜨릴지는 아직 정해지지 않았다. MASt3R-SLAM은 이미지 검색으로 loop closure 후보를 찾는 기존 방식을 그대로 썼다.

**metric scale의 일반화.** DUSt3R의 pointmap은 상대 scale이어서, 깊이 비율은 복원해도 절대 scale은 모른다. Metric3D나 Depth Anything v2가 metric depth를 목표로 했듯, foundation 3D에서도 여러 환경에서 metric scale을 맞히는 문제가 남는다. 이미지만으로는 절대 scale이 기하적으로 정해지지 않으므로, GPS나 IMU 없이 얻는 scale은 학습한 prior에 기댄 추정이다.

**SLAM에 쓰기.** Ch.15의 3DGS처럼 foundation 3D도 SLAM 연구에 들어오고 있다. 2024–2025년 MASt3R-SLAM과 VGGT-SLAM은 모두 foundation 모델의 출력을 그래프 최적화로 묶었다. 긴 sequence를 실시간으로 처리하는 방법과 §16.6의 semantic 연구(Hydra, Clio, ConceptGraphs)와 결합하는 방법은 남은 과제다. Ch.19에서 다시 다룬다.
