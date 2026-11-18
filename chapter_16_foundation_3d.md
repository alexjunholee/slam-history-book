# 20장 3D foundation model: DUSt3R에서 VGGT까지

2022년 Naver Labs Europe의 Philippe Weinzaepfel과 Jérôme Revaud 등이 발표한 CroCo는 같은 장면을 다른 각도에서 찍은 두 이미지를 이용해 시각 표현을 배우는 자기지도 사전학습(cross-view self-supervised pretraining) 연구였다. 출발점은 순수한 feature learning이었다. 그런데 불과 1년 남짓 뒤, 같은 연구소는 CroCo의 뼈대 위에 전혀 다른 차원의 시스템을 올려놓았다. 카메라 내부 파라미터도, 특징점 매칭도 건너뛰고 이미지에서 곧장 3D pointmap을 토해 내는 DUSt3R였다. 이어 2025년에는 Oxford VGG 그룹과 Meta AI가 여러 시점을 한 번에 꿰어 내는 VGGT를 내놓으면서, 수십 년간 굳어 있던 3D 재구성의 순서 자체가 흔들리기 시작했다.

---

## 20.1 DUSt3R — learned pointmap

2010년대 내내 3D 재구성은 사실상 한 가지 정해진 각본을 따랐다. 특징점을 뽑아 짝을 짓고, 카메라 내부·외부 파라미터를 추정한 뒤, triangulation으로 점군을 세우고 마지막으로 bundle adjustment를 돌려 전체 오차를 다듬는 순서다. [COLMAP(Schönberger & Frahm, CVPR 2016)](https://openaccess.thecvf.com/content_cvpr_2016/html/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.html)은 바로 이 고전 파이프라인을 가장 완성도 높게 구현한 대표 소프트웨어였다.

[Shuzhe Wang et al. 2024. DUSt3R: Geometric 3D Vision Made Easy (CVPR)](https://arxiv.org/abs/2312.14132)는 이 오랜 절차를 통째로 우회한다. 입력은 이미지 두 장뿐이며, 초점 거리(focal length)나 주점(principal point) 같은 카메라 내부 파라미터조차 받지 않는다. 출력은 두 이미지의 모든 픽셀을 첫 번째 카메라 좌표계로 함께 옮겨 놓은 3D 좌표 지도, 곧 **pointmap**이다.

이 과감한 도약의 엔진은 CroCo에서 물려받은 encoder-decoder 구조의 transformer다. 두 이미지는 각각 독립적으로 encoding된 뒤 한 쌍의 decoder로 들어가 정보를 주고받는다. decoder 블록마다 먼저 self-attention으로 자기 이미지 안의 token끼리 문맥을 훑고, 이어서 cross-attention으로 상대 이미지의 모든 token을 건너다본다. 어느 픽셀이 다른 이미지의 어느 픽셀과 같은 3D 점을 바라보고 있는지는 명시적 매칭 알고리즘이 아니라 이 cross-attention이 대규모 데이터 속에서 암묵적으로 터득한다. 논문은 Habitat, MegaDepth, ARKitScenes, Static Scenes 3D, BlendedMVS, ScanNet++, CO3D-v2, Waymo 등 여덟 개 데이터셋에서 850만 쌍의 이미지를 끌어와 학습시켰다. 흥미로운 점은 그 학습 정답 가운데 일부가 합성 데이터나 센서 측정뿐 아니라 바로 고전 SfM 소프트웨어의 재구성 결과에서 왔다는 사실이다.

두 이미지의 pointmap이 이미 첫 번째 카메라 좌표계라는 공통 무대 위에 놓여 있으므로, 상대 카메라 자세를 뽑아내는 일은 훨씬 단순해진다. 논문은 두 pointmap을 Procrustes alignment로 바로 맞추는 방법을 먼저 제시하고, 잡음과 outlier에 강한 실전 해법으로 PnP-RANSAC을 쓴다. 이미지가 세 장 이상으로 늘어나면 모든 이미지 쌍에서 얻은 pointmap을 하나의 전역 좌표계로 끌어모으는 global alignment 최적화를 푼다. 겉모습은 bundle adjustment를 닮았지만, 특징점 매칭이나 사전 카메라 모델 없이 돌아간다는 점이 다르다.

> 🔗 **차용.** DUSt3R의 backbone은 ViT([Dosovitskiy et al., ICLR 2021](https://arxiv.org/abs/2010.11929))에서 왔고, 더 직접적인 설계 모태는 같은 Naver Labs Europe의 CroCo([Weinzaepfel et al. 2022](https://arxiv.org/abs/2210.10716))다. CroCo는 한 이미지의 가려진 패치를 다른 시점 이미지의 단서로 복원하는 cross-view completion으로 사전학습을 수행했고, DUSt3R는 그 encoder-decoder 뼈대를 그대로 가져와 출력 과제만 pointmap 예측으로 바꿔 달았다.

---

## 20.2 매칭까지 하는 MASt3R

DUSt3R는 재구성의 핵심인 픽셀 대응을 cross-attention 내부에 숨겨 두었기에, SuperPoint+SuperGlue나 LightGlue처럼 정밀한 명시적 매칭이 필요한 자리에 그대로 쓰기에는 아쉬움이 남았다.

[Vincent Leroy et al. 2024. Grounding Image Matching in 3D with MASt3R (ECCV)](https://arxiv.org/abs/2406.09756)은 DUSt3R 위에 matching head를 얹어 이 빈칸을 메웠다. 네트워크가 pointmap과 함께 픽셀별 서술자(descriptor)를 내놓도록 3D 위치와 서술자를 공동 학습시킨 것이다. 실제 매칭은 두 이미지에서 서로가 서로의 최근접 이웃이 되는 쌍을 고르는 reciprocal matching으로 수행하며, 이를 빠르게 처리하는 전용 알고리즘도 함께 제시했다.

3D 기하와 결합된 매칭이 손에 들어오자 곧바로 SLAM 시스템으로 이어졌다. 2024년 12월 Imperial College London Davison 그룹의 Riku Murai, Eric Dexheimer, Andrew Davison이 공개한 [MASt3R-SLAM](https://arxiv.org/abs/2412.12392)(CVPR 2025)은 MASt3R의 매칭을 frontend로, 그래프 기반 global optimization을 backend로 엮어 RTX 4090 한 장에서 약 15 fps의 실시간 성능을 냈다. 정확한 카메라 캘리브레이션 없이도 dense 매칭이 가능해지면서 foundation 모델의 출력을 COLMAP 형식의 SfM 파이프라인에 잇는 실용적 조합도 자연스레 등장했다. 2025년 6월에는 §20.4에서 살펴볼 VGGT의 공개 저장소에도 예측 결과를 COLMAP 형식으로 내보내고 선택적으로 bundle adjustment를 돌리는 스크립트가 추가됐다.

> 📜 **예언 vs 실제.** DUSt3R는 이미지 쌍을 따로 푼 뒤 global alignment로 사후 정합하는 구조였기에 순차 처리와 실시간 구동은 후속 과제로 남아 있었다. 그러나 DUSt3R 공개 후 불과 8개월 뒤인 2024년 8월에 Spann3R가 global alignment 없이 이미지를 순서대로 처리해 보였고, 12월에는 MASt3R-SLAM이 아예 완결된 실시간 SLAM 시스템으로 묶어 냈다.

> 🔗 **차용.** MASt3R의 3D-anchored matching은 SuperGlue([Sarlin et al. 2020](https://arxiv.org/abs/1911.11763))가 겨냥했던 2D 서술자의 모호성 문제를 다른 각도에서 푼다. SuperGlue가 그래프 신경망으로 이미지 내 주변 맥락을 훑어 모호성을 낮췄다면, MASt3R는 3D 기하 구조를 함께 학습시켜 대응의 애매함을 걷어 냈다.

---

## 20.3 Spann3R — sequential 처리

DUSt3R와 MASt3R는 기본적으로 이미지 묶음을 한데 모아 놓고 쌍을 지어 처리한다. 그러나 카메라가 움직이면서 프레임이 시간 순서대로 한 장씩 들어오는 SLAM에서는 매 프레임마다 지도를 즉시 갱신할 수 있어야 한다.

[Hengyi Wang & Lourdes Agapito 2025. 3D Reconstruction with Spatial Memory (Spann3R, 3DV)](https://arxiv.org/abs/2408.16061)은 DUSt3R를 바로 이 순차 처리 문법에 맞게 개조한 시스템이다. 이미 지나온 프레임들의 기하 정보를 외부 spatial memory에 차곡차곡 저장해 두고, 새 프레임이 도착하면 그 memory를 조회해 공통 좌표계의 pointmap을 곧바로 출력한다. 뒤늦게 모든 쌍을 맞추는 global alignment 최적화가 아예 필요 없어진 것이다.

Spann3R 역시 캘리브레이션 없이 동작하면서 프레임이 들어올 때마다 실시간으로 지도를 키웠다. 논문은 RTX 4090 한 장에서 test-time 최적화 없이 50 fps가 넘는 속도로 점진적 3D 재구성을 수행했다고 보고했다.

> 🔗 **차용.** Spann3R는 DUSt3R의 ViT encoder와 두 decoder 구조를 그대로 물려받았다. 대신 이전에 예측한 pointmap과 decoder의 기하 feature를 가벼운 memory encoder에 통과시켜 memory key와 value로 저장해 두고, 새 프레임에서는 decoder의 기하 feature를 query 삼아 과거 기억을 꺼내 쓴다. 이 memory 관리 방식은 동영상 객체 분할에서 인간의 기억 모델을 본뜬 X-Mem을 참고했다.

---

## 20.4 VGGT — multi-view joint inference

Spann3R가 외부 메모리를 징검다리 삼아 프레임을 한 장씩 넘겨 짚는 동안, Oxford VGG 그룹과 Meta AI의 Jianyuan Wang, Minghao Chen, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, David Novotny는 2025년 초 아예 판을 넓혀 버리는 해법을 제시했다. 수십, 수백 장의 이미지를 한꺼번에 집어넣으면 카메라 파라미터, depth map, point map, 3D point track을 단 한 번의 forward pass로 쏟아 내는 네트워크였다.

[Jianyuan Wang et al. 2025. VGGT: Visual Geometry Grounded Transformer (CVPR)](https://arxiv.org/abs/2503.11651)는 DUSt3R의 쌍 단위 처리를 다중 시점 동시 추론으로 끌어올렸다. DUSt3R로 이미지 $N$장을 재구성하려면 최대 $N(N-1)/2$개에 달하는 이미지 쌍을 일일이 돌린 뒤 무거운 global alignment를 풀어야 했다. VGGT는 $N$장을 통째로 transformer에 넣고 attention 연산으로 모든 시점 사이의 관계를 동시에 조율한다. 논문은 한 번에 수백 장까지 입력할 수 있다고 밝혔으며, H100 GPU 한 장으로 측정한 feature backbone 처리 시간은 20장에 0.31초, 100장에 3.12초에 불과했다.

정량 비교에서도 VGGT는 별도의 후처리 없이 카메라 자세 정확도와 점군 품질 모두 DUSt3R를 앞섰고, global alignment라는 사후 최적화를 덜어 낸 덕분에 속도까지 빨랐다.

---

## 20.5 pose estimation과 reconstruction의 경계

전통적인 컴퓨터 비전에서는 이미 지어진 지도 위에서 내 위치를 찾는 localization과, 미지의 장면을 3D로 깎아 내는 reconstruction을 서로 다른 문제로 떼어 놓고 다뤘다. SLAM은 이 둘을 동시에 맞물려 풀어야 했기에 유독 까다로웠다.

DUSt3R에서 VGGT에 이르는 계보는 이 경계를 흐린다. 기하 복원과 카메라 추정이 같은 학습 표현 위에서 한 몸처럼 돌아가기 때문이다. DUSt3R 단계에서는 아직 pointmap에서 자세를 따로 계산하고 여러 시점을 global alignment로 엮는 뒷손질이 필요했지만, VGGT에 이르러서는 단 한 번의 forward pass 안에서 카메라와 3D 기하가 함께 태어난다.

그렇다고 multi-view geometry의 원리가 증발한 것은 아니다. DUSt3R, MASt3R, VGGT 모두 SfM과 센서 측정으로 만든 다중 시점 정답 데이터를 먹고 자랐으니, 기하 법칙은 네트워크가 흡수한 학습 신호 속에 녹아 있다. 다만 epipolar constraint, triangulation, bundle adjustment 같은 알고리즘이 transformer 가중치 내부에 어떤 형태로 자리 잡았는지는 아직 아무도 증명하지 못했다. 사라진 것은 기하학 자체가 아니라, 사람이 한 줄씩 짠 명시적 파이프라인이다. 그 대가로 DUSt3R가 이상한 3D 구조를 내놓았을 때, 실패의 원인이 가중치 깊은 곳에 숨어 있어 어느 단계가 어긋났는지 짚어 내기는 훨씬 어려워졌다.

> 📜 **예언 vs 실제.** MASt3R 논문은 결론에서 여러 벤치마크의 카메라 자세·localization 성능을 끌어올렸을 뿐 아니라 적은 수의 이미지로도 작동해 localization의 활용 폭을 크게 넓힐 것이라 기대했다. 실제로 같은 해 말 MASt3R의 매칭 모듈은 MASt3R-SLAM의 frontend로 채택되어 실시간 SLAM 시스템 안으로 들어왔다(§20.2).

---

## 20.6 Semantic foundation model을 지도에 쓰기

DUSt3R, MASt3R, VGGT가 점군과 카메라 자세라는 기하의 언어를 혁신하는 동안, 같은 시기의 SLAM 연구에서는 CLIP, DINO, SAM 같은 semantic foundation model을 지도에 이식하려는 시도가 나란히 자라고 있었다. 목표는 분명했다. 사람이 미리 적어 준 class 목록(dictionary) 바깥의 개념까지 지도가 이해하게 만드는 것이다.

계층적 semantic 지도를 실시간으로 쌓아 올린 대표 계보는 MIT Luca Carlone 연구실에 있다. [Nathan Hughes et al. 2022. Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization (RSS)](https://arxiv.org/abs/2201.13360)는 Kimera(Rosinol et al., ICRA 2020)가 만든 metric-semantic mesh 위에 objects → places → rooms → buildings로 이어지는 3D scene graph를 실시간으로 세웠다. 다만 정해진 class만 알아보는 닫힌 집합(closed-set) 분류기를 썼기에, Handbook 16장이 지적하는 *"relatively small (e.g., 100-1000 labels) and predefined dictionary"*의 울타리 안에 갇혀 있었다. 그래도 Hydra는 최초로 실시간 동작을 달성한 spatial perception system이라는 이정표를 세웠다.

이 닫힌 목록의 벽을 허문 것이 foundation model이었다. ETH Zurich와 Google의 [Songyou Peng et al. 2023. OpenScene: 3D Scene Understanding with Open Vocabularies (CVPR)](https://arxiv.org/abs/2211.15654)는 CLIP feature를 3D 점군으로 distillation해, 사용자가 어떤 임의의 단어를 던지더라도 장면 속 점이 그 의미와 얼마나 맞닿아 있는지 짚어 냈다. 이어 Toronto·Montréal·MIT 등이 함께 발표한 [Qiao Gu et al. 2024. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA)](https://arxiv.org/abs/2309.16650)는 단순 class 라벨 대신 VLM이 써 준 설명문을 그래프 노드의 속성으로 달고 객체 사이의 관계까지 LLM으로 서술하게 해, 사전에 등록되지 않은 개념도 3D 그래프에 묶을 수 있게 했다. 나아가 [Dominic Maggio et al. 2024. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs (RA-L)](https://arxiv.org/abs/2404.13696)는 로봇에게 주어진 자연어 임무(task)를 기준으로 삼아, information bottleneck 원리에 따라 임무에 꼭 필요한 추상화 수준만 남기고 무관한 세부는 하나로 뭉개 버리는 과제 중심 scene graph를 구현했다.

한때 2017–2019년 무렵 유행했다가 주류에서 밀려났던 object-as-landmark 연구(→ 22장 §22.4)와 달리, semantic SLAM은 이처럼 기하 지도 위에 얹히는 계층적 scene graph의 모습으로 되살아났다. 심지어 기하 foundation model의 출력에 곧장 open-vocabulary 의미를 실어 나르는 융합도 등장했다. UT Austin과 NVIDIA 등의 [Fan et al. 2024. Large Spatial Model (LSM, NeurIPS)](https://arxiv.org/abs/2410.18956)은 DUSt3R로 초기화한 pointmap 예측 네트워크에 LSeg feature를 결합해, 자세도 모르는 두 장의 이미지만으로 3D 기하·외관·open-vocabulary semantic을 한 번의 forward pass로 동시에 뽑아냈다.

> 🔗 **차용.** Clio는 Carlone 그룹이 Hydra의 실시간 계층 구조에 과제 기반 추상화를 덧입혀 빚은 후속작이며, ConceptGraphs는 독립적인 open-vocabulary 객체 그래프 계보를 연 연구다.

---

## 20.7 SLAM에서 무엇이 남는가

기하와 의미를 거대 모델이 한꺼번에 뱉어 내는 시대에도, 막상 이를 실시간 SLAM 시스템으로 묶으려 들면 고전 SLAM의 뼈대가 다시 고개를 든다. MASt3R-SLAM이 대표적인 예다. MASt3R가 feature matching과 3D 대응의 내부를 완전히 갈아엎었지만, 시스템 차원에서 언제 keyframe을 찍고, 어떻게 loop closure를 찾으며, 지도를 어떻게 갈무리할지 결정하는 문법은 여전히 고전 SLAM에서 빌려 왔다.

앞에서 다룬 학습 기반 지도들 역시 예외가 아니었다. NeRF-SLAM은 지도를 radiance field로 바꾸면서도 keyframe 기반 tracking을 놓지 않았고, 초기 3DGS-SLAM인 MonoGS는 명시적 loop closure가 빠진 자리를 논문의 future work로 남겨 두어야 했다(18장).

2025년 MIT의 Dominic Maggio, Hyungtae Lim, Luca Carlone이 발표한 [VGGT-SLAM](https://arxiv.org/abs/2505.12549)(NeurIPS 2025)은 이 사실을 가장 단적으로 증명한다. VGGT가 지역 submap을 재구성해 내더라도, 시스템 전체가 무너지지 않으려면 이어지는 submap 사이의 15자유도 projective transform을 \(\mathrm{SL}(4)\) 다양체 위에서 최적화하고 loop-closure 제약까지 한데 풀어야 했다. *The SLAM Handbook* 13장(Teed, Deng, Chidlovskii, Revaud, Wimbauer, Cremers)이 DUSt3R를 논하며 내린 결론은 간명하다. *"a form of factor graph is still necessary"* (p.386). Transformer가 지역 기하를 빚어내더라도, 시간을 가로질러 전역 일관성을 붙드는 것은 여전히 그래프 최적화의 몫이다.

---

## 아직 열린 문제

**대규모 sequence 처리.** DUSt3R는 이미지 쌍마다 pointmap을 계산하므로 쌍의 개수가 입력 이미지 수의 제곱으로 늘어난다. VGGT 논문의 비교에서도 DUSt3R는 32장을 넘기자마자 GPU 메모리 한계에 부딪혔다. VGGT 자신은 한 번에 모든 시점을 묶어 H100 한 장에서 200장을 8.75초(최대 메모리 40.63 GB) 만에 처리했지만, 장시간 주행에서 쌓이는 1,000장이나 10,000장의 영상을 그대로 들이밀 수는 없다. Spann3R의 점진적 메모리 방식이 물꼬를 텄고 sparse attention이나 계층적 global alignment 같은 우회로가 모색되고 있지만, 대규모 실외 환경을 거침없이 삼키는 구조는 아직 완성되지 않았다.

**loop closure의 정의.** 고전 SLAM에서 loop closure란 다시 찾은 장소를 식별하고 누적된 drift를 그래프 전역에 퍼뜨려 고치는 절차다. 그러나 DUSt3R 계열의 pointmap 기반 지도에서 "재방문"을 어떻게 정의하고, 교정 신호를 어떤 방식으로 지도 전체에 전파할지는 아직 합의된 원리가 없다. 현재 MASt3R-SLAM조차 loop closure 후보를 찾는 데는 기존 이미지 검색 방식을 그대로 빌려 쓴다.

**metric scale의 일반화.** DUSt3R의 pointmap은 상대적인 깊이 비율은 맞혀도 미터 단위의 절대 크기(scale)는 알지 못한다. Metric3D나 Depth Anything v2가 단안 깊이 추정에서 metric scale 복원에 도전했듯, foundation 3D 역시 다양한 카메라와 환경을 가로질러 절대 scale을 일관되게 맞히는 과제를 안고 있다. 이미지만으로는 절대 크기가 기하적으로 결정되지 않으므로, GPS나 IMU 없이 얻어 내는 scale은 결국 학습한 prior에 기댄 추정일 수밖에 없다.

**SLAM에 쓰기.** 3DGS가 그랬듯 foundation 3D 모델들도 빠르게 SLAM의 심장부로 들어오고 있다. 2024–2025년 MASt3R-SLAM과 VGGT-SLAM은 모두 foundation 모델이 만든 출력을 고전적인 그래프 최적화로 엮어 내는 하이브리드 해법을 택했다. 끝없이 길어지는 영상 스트림을 실시간으로 소화하면서, §20.6에서 본 semantic 연구(Hydra, Clio, ConceptGraphs)까지 하나의 체계로 녹여 내는 일은 23장에서 마주할 다음 장의 질문으로 남는다(→ 23장).

---

이로써 16장에서 20장까지 이어진 학습 기반 SLAM의 흐름은 한 바퀴를 돌았다. 그런데 지금까지 우리가 훑어 온 모든 계보—고전 사진측량부터 MonoSLAM, ORB-SLAM, DSO, 그리고 DUSt3R에 이르기까지—에는 단 하나의 공통점이 있었다. 모두 *카메라*라는 눈으로 빛을 받아 세상을 읽는 이야기였다는 점이다. 같은 시간, 그 바로 옆에서는 빛 대신 레이저 펄스와 전파로 거리를 직접 재는 전혀 다른 연구 공동체가 독자적인 역사를 써 내려가고 있었다.
