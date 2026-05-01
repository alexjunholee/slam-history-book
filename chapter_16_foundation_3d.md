# Ch.16 — Foundation 3D: DUSt3R에서 VGGT까지

Naver Labs Europe의 Philippe Weinzaepfel과 Jerome Revaud는 2022년 CroCo를 발표하면서, 두 이미지가 같은 장면을 찍었다는 사실을 단서 삼아 visual representation을 학습하는 cross-view self-supervised pretraining 방식을 제안했다. 그것은 feature learning 논문처럼 보였다. 1년 뒤 같은 팀이 CroCo의 구조 위에서 calibration 없이 pointmap을 직접 출력하는 시스템을 만들었을 때, DUSt3R는 multi-view geometry 전체를 재정의하는 논문이 되었다. Naver Labs Europe에서 시작한 계보가 Oxford의 VGG 그룹으로 이어지며, 2026년 현재 "SfM이 무엇인가"라는 질문 자체를 다시 쓰고 있다.

---

## 16.1 DUSt3R — learned pointmap

2013년부터 10년간 3D 재건은 동일한 절차를 따랐다. 특징점을 찾고, 매칭하고, 카메라 내부 파라미터와 외부 파라미터를 추정하고, triangulation으로 점군을 만들고, bundle adjustment로 전체를 정제한다. [COLMAP(Schönberger & Frahm, 2016)](https://openaccess.thecvf.com/content_cvpr_2016/html/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.html)이 이 파이프라인의 가장 완성된 형태였다. 오차는 줄었지만 절차의 구조는 바뀌지 않았다.

[Shuzhe Wang et al. 2023. DUSt3R: Geometric 3D Vision Made Easy](https://arxiv.org/abs/2312.14132)는 이 절차를 우회한다. 두 이미지를 입력으로 받아 각 픽셀에 대한 3D 좌표를 직접 출력한다. 내부 파라미터(focal length, principal point)를 요구하지 않는다. pointmap이라 불리는 이 출력은, 이미지 좌표계가 아닌 공통된 3D 공간에서의 좌표다. 카메라가 어떤 렌즈를 달고 있는지 몰라도 된다.

왜 가능한가. DUSt3R의 transformer는 CroCo에서 물려받은 encoder-decoder 구조를 쓴다. 각 이미지는 독립적으로 encoding된 뒤, decoder에서 cross-attention을 통해 다른 이미지의 encoder 출력을 참조한다. self-attention이 단일 이미지 내 픽셀 관계를 처리한다면, cross-attention은 두 이미지 사이의 대응을 암묵적으로 학습한다. 어느 픽셀이 어느 픽셀과 같은 3D 점을 보는지, 이 대응 관계를 대규모 데이터에서 패턴으로 흡수한다. 규칙 코딩이 빠진다. DUSt3R의 훈련 데이터는 MegaDepth, ScanNet, ARKitScenes, BlendedMVS 등 수백만 장의 이미지 쌍이다. ground truth는 COLMAP이 만들었다. 고전 SfM이 학습 시대의 ground truth를 공급한다는 역전이 여기서 일어난다.

> 🔗 **차용.** DUSt3R의 backbone은 ViT([Dosovitskiy et al. 2020](https://arxiv.org/abs/2010.11929))에서 가져온다. 그러나 결정적 발판은 Naver Labs Europe 내부의 선행 작업인 CroCo([Weinzaepfel et al. 2022](https://arxiv.org/abs/2210.10716))다. CroCo는 두 이미지에서 한 쪽의 masking된 영역을 다른 이미지의 정보로 복원하는 cross-view self-supervised pretraining을 제안했다. DUSt3R는 CroCo의 encoder-decoder 구조를 그대로 물려받아 태스크만 "pointmap 예측"으로 바꿨다.

두 이미지에서 pointmap 한 쌍을 얻으면, 카메라 포즈는 이 pointmap들 사이의 rigid alignment로 구한다. Procrustes alignment의 일반화다. pose estimation이 pointmap의 파생물이 된다.

세 장, 열 장의 이미지로 확장할 때 DUSt3R는 global alignment를 푼다. 모든 이미지 쌍의 pointmap을 하나의 공통 좌표계로 정합하는 최적화 문제다. 이때 비로소 bundle adjustment와 유사한 무언가가 등장하지만, 피처 매칭이나 카메라 모델 없이 진행된다.

---

## 16.2 매칭을 삼킨다: MASt3R

DUSt3R의 결과는 novel view synthesis보다 reconstruction에 가깝다. 그런데 재건에서 중요한 서브태스크(두 이미지 사이의 정밀한 픽셀 대응 찾기, 즉 feature matching)를 DUSt3R는 암묵적으로만 처리한다. SuperPoint+SuperGlue, LightGlue가 수행하는 명시적 매칭을 대체하려면 추가 장치가 필요했다.

[Vincent Leroy et al. 2024. Grounding Image Matching in 3D with MASt3R (ECCV)](https://arxiv.org/abs/2406.09756)는 DUSt3R에 matching head를 추가한다. pointmap과 함께 각 픽셀의 feature descriptor를 출력하도록 훈련하되, 3D 위치와 feature가 일관되도록 joint learning한다. 이렇게 나온 feature는 3D 공간에 anchored되어 있다. 매칭은 이 feature descriptor를 nearest neighbor 검색하는 것으로 단순화된다.

> 🔗 **차용.** MASt3R의 3D-anchored matching은 SuperGlue([Sarlin et al. 2020](https://arxiv.org/abs/1911.11763))가 풀려던 문제(2D descriptor의 모호성을 context로 해소)를 다른 방향에서 공략한다. SuperGlue는 그래프 신경망으로 2D 매칭의 모호성을 줄였다. MASt3R는 3D 구조를 직접 학습함으로써 모호성의 원인 자체를 없앤다.

MASt3R 공개 이후 수개월 내에 SLAM 커뮤니티에서 SuperPoint+SuperGlue 조합을 MASt3R로 교체하는 실험이 여러 그룹에서 보고되었다. 2024년 말 [Riku Murai, Eric Dexheimer, Andrew Davison](https://arxiv.org/abs/2412.12392) — Imperial College London의 Davison 그룹 — 이 MASt3R-SLAM을 공개했을 때, 이 시스템은 MASt3R의 매칭을 frontend로, 그래프 기반 global optimization을 backend로 사용했다. 고전적 SLAM 아키텍처의 모양은 유지한 채 내부 부품이 거의 전부 교체된 형태다.

MASt3R의 강점은 ground-truth calibration 없이도 dense 매칭이 가능하다는 점이다. 2026년 현재 COLMAP 기반 SfM 파이프라인에 DUSt3R나 MASt3R를 삽입하는 것이 실험 설정에서 표준화되고 있다.

> 📜 **예언 vs 실제.** DUSt3R 논문 자체는 별도의 "Future Work" 절을 두지 않았지만, pair-wise + global alignment라는 구조 자체가 sequence 처리와 실시간 구동을 다음 과제로 암시한다. Spann3R는 2024년 8월, MASt3R-SLAM은 2024년 말 나왔다. 두 후속 작업이 각각 sequential extension과 SLAM 통합 문제에 6-12개월 내에 응답했다. 이 속도 자체가 이 분야의 이상한 점이다. `[진행형]`

---

## 16.3 Spann3R — sequential 처리

그런데 batch 처리에는 근본적인 제약이 있다. SLAM은 이미지가 미리 다 갖춰지지 않는다.

DUSt3R와 MASt3R는 이미지 집합을 입력받아 일괄 처리한다. 가방 속 이미지들을 한 번에 펼쳐 놓고 정합하는 방식이다. SLAM은 다르다. 이미지가 시간 순서로 들어오고, 시스템은 각 프레임마다 지도를 갱신해야 한다.

[Hengyi Wang & Lourdes Agapito 2024. 3D Reconstruction with Spatial Memory (Spann3R)](https://arxiv.org/abs/2408.16061)는 DUSt3R의 구조를 sequential 처리에 맞게 고친다. 핵심 아이디어는 spatial memory다. 이미 처리한 프레임들의 정보를 memory bank에 저장하고, 새 프레임이 들어올 때 이 메모리에 cross-attention을 수행한다. 새 이미지의 각 픽셀이 과거 프레임의 어떤 정보와 연관되는지 attention이 결정한다.

> 🔗 **차용.** Spann3R의 spatial memory 메커니즘은 concept 면에서 cross-attention memory와 유사하다. 구조적으로 DUSt3R의 사전학습된 ViT encoder-decoder를 그대로 물려받되, 디코더 출력(geometric feature)과 이미지 feature를 결합한 memory key를 두어 appearance와 distance를 동시에 반영한 메모리 조회를 구현한다. DUSt3R가 잡아낸 기하 표현이 그대로 sequential 메모리의 색인으로 재활용되는 경로다.

Spann3R는 calibrated 카메라 없이도 작동하는 DUSt3R의 특성을 그대로 가져간다. 순차 이미지가 들어올 때마다 현재까지의 지도를 점진적으로 갱신한다. 완전한 실시간은 아니지만, DUSt3R의 일괄 처리 방식보다 SLAM 적용에 한 발 더 가깝다.

---

## 16.4 VGGT — multi-view joint inference

Spann3R는 sequential 처리를 가능하게 했다. 그러나 pair-wise pointmap + global alignment라는 DUSt3R의 기본 골격은 그대로였다. Oxford의 VGG 그룹이 들어온다. Jianyuan Wang, Minghao Chen, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, David Novotny가 2025년 초 DUSt3R의 논리를 끝까지 밀어붙였다. 임의의 다중 이미지를 동시에 입력받아, 카메라 포즈와 깊이, 점군을 한 번의 forward pass로 출력한다.

[Jianyuan Wang et al. 2025. VGGT: Visual Geometry Grounded Transformer](https://arxiv.org/abs/2503.11651)는 DUSt3R의 pair-wise 처리를 진정한 multi-view joint inference로 바꿨다. DUSt3R에서 N장의 이미지를 처리하려면 N(N-1)/2쌍의 pointmap을 구한 뒤 global alignment를 풀어야 한다. VGGT는 N장을 한꺼번에 transformer에 통과시킨다. attention이 모든 이미지 쌍 사이의 관계를 동시에 처리한다.

> 🔗 **차용.** VGGT가 COLMAP의 역할을 재정의하는 맥락에서, 더 오래된 계보가 있다. COLMAP 자체가 학습 시대의 ground truth를 만든다는 아이러니는 앞에서 언급했다. 그런데 COLMAP이 실제로 수행하는 일(pair-wise geometry estimation → graph construction → global optimization)의 각 단계가 VGGT 안에서 implicit하게 재현된다. 고전 SfM이 알고리즘으로 구현한 것을 foundation model이 weight로 흡수한 형태다.

DUSt3R와의 정량 비교에서 VGGT는 카메라 포즈 추정 정확도와 점군 품질 면에서 일관된 우위를 보였다. 처리 속도도 global alignment 최적화가 없으므로 더 빠르다. 그리고 여기서 개념적인 문제가 생긴다.

---

## 16.5 pose estimation과 reconstruction의 경계 소멸

전통 컴퓨터 비전은 두 문제를 구분했다. 카메라 포즈 추정은 이미 알려진 지도에서 현재 위치를 찾는 것이고, 3D 재건은 알려지지 않은 환경의 기하를 복원하는 것이다. SLAM은 이 둘을 동시에 풀기 때문에 어려웠다.

DUSt3R부터 VGGT까지의 시스템은 이 구분에 무관심하다. pointmap을 예측하면 포즈가 나오고, 포즈가 나오면 reconstruction이 나온다. "카메라를 먼저 구하고 점군을 나중에" 또는 "점군을 먼저 구하고 카메라를 나중에"라는 순서 자체가 사라진다. 하나의 forward pass가 전부를 동시에 출력한다.

그러면 지금까지 배운 multi-view geometry는 폐기되는가. 그렇지 않다. DUSt3R·MASt3R·VGGT가 잘 작동하는 이유는 epipolar constraint, triangulation, bundle adjustment가 구현하는 기하 원리를 transformer weight 안에서 학습했기 때문이다. 폐기된 것은 명시적 알고리즘 구현 방식이다. 기하 자체는 implicit하게 들어가 살아남았다.

그러나 연구자에게 이것은 실질적 전환이다. Schönberger의 COLMAP 코드를 디버깅하던 방식으로 DUSt3R를 디버깅할 수 없다. 어디서 실패했는지, 왜 실패했는지가 attention weight 안에 묻혀 있다. 해석 가능성 문제가 새로운 형태로 등장한다.

> 📜 **예언 vs 실제.** MASt3R 논문은 결론부를 짧게 맺으며 ground-truth calibration이 없는 매칭이 여러 downstream 태스크에 열려 있다고 시사했다. 명시적 파이프라인 재편 예언은 아니었다. 2026년 현재 여러 photogrammetry 소프트웨어가 DUSt3R/MASt3R를 initialization 단계로 채택하는 것을 평가 중이며, hybrid 삽입의 형태로 자리잡고 있다. `[진행형]`

Naver Labs Europe이라는 한 연구소가 CroCo(2022) → DUSt3R(2023) → MASt3R(2024)의 단계를 2년 내에 밟았다. 이 속도는 특이하다. 한 팀이 pretraining 방법론부터 매칭 시스템까지의 스택을 연속해서 발표했다. 진원지는 Naver Labs Europe이었다. Google Brain, DeepMind, Meta AI가 아니다. Weinzaepfel·Revaud·Leroy를 중심으로 한 소규모 팀의 집중적 투자가 만든 결과다. SLAM 단계로 옮기는 일은 Imperial College London의 Davison 그룹(MASt3R-SLAM)이 이어받았다.

---

## 16.6 다른 갈래 — semantic foundation이 지도로 들어온다

지금까지의 서술은 geometric foundation이다. DUSt3R·MASt3R·VGGT는 pointmap·카메라 포즈·기하 구조를 다룬다. 그런데 2022년 전후로 "foundation 3D"라는 단어가 SLAM 문헌에서 두 갈래로 쓰이기 시작했다. 한쪽은 Naver Labs Europe에서 출발한 geometric 계보고, 다른 쪽은 CLIP·DINO·SAM을 지도 안으로 끌어들이는 semantic 계보다. 전자는 calibration을 없앴고, 후자는 dictionary를 없앴다.

semantic 갈래의 시작은 MIT의 Luca Carlone 그룹에서 나왔다. [Nathan Hughes et al. 2022. Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization](https://arxiv.org/abs/2201.13360)는 Kimera(Rosinol 2020)의 metric-semantic mesh 위에 objects → places → rooms → buildings의 hierarchical scene graph를 online으로 얹었다. closed-set 분류기를 쓰는 한 handbook이 "100-1000 labels predefined dictionary"라고 못 박은 제약 안이었지만, Hydra는 hierarchical map이 실시간으로 굴러간다는 것을 처음 보여줬다.

dictionary의 벽은 foundation model이 허물었다. [Songyou Peng et al. 2023. OpenScene: 3D Scene Understanding with Open Vocabularies (CVPR)](https://arxiv.org/abs/2211.15654)가 ETH/Pollefeys 그룹에서, 곧이어 [Qiao Gu et al. 2024. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA)](https://arxiv.org/abs/2309.16650)가 Montréal-MIT 협업으로 발표됐다. OpenScene은 CLIP feature를 3D 점군에 distillation해서 "이 점은 의자와 얼마나 가까운가"를 자연어 질의로 풀 수 있게 했다. ConceptGraphs는 한 걸음 더 나아갔다. class label 대신 VLM이 생성한 language description을 node attribute로 달고, object 사이 관계를 LLM이 서술한다. Hydra의 hierarchical 구조에 Peng의 open-vocabulary feature가 결합되면서 scene graph는 사전에 없는 개념까지 수용하게 됐다.

[Dominic Maggio et al. 2024. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs](https://arxiv.org/abs/2404.13696)는 이 계보를 task 쪽으로 돌렸다. 로봇이 받은 자연어 task를 information bottleneck으로 해석해서, 그 task에 필요한 추상화 수준만 scene graph에 남긴다. "커피 머신 근처 청소"라는 지시에서 커피 머신과 그 주변 객체는 보존되고, 무관한 디테일은 묶인다. hierarchical graph의 어느 층을 노출할지가 task에 따라 달라지는 것이다.

> 🔗 **차용.** ConceptGraphs와 Clio의 계보는 Hydra의 hierarchical 구조를 그대로 물려받는다. Carlone 그룹의 scene graph 정의(Armeni → Rosinol-Kimera → Hughes-Hydra → Maggio-Clio)가 8년에 걸쳐 누적된 뒤, 그 위에 CLIP·VLM·LLM이 얹혔다. "표현이 바뀌어도 구조는 살아남는다"는 5부 전체의 패턴이 여기서도 성립한다. 바뀐 것은 node에 붙는 feature고, 살아남은 것은 objects-places-rooms라는 계층 자체다.

지도에 semantic을 싣는 문제는 Ch.18 §18.4가 2017-2019년 object-as-landmark 계보의 시장 축소를 짚은 뒤 이어지는 별개 궤적이다. semantic SLAM이 hierarchical scene graph라는 형태로 귀환했다는 사실, 그리고 geometric foundation(DUSt3R 계보)과 semantic foundation(Hydra → ConceptGraphs → Clio 계보)이 2026년 현재 아직 본격적으로 만나지 않았다는 사실이 기록할 만하다. VGGT의 pointmap에 CLIP feature를 붙인 end-to-end 시스템, 또는 Clio의 scene graph에 DUSt3R의 calibration-free 기하를 결합한 구성은 아직 보고되지 않았다. 만남의 지점은 Ch.19의 열린 문제로 넘긴다.

---

## 16.7 SLAM에서 무엇이 남는가

MASt3R-SLAM이 고전 SLAM의 아키텍처를 빌려 쓴다는 점은 흥미롭다. keyframe 선택과 loop closure, map management, 이 구조들이 새 표현 위에서도 그대로 필요했다. DUSt3R 계열이 feature matching과 reconstruction의 내부를 교체했지만, SLAM 시스템 수준의 판단들은 고전 방법이 해결한 방식 그대로 재사용한다.

이 관찰은 5부 전체에 걸쳐 반복되는 패턴과 일치한다. NeRF-SLAM이 NeRF를 map 표현으로 채택하면서도 keyframe 기반 tracking을 유지했다. 3DGS-SLAM이 Gaussian을 채택하면서도 loop closure를 classical 방식으로 했다 (Ch.15). Ch.15b의 dynamic SLAM도 mask 제거라는 front-end만 교체했을 뿐 back-end는 그대로였다. 표현(representation)을 바꿔도 시스템 구조는 그대로 살아남는다.

Foundation 3D의 경우에도 이 패턴이 반복된다. 2025년 MIT의 Dominic Maggio와 Luca Carlone이 [VGGT-SLAM](https://arxiv.org/abs/2505.12549)을 공개했다. VGGT가 local submap을 재건하면 factor graph가 그것들을 global 좌표계로 엮는 구성이다. transformer가 기하를 흡수했지만 factor graph는 살아남았다. Revaud 본인도 Handbook Ch.13에서 "a form of factor graph is still necessary"라고 적었다. 흡수의 속도는 특이하지만 최종 형태는 여전히 열려 있다. 실시간 대규모 sequence를 foundation 3D가 어디까지 감당할지, 그리고 16.6에서 언급한 semantic 갈래와 어디서 합류할지는 2026-2027년의 관찰 대상이다.

---

## 🧭 아직 열린 것

**대규모 sequence 처리의 벽.** DUSt3R와 VGGT의 transformer는 이미지 수에 quadratic하게 메모리를 요구한다. 100장까지는 현실적이지만 1,000장, 10,000장은 다른 문제다. Spann3R의 incremental 방식이 partial answer지만, 대규모 outdoor 환경의 매끄러운 처리는 미해결이다. 누가 지금 붙어 있는가. 여러 그룹이 sparse attention, hierarchical global alignment를 탐색하고 있으나 합의된 방법이 없다.

**Loop closure를 이 프레임에서 어떻게 정의하는가.** 고전 SLAM에서 loop closure는 이전에 방문한 장소를 인식하고 누적 오차를 교정하는 메커니즘이다. DUSt3R 계열에서 "이전에 방문한 장소"를 어떻게 표현하고, pointmap 기반 지도에서 교정을 어떻게 propagate하는가. MASt3R-SLAM이 기존 방식으로 처리하지만, 이것이 최선인지 원리적 해법인지 알 수 없다.

**Metric scale의 일반화.** DUSt3R의 pointmap은 relative scale이다. 두 이미지 사이의 깊이 비율은 복원하지만 절대 스케일은 모른다. Metric3D나 Depth Anything v2가 metric depth를 목표로 했듯, foundation 3D에서도 metric scale을 일반화하는 문제가 남는다. 카메라 독립적 metric은 foundation 규모에서도 쉽지 않다. GPS나 IMU 없이 absolute scale을 결정하는 물리적 제약은 데이터 규모와 무관하게 존재한다.

**이 흐름이 SLAM의 미래인가, 별개 갈래인가.** 15장의 3DGS처럼 foundation 3D도 SLAM 커뮤니티가 흡수하는 중이다. MASt3R-SLAM과 VGGT-SLAM이 2024-2025년에 연달아 등장하며 흡수의 경로는 윤곽이 잡혔다. 그러나 실시간 대규모 sequence 구동, 그리고 §16.6이 짚은 semantic 갈래(Hydra → ConceptGraphs → Clio)와의 합류 지점은 여전히 불분명하다. geometric foundation과 semantic foundation이 한 시스템에서 만나는 형태는 Ch.19 열린 문제의 핵심 축이다.

---

5부의 세 챕터가 같은 결론에 도달한다. NeRF든 foundation model이든, 표현을 바꾸면 reconstruction과 localization의 내부가 바뀐다. 그러나 SLAM 시스템 수준의 구조(keyframe, loop closure, map management)는 새 표현 위에서도 살아남는다. 6부는 이 패턴의 경계 밖을 본다. Ch.17은 카메라가 아닌 LiDAR를 중심에 둔 평행한 발전 궤적으로 넘어간다. 같은 시기에 같은 문제를 다른 센서와 다른 문화로 풀었던 계보다.
