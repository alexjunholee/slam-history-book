# Ch.0 — SLAM Solved?

2026년, 핸드폰을 들면 AR 레이어가 벽에 달라붙는다. 실내 배송 로봇은 지도를 받지 않고도 주방과 회의실을 구분한다. [DUSt3R](https://arxiv.org/abs/2312.14132) 계열 모델에 사진 몇 장을 던지면 수 초 안에 3D 구조가 나온다. 이제는 데모라기보다 제품이고, 대체로 배경 기술에 가깝다. 그래서 SLAM을 대체로 풀린 문제로 치는 분위기가 있다.

---

2003년으로 돌아가 보면 풍경이 다르다. Andrew Davison은 Imperial College London의 실험실에서 노트북 한 대와 웹캠 한 대로 실시간 3D 추적을 시연했다. [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)이라 불린 그 시스템은 30Hz의 데스크톱 처리 속도에서 한 프레임당 10여 개의 특징만 주시하며 수십 개 규모의 희박한 지도를 유지했다. 방 하나, 책상 하나였다. 카메라가 책상 밖으로 나가면 지도가 발산했다. 그것이 당시 실시간 단안 SLAM의 대표적인 규모와 한계였다.

당시 시스템이 한 프레임에서 추적한 특징은 오늘날 핸드폰 AR보다 훨씬 적었다. 2003년의 추적 시스템은 *어떤 경로*를 거쳐 2026년의 AR로 이어졌는가.

---

SLAM의 역사는 네 가지 서로 다른 전통이 독립적으로 진행되다가 충돌하며 서로를 흡수한 흔적이다. 사진측량학자들은 20세기 전반부터 여러 사진의 광선 다발을 함께 조정했다. 로봇공학자들은 1986년 [Smith-Cheeseman](https://arxiv.org/abs/1304.3111)의 확률적 공간관계 프레임을 거치며 지도를 확률의 언어로 다루기 시작했다. Durrant-Whyte와 Bailey의 [후대 역사 정리](https://www-personal.acfr.usyd.edu.au/tbailey/publications/slamtutorial1.htm)는 "SLAM"이라는 약어가 1995년 ISSRR 워크숍에서 채택됐다고 기록한다. 컴퓨터 비전 연구자들은 실시간 특징점 추적을 발전시켰고, 2020년대의 딥러닝 공동체는 이 구성 요소들을 학습된 모델 안에 다시 배치하고 있다.

EKF 기반 SLAM이 graph-based로 교체된 것은 기술의 자연스러운 진화였는가, 아니면 몇 사람의 선택이 가른 우연이었는가. Feature-based와 direct method의 분기는 처음부터 예견된 것이었는가. 딥러닝이 geometry 파이프라인을 대체하는 속도가 이토록 더딘 이유는 무엇인가. Counterfactual은 선택지가 실제로 존재했을 때만 의미가 있다. 그 선택지들은 실제로 존재했다.

---

그 경로를 추적하려면 도구가 필요하다. 연도만 나열하면 연대기가 되고, 기법만 설명하면 교과서가 된다. 이 책은 계보와 예측이라는 두 렌즈로 역사를 읽는다. 어떤 아이디어가 어디서 왔는가. 연구자들이 당시 시점에서 본 미래와 실제로 펼쳐진 미래가 어떻게 갈렸는가.

이 책에는 네 가지 반복 장치가 있다. 각 챕터를 읽을 때 이 장치들을 길잡이로 쓸 수 있다.

**계보 도입**은 챕터 첫 한두 단락에 놓인다. 그 챕터의 주인공이 어떤 지적 유산을 물려받았는지를 인물과 연도로 드러낸다. SLAM의 어떤 아이디어도 진공에서 탄생하지 않았다. 계보를 보면 차용의 지형이 보인다.

**🔗 차용 박스**는 특정 기법이 어디서 왔는지를 한두 문장으로 명시하는 마진 주석이다. "ORB-SLAM의 이 구조는 Strasdat 2011에서 왔다"처럼. 연구자들은 인용하지만 계보를 명시하지 않는 경우가 많다. 이 박스는 그 계보를 드러낸다.

**📜 예언 vs 실제 박스**는 원 논문의 Conclusion·Future Work·Summary 섹션이 짚은 것과 실제로 일어난 일을 대조한다. [Triggs 1999](https://dblp.org/rec/conf/dagstuhl/TriggsMHF99.html)의 BA 종합 논문이 §12 "Summary and Recommendations"에서 대규모 희소 구조 활용을 핵심 지침으로 남긴 자리를, 2010년대 [COLMAP](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)이 수만 장 규모의 SfM을 오픈소스 실전 도구로 만들며 다른 각도에서 채웠다. 예측의 방향은 맞았지만 경로는 달랐다. 연구자가 당시 시점에서 본 미래와 실제 미래의 간극이 이 장치의 대상이다.

**🧭 아직 열린 것**은 챕터 말미에 놓인다. 그 챕터가 다룬 주제에서 2026년 기준 아직 해결되지 않은 항목들이다. SLAM이 풀렸다는 인식 안에 숨어 있는 열린 문제들을 꺼낸다. Ch.19에서 이 항목들을 전 챕터에 걸쳐 수확해 재구성한다.

---

책은 6부로 구성된다.

**1부: 선사시대**는 SLAM이 로봇공학에서 태어나기 이전, 사진측량과 고전 컴퓨터 비전이 쌓아 올린 도구들을 추적한다. 왜 bundle adjustment가 여전히 모든 최적화 backend의 뼈대인가?

**2부: 고전 SLAM**은 확률적 지도 구축과 EKF의 한계에서 MonoSLAM·PTAM, graph-based SLAM과 최적성 인증까지를 다룬다. 지도와 추정을 함께 유지하는 문제는 어떻게 필터에서 최적화로 확장되었는가?

**3부: 성숙기**는 ORB-SLAM과 관성 사전적분·연속시간 추정, direct method, RGB-D, place recognition을 다룬다. 초기 실시간 시연을 넘어 더 넓은 환경과 다양한 센서로 확장한 과정을 추적한다.

**4부: 러닝 융합기**는 monocular depth 추정, end-to-end SLAM, 기하와 학습을 결합한 하이브리드 방법을 다룬다. 학습은 기존 파이프라인의 어느 계산을 맡았고, 기하학적 제약은 어디에 남았는가?

**5부: 표현의 혁명**은 Neural Radiance Fields와 3D Gaussian Splatting, 동적 장면 표현, 3D foundation model을 다룬다. 지도와 장면을 표현하는 방식이 바뀌면서 추적과 재구성의 관계도 달라졌다.

**6부: 막힌 길과 열린 문제**는 SLAM 역사의 실패한 경로들과, 오늘날 "풀렸다"는 인식 뒤에 남아 있는 구조적 미해결 문제들을 꺼낸다.

---

범위를 정해야 지도가 된다. Foundation model과 SLAM의 관계도 현재 연구가 남긴 열린 질문으로 다루되, 미래를 단정하지 않는다. 과거에 무슨 일이 있었고 왜 그랬는가가 재료다. 당시 제약 조건에서 그 선택이 어떤 의미였는지를 드러내는 것이 목표에 가깝다. homogeneous coordinates, epipolar geometry, EKF 공식은 독자가 이미 안다고 가정한다. 계보의 추적이 이 책의 일이고, 어떤 카메라나 LiDAR를 고를 것인가는 다른 책의 주제다.

수식과 정리·증명까지 체계적으로 짚고 싶다면 [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release)이 있다. Carlone, Kim, Barfoot, Cremers, Dellaert가 편집해 Cambridge University Press에서 2026년에 나온 이 책은 18개 챕터에 SLAM의 현재 이론과 시스템을 총정리한다. 이 역사서는 그 상태에 이르기까지의 경로를 기록한다.

그 Handbook의 Epilogue에서 편집자 5인이 공동으로 남긴 격언 중 하나는 *"If someone tells you 'SLAM is solved,' don't listen to them"*이다. "풀린 문제로 치는 분위기"는 분야 내부의 관찰 대상이지 분야의 합의가 아니다.

---

Davison의 2003년 데모 영상은 지금도 인터넷에 남아 있다. 흔들리는 화면, 깜빡이는 랜드마크 점들, 수십 개 규모의 희박한 지도. 거기서 여기까지 오는 사이에는 어떤 일이 있었는가.

그 기록은 MonoSLAM보다 훨씬 앞에서 시작한다. "SLAM"이라는 약어가 1990년대에 정착하기 전, 심지어 Smith-Cheeseman이 확률적 지도를 수식으로 쓰기 전부터 사진측량학자들은 카메라로 3D 구조를 복원하고 있었다. 그 선사(先史)는 사진측량에서 시작한다.
