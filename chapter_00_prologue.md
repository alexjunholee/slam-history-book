# Ch.0 — SLAM Solved?

2026년, 핸드폰을 들면 AR 레이어가 벽에 달라붙는다. 실내 배송 로봇은 지도를 받지 않고도 주방과 회의실을 구분한다. [DUSt3R](https://arxiv.org/abs/2312.14132) 계열 모델에 사진 몇 장을 던지면 수 초 안에 3D 구조가 나온다. 이제는 데모라기보다 제품이고, 대체로 배경에 가깝다. 그래서 SLAM을 대체로 풀린 문제로 치는 분위기가 있다.

---

2003년으로 돌아가 보면 풍경이 다르다. Andrew Davison은 Imperial College London의 실험실에서 노트북 한 대와 웹캠 한 대로 실시간 3D 추적을 시연했다. [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)이라 불린 그 시스템은 30Hz의 데스크톱 처리 속도에서 한 프레임당 10여 개의 특징만 주시하며 수십 개 규모의 희박한 지도를 유지했다. 한 방의 책상 하나. 카메라가 책상 밖으로 나가면 지도가 발산했다. 그것이 당시 최고였다.

오늘날 핸드폰 AR이 순간 추적하는 특징점 수의 수백분의 일 수준이 당시 최고치였고, 그 격차를 채우는 데 23년이 걸렸다. *어떤 경로*로 채워졌는지가 이 책의 관심사다.

---

SLAM의 역사는 네 가지 서로 다른 전통이 독립적으로 진행되다가 충돌하며 서로를 흡수한 흔적이다. 사진측량학자들은 100년 전에 bundle adjustment를 손으로 풀었다. 로봇공학자들은 1986년 [Smith-Cheeseman](https://arxiv.org/abs/1304.3111)의 확률적 공간관계 프레임부터 지도를 확률의 언어로 다루기 시작했고, 이 문제 설정에 "SLAM"이라는 이름이 붙은 것은 그보다 9년 뒤인 [Durrant-Whyte & Leonard의 1995년 survey](https://ieeexplore.ieee.org/document/476131)에서였다. 컴퓨터 비전 연구자들은 실시간 특징점 추적에 집착했다. 그리고 2020년대의 딥러닝 공동체는 이 모든 것을 단일 네트워크로 흡수하려 시도하고 있다.

이 책이 던지는 질문은 "어떻게"가 아니라 "왜 이런 방식으로"다. EKF 기반 SLAM이 graph-based로 교체된 것은 기술의 자연스러운 진화였는가, 아니면 몇 사람의 선택이 가른 우연이었는가. Feature-based와 direct method의 분기는 처음부터 예견된 것이었는가. 딥러닝이 geometry 파이프라인을 대체하는 속도가 이토록 더딘 이유는 무엇인가. Counterfactual이 의미 있는 질문은, 선택지가 실제로 존재했을 때뿐이다. 이 책은 그 선택지들이 실제로 존재했음을 드러낸다.

---

그 경로를 추적하려면 도구가 필요하다. 연도만 나열하면 연대기가 되고, 기법만 설명하면 교과서가 된다. 이 책은 계보와 예측이라는 두 렌즈로 역사를 읽는다. 어떤 아이디어가 어디서 왔는가. 연구자들이 당시 시점에서 본 미래와 실제로 펼쳐진 미래가 어떻게 갈렸는가.

이 책에는 네 가지 반복 장치가 있다. 각 챕터를 읽을 때 이 장치들을 길잡이로 쓸 수 있다.

**계보 도입**은 챕터 첫 한두 단락에 놓인다. 그 챕터의 주인공이 어떤 지적 유산을 물려받았는지를 인물과 연도로 드러낸다. SLAM의 어떤 아이디어도 진공에서 탄생하지 않았다. 계보를 보면 차용의 지형이 보인다.

**🔗 차용 박스**는 특정 기법이 어디서 왔는지를 한두 문장으로 명시하는 마진 주석이다. "ORB-SLAM의 이 구조는 Strasdat 2011에서 왔다"처럼. 연구자들은 인용하지만 계보를 명시하지 않는 경우가 많다. 이 박스는 그 계보를 드러낸다.

**📜 예언 vs 실제 박스**는 원 논문의 Conclusion·Future Work·Summary 섹션이 짚은 것과 실제로 일어난 일을 대조한다. [Triggs 1999](https://dblp.org/rec/conf/dagstuhl/TriggsMHF99.html)의 BA 종합 논문이 §12 "Summary and Recommendations"에서 대규모 희소 구조 활용을 핵심 지침으로 남긴 자리를, 2010년대 [COLMAP](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)이 수만 장 규모의 SfM을 오픈소스 실전 도구로 만들며 다른 각도에서 채웠다. 예측의 방향은 맞았지만 경로는 달랐다. 연구자가 당시 시점에서 본 미래와 실제 미래의 간극이 이 장치의 대상이다.

**🧭 아직 열린 것**은 챕터 말미에 놓인다. 그 챕터가 다룬 주제에서 2026년 기준 아직 해결되지 않은 항목들이다. SLAM이 풀렸다는 인식 안에 숨어 있는 열린 문제들을 꺼낸다. Ch.19에서 이 항목들을 전 챕터에 걸쳐 수확해 재구성한다.

---

책은 6부로 구성된다.

**1부: 선사시대**는 SLAM이 로봇공학에서 태어나기 이전, 사진측량과 고전 컴퓨터 비전이 쌓아 올린 도구들을 추적한다. 왜 bundle adjustment가 여전히 모든 최적화 backend의 뼈대인가.

**2부: SLAM의 탄생**은 1986년 Smith-Cheeseman의 확률적 프레임부터 Davison의 MonoSLAM까지, 로봇이 처음으로 스스로 지도를 만들기 시작한 시기를 추적한다. 문제 설정은 1986년에 잡혔고, "SLAM"이라는 약어와 표준 용어가 커뮤니티에 정착한 것은 1995년 Durrant-Whyte·Leonard의 survey가 기점이었다. EKF라는 도구가 어떻게 dominant paradigm이 되었고 왜 그 한계가 구조적이었는가.

**3부: 병렬 혁명**은 PTAM이 지도 작성과 카메라 추적을 분리한 2007년부터 graph-based SLAM과 loop closure, 그리고 ORB-SLAM까지를 다룬다. "실시간 SLAM"이 desktop에서 가능해진 10년.

**4부: 방법론의 분기**는 feature-based와 direct method의 갈림길, RGB-D의 등장, place recognition이 독립 서브필드로 분화하는 과정을 다룬다. 서로 다른 가정이 어떻게 서로 다른 생태계를 만들었는가.

**5부: 학습의 유입**은 monocular depth 추정부터 end-to-end SLAM, Neural Radiance Fields, 3D Gaussian Splatting까지를 다룬다. 딥러닝이 geometry 파이프라인을 흡수하는 속도와 그 마찰의 원인.

**6부: 막힌 길과 열린 문제**는 SLAM 역사의 실패한 경로들과, 오늘날 "풀렸다"는 인식 뒤에 남아 있는 구조적 미해결 문제들을 꺼낸다.

---

범위를 정해야 지도가 된다. Foundation model이 SLAM을 대체할 것인가 같은 질문은 이 책의 관심사가 아니다. 과거에 무슨 일이 있었고 왜 그랬는가가 재료다. 당시 제약 조건에서 그 선택이 어떤 의미였는지를 드러내는 것이 목표에 가깝다. homogeneous coordinates, epipolar geometry, EKF 공식은 독자가 이미 안다고 가정한다. 계보의 추적이 이 책의 일이고, 어떤 카메라나 LiDAR를 고를 것인가는 다른 책의 주제다.

수식과 정리·증명까지 체계적으로 짚고 싶다면 [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release)이 있다. Carlone, Kim, Barfoot, Cremers, Dellaert가 편집해 Cambridge University Press에서 2026년에 나온 이 책은 18개 챕터에 SLAM의 현재 이론과 시스템을 총정리한다. 이 책은 그 상태에 이르기까지의 경로를 기록한다.

그 Handbook의 Epilogue에서 편집자 5인이 공동으로 남긴 격언 중 하나는 *"If someone tells you 'SLAM is solved,' don't listen to them"*이다. 이 장의 도입에서 말한 "풀린 문제로 치는 분위기"는 분야 내부의 관찰 대상이지 분야의 합의가 아니다.

---

Davison이 2003년 웹캠 앞에 서 있었을 때, 그는 자신이 무엇을 시작하는지 정확히 알지 못했다. 그 데모 영상은 지금도 인터넷에 남아 있다. 흔들리는 화면, 깜빡이는 랜드마크 점들, 수십 개 규모의 희박한 지도. 거기서 여기까지 오는 사이 어떤 일이 있었는지를 기록한다.

그 기록은 MonoSLAM보다 훨씬 앞에서 시작한다. "SLAM"이라는 약어가 1995년 survey에서 정착하기도 전, 심지어 Smith-Cheeseman이 확률적 지도를 수식으로 쓰기도 전에, 사진측량학자들은 이미 카메라로 3D 구조를 복원하고 있었다. 다음 챕터는 그 선사(先史)를 추적한다.
