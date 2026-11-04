# Ch.0 — SLAM Solved?

2026년에는 핸드폰 카메라로 방을 비추면 AR 화면에 띄운 가상 물체가 벽에 붙은 채 움직이지 않는다. 실내 배송 로봇은 미리 받은 지도 없이도 주방과 회의실을 구분한다. [DUSt3R](https://arxiv.org/abs/2312.14132)와 그 후속 모델에 사진 몇 장을 넣으면 몇 초 안에 3D 구조가 나온다. 이런 기능은 이미 제품에 들어가 있고, 그 뒤에서 SLAM이 사용자 눈에 띄지 않게 돈다. 그래서 SLAM을 거의 풀린 문제로 여기는 분위기가 있다.

---

2003년에는 사정이 달랐다. 당시 Oxford 대학 공학과의 Robotics Research Group에 있던 Andrew Davison은 손에 든 카메라 한 대로 카메라 위치를 실시간으로 추정하면서 지도를 만들어 보였다. ICCV 2003에 발표한 이 시스템은 나중에 [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)이라는 이름으로 알려졌다. 논문에 따르면 시스템은 2.2 GHz Pentium 데스크톱 PC에서 1초에 30프레임을 처리했다. 어느 위치에서든 보이는 특징이 6~10개쯤 되도록 지도를 관리했고, 논문에 실린 20초 실행에서 만든 지도에는 특징이 60개쯤 있었다. 실험은 방 안 책상 주변에서 했다. 논문은 이 범위에서는 EKF의 비선형성 문제를 겪지 않았지만 카메라가 처음 좌표계에서 멀리 움직이면 문제가 커질 수 있다고 적었다. 당시 실시간 단안 SLAM이 다룰 수 있는 규모가 그 정도였다.

책상 주변에서 특징 몇 개를 따라가던 시스템이 어떻게 2026년의 핸드폰 AR까지 왔을까?

---

SLAM에는 따로 발전하던 여러 분야의 연구가 이어져 있다. 사진측량학자들은 1950년대 후반부터 여러 사진의 광선 다발을 한꺼번에 조정했다. Smith, Self, Cheeseman은 1986년 UAI 학회에서 발표하고 1988년 논문집에 실은 [논문](https://arxiv.org/abs/1304.3111)에서 물체 사이 공간 관계의 불확실성을 확률로 표현했다. 이때부터 로봇공학자들은 지도를 확률 분포로 다루기 시작했다. Durrant-Whyte와 Bailey가 2006년에 쓴 [튜토리얼](https://www-personal.acfr.usyd.edu.au/tbailey/publications/slamtutorial1.htm)에 따르면 "SLAM"이라는 약어는 1995년 International Symposium on Robotics Research(ISRR)에서 발표된 서베이 논문에 처음 나왔다. 컴퓨터 비전 연구자들은 특징점을 실시간으로 추적하는 기법을 다듬었다. 2020년대에는 딥러닝 연구자들이 이 구성 요소들을 학습 모델 안에 다시 담고 있다.

EKF(extended Kalman filter, 확장 칼만 필터) 기반 SLAM이 graph-based SLAM으로 바뀔 때 기술 발전과 몇몇 연구자의 선택은 각각 얼마나 작용했을까? Feature-based 방법과 direct 방법이 나뉘리라는 것을 처음부터 내다본 사람이 있었을까? 딥러닝은 왜 기하 계산으로 짠 파이프라인을 이렇게 더디게 대체하고 있을까? 이 책은 당시 연구자들 앞에 실제로 있던 선택지를 근거로, 다른 선택을 했다면 어떻게 됐을지(반사실 가정, counterfactual) 따져 본다.

---

사진측량에서 시작해 SLAM으로 이어진 아이디어가 어디서 왔는지, 연구자들이 당시 내다본 미래와 실제로 벌어진 일이 어떻게 달랐는지를 장마다 따라간다. 예를 들어 [Triggs 등(1999)](https://dblp.org/rec/conf/dagstuhl/TriggsMHF99.html)은 bundle adjustment를 정리한 논문의 마지막 절 "Summary and Recommendations"에서 문제 구조를 활용하는 일이 아주 중요하다고 쓰고, 배치(batch) 문제에는 몇 가지 예외를 두고 희소 분해를 쓰는 Gauss-Newton 방법을 권했다. [COLMAP(CVPR 2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)은 SfM(structure from motion, 여러 사진으로 카메라 위치와 3D 구조를 함께 복원하는 방법)을 입력 사진이 7만 장 넘는 문제까지 시험했고, 전체 구현을 오픈소스로 공개했다. 본문의 🔗 상자에는 어떤 기법이 어느 논문에서 왔는지 적었다. 📜 상자에서는 원 논문의 결론과 future work를 실제로 일어난 일과 견주어 본다. 장 끝의 🧭 항목에는 2026년에도 풀리지 않은 문제를 적었고, Ch.19에서 이 항목들을 모아 다시 다룬다.

---

이 책은 여섯 부로 나뉜다.

**1부: 선사시대**는 SLAM이 로봇공학에서 나오기 전에 사진측량과 고전 컴퓨터 비전이 만든 도구를 다룬다. bundle adjustment는 왜 지금도 최적화 backend의 바탕일까?

**2부: 고전 SLAM**은 확률적 지도 작성과 EKF의 한계에서 시작해 MonoSLAM·PTAM, graph-based SLAM, 최적해 인증까지 다룬다. 지도와 위치를 함께 추정하는 방법은 어떻게 필터에서 최적화로 옮겨 갔을까?

**3부: 성숙기**는 ORB-SLAM과 관성 사전적분·연속시간 추정, direct method, RGB-D, place recognition을 다룬다. 초기 실시간 시연 뒤에 SLAM이 더 넓은 환경과 여러 센서로 쓰임새를 넓힌 과정이다.

**4부: 러닝 융합기**는 monocular depth 추정, end-to-end SLAM, 기하와 학습을 결합한 하이브리드 방법을 다룬다. 학습한 모델은 기존 파이프라인의 어느 계산을 맡았고, 기하 제약은 어디에 남았을까?

**5부: 표현의 혁명**은 Neural Radiance Fields와 3D Gaussian Splatting, 동적 장면 표현, 3D foundation model을 다룬다. 지도와 장면을 표현하는 방식이 바뀌자 추적과 재구성의 관계도 달라졌다.

**6부: 막힌 길과 열린 문제**는 SLAM 역사에서 성과 없이 끝난 시도와, "풀렸다"는 인식과 달리 아직 남은 문제를 다룬다.

---

Foundation model과 SLAM의 관계는 아직 연구가 진행 중이어서 이 책에서는 결론을 내리지 않는다. 이 책은 과거에 무슨 일이 있었고 왜 그랬는지, 당시 제약 아래에서 그 선택이 어떤 뜻이었는지를 쓴다. 독자가 동차 좌표(homogeneous coordinates), epipolar geometry, EKF 공식은 이미 안다고 보고 썼다. 카메라나 LiDAR를 고르는 방법은 다루지 않는다.

수식과 정리·증명을 체계적으로 보려면 [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release)을 읽으면 된다. Carlone, Kim, Barfoot, Cremers, Dellaert가 편집했고 Cambridge University Press에서 나온다(서지 정보상 2026년). 18개 장에 SLAM의 이론과 시스템을 정리했다. 이 역사서는 SLAM이 그렇게 정리되기까지 연구자들이 거친 과정을 기록한다.

Handbook 저자들은 맺음말(Epilogue)에 독자에게 남기는 말을 모아 실었고, 그 가운데 하나가 *"If someone tells you 'SLAM is solved,' don't listen to them"*이다. 이 책도 앞에서 말한 "거의 풀린 문제"라는 인식을 분야 안에 퍼진 분위기로 다룬다.

---

Davison의 2003년 데모 영상은 지금도 인터넷에서 볼 수 있다. 화면이 흔들리고, 랜드마크 점이 깜빡이고, 지도에는 점이 수십 개뿐이다.

이야기는 MonoSLAM보다 훨씬 앞에서 시작한다. "SLAM"이라는 약어가 1990년대에 자리 잡기 전, Smith, Self, Cheeseman이 확률적 지도를 수식으로 쓰기도 전에 사진측량학자들은 사진으로 3D 구조를 복원하고 있었다.
