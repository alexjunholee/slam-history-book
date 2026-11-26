# 3장 SfM: Longuet-Higgins에서 COLMAP까지

Harris와 Lowe가 이미지 속에서 쫓을 만한 점을 골라내는 기술을 벼리는 동안, 또 다른 연구자 집단은 전혀 다른 방향에서 같은 산을 오르고 있었다. 이미 확보된 대응점들이 있을 때, 사진 두 장만으로 카메라가 어떻게 움직였고 장면의 3D 구조가 어떤 모양인지 어떻게 역산할 수 있을까? 특징을 *검출*하는 연구와 특징으로 *공간을 재구성*하는 연구는 한동안 각자의 트랙을 달리다, 2000년대 중반에 이르러 비로소 하나의 거대한 3D 복원 파이프라인으로 합류했다.

현대 컴퓨터 비전의 다중 시점 기하(two-view geometry)에서는 1981년 *Nature*에 실린 세 쪽짜리 짧은 논문이 출발선으로 자주 꼽힌다. 서식스 대학교 실험심리학 연구실의 H.C. Longuet-Higgins는 "[A Computer Algorithm for Reconstructing a Scene from Two Projections](https://cseweb.ucsd.edu/classes/fa01/cse291/hclh/SceneReconstruction.pdf)"에서 두 사진 속 같은 점 여덟 개의 좌표만 쥐여 주면 단순한 일차 연립방정식을 풀어 두 시점의 상대 자세와 장면의 3D 구조를 동시에 구할 수 있음을 증명했다. 사진측량이나 인간 운동 지각(motion perception) 문헌에 선행 연구가 없지 않았지만, 컴퓨터가 대수적으로 두 영상을 엮어 3차원을 복원하는 현대적 프레임은 바로 이 세 쪽에서 시동이 걸렸다.

---

## 3.1 Essential Matrix와 8-point Algorithm

Longuet-Higgins의 통찰은 명쾌했다. 두 대의 카메라가 같은 3차원 점을 바라보면, 양쪽 이미지 좌표 사이에는 반드시 만족해야 할 대수적 제약이 걸린다. 카메라 내부 파라미터로 좌표를 정규화하면, 이 기하적 관계는 단 하나의 $3\times 3$ 행렬로 압축된다. 논문에서 $Q$라고 적었던 이 행렬이 오늘날 우리가 **essential matrix** $\mathbf{E}$라 부르는 대상이다.

서로 대응하는 두 정규화 좌표를 $\mathbf{x}_1$, $\mathbf{x}_2$라 하면 다음의 **epipolar constraint**가 성립한다.

$$\mathbf{x}_2^\top \mathbf{E} \mathbf{x}_1 = 0$$

$\mathbf{E}$는 두 카메라 사이의 회전 $\mathbf{R}$과 이동 $\mathbf{t}$를 $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$ 형태로 품고 있다(여기서 $[\mathbf{t}]_\times$는 $\mathbf{t}$의 skew-symmetric 행렬이다).

스케일의 모호성을 제하면 $\mathbf{E}$의 실제 자유도는 5다. 따라서 이론상 대응점 5쌍만 있으면 해를 정할 수 있고, 실제로 사진측량에서는 오래전부터 점 5개로 푸는 방법을 알고 있었다. 문제는 5개만으로 해를 구하려면 3차 연립방정식 다섯 개를 반복 계산으로 풀어야 해 연산이 까다롭고 해의 후보도 여러 개로 갈린다는 점이었다. 훗날 David Nistér가 효율적인 비선형 [5-point algorithm](http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/2004-PAMI-David%20Nister-An%20Efficient%20Solution%20to%20the%20Five-Point%20Relative%20Pose%20Problem.pdf)(2004)을 완성하기 전까지, 연구자들에게는 빠르고 단순한 선형 해법이 필요했다. Longuet-Higgins는 대담한 타협을 택했다. $3\times 3$ 행렬의 원소 9개 중 스케일 하나를 고정해 미지수를 8개로 보고, rank-2 같은 비선형 기하 제약은 일단 미뤄 두었다. 그렇게 대응점 8쌍을 모아 일차 연립방정식으로 $\mathbf{E}$를 먼저 푼 뒤 사후에 제약을 맞추는 전략을 세웠다. 구현이 놀라울 만큼 간단하고 계산도 가벼웠던 이 **8-point algorithm**은 순식간에 널리 퍼졌다.

물론 대가도 있었다. 픽셀 좌표는 수백에서 수천 단위에 이르는 반면 동차 좌표의 마지막 원소는 1이기 때문에, 좌표값 그대로 연립방정식 행렬을 쌓으면 원소 크기가 몇 자릿수씩 벌어져 SVD(특이값 분해) 결과가 노이즈에 몹시 취약해졌다.

한편 내부 파라미터 $\mathbf{K}$를 모르는 카메라에는 essential matrix를 일반화한 **fundamental matrix** $\mathbf{F}$가 쓰인다. $\mathbf{K}_1, \mathbf{K}_2$를 각각 두 카메라의 내부 행렬이라 할 때 $\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \mathbf{K}_1^{-1}$(같은 카메라라면 $\mathbf{F} = \mathbf{K}^{-\top} \mathbf{E} \mathbf{K}^{-1}$)로 정의되며, 여전히 $\mathbf{x}_2^\top \mathbf{F} \mathbf{x}_1 = 0$을 만족한다. 이 덕분에 현대 SfM 파이프라인은 $\mathbf{K}$를 알면 $\mathbf{E}$를 바로 풀고, 모르면 $\mathbf{F}$부터 추정해 들어가는 유연함을 얻었다.

> 🔗 **차용.** Richard Hartley는 8-point algorithm의 수치 불안정성을 단숨에 바로잡았다. 1997년 논문 ["In Defense of the Eight-Point Algorithm"](https://www.cse.unr.edu/~bebis/CS485/Handouts/hartley.pdf)에서 그는 이미지마다 점들의 무게중심을 원점으로 옮기고 원점까지의 평균 거리가 $\sqrt{2}$가 되도록 스케일을 맞춘 뒤 $\mathbf{F}$를 풀고, 마지막에 원래 좌표계로 되돌리는 절차를 제안했다. 이 절차는 카메라 내부 파라미터로 좌표를 정규화하는 것과는 목적이 다른 순수한 수치 조건 개선이었지만, Longuet-Higgins의 기하를 건드리지 않고도 성능을 극적으로 끌어올렸다. 덕분에 이 정규화 8-point 방법은 다중 시점 기하 교과서의 불변의 표준이 됐다.

---

## 3.2 Tomasi-Kanade Factorization

Longuet-Higgins 이후 약 10년간 SfM 연구는 주로 사진 두 장 사이의 기하를 파고드는 데 머물렀다. 판을 흔든 것은 1992년 *IJCV*에 실린 Carlo Tomasi(Cornell)와 Takeo Kanade(CMU)의 **[factorization method](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/TomasiC_Shape%20and%20motion%20from%20image%20streams%20under%20orthography.pdf)**였다. 이들은 두 장씩 끊어 보는 대신 연속된 수많은 프레임을 하나의 큰 행렬에 담아 한꺼번에 풀어 버리는 발상을 내놓았다.

프레임 $F$장에서 점 $P$개를 끝까지 추적하면, 그 이미지 좌표들을 $2F \times P$ 크기의 관측 행렬 $\mathbf{W}$에 차곡차곡 쌓을 수 있다. 각 행에서 그 프레임의 점 좌표 평균을 빼 이동 성분을 지우면, 정사영(orthographic) 카메라 모델에서는 놀랍게도 $\mathbf{W}$의 rank가 항상 3 이하가 된다. 논문에서 rank theorem이라 부른 이 성질 덕분에 $\mathbf{W}$는 두 행렬의 곱으로 깔끔하게 쪼개진다.

$$\mathbf{W} = \mathbf{M} \mathbf{S}$$

여기서 $\mathbf{M}$은 $2F \times 3$ 모션 행렬, $\mathbf{S}$는 $3 \times P$ 구조 행렬이다. $\mathbf{W}$를 SVD해 가장 큰 특이값 3개만 남기면 곧바로 rank-3 분해를 손에 쥘 수 있다. 다만 임의의 가역 행렬 $\mathbf{A}$에 대해 $\mathbf{M}\mathbf{A}$와 $\mathbf{A}^{-1}\mathbf{S}$ 역시 같은 $\mathbf{W}$를 만든다. 따라서 **metric constraints**(후속 문헌에서는 흔히 metric upgrade라 부른다)로 $\mathbf{A}$를 확정 지어야 하는데, 이는 카메라 축을 뜻하는 행들이 서로 직교하고 길이가 1이어야 한다는 제약이다.

단 한 번의 SVD와 행렬 보정만으로 모든 프레임의 움직임과 모든 점의 3D 좌표를 동시에 건져 올리는 우아함 덕분에, factorization은 반복 비선형 최적화인 bundle adjustment보다 구현이 훨씬 단출하고 매력적으로 보였다.

하지만 이 우아함은 정사영·affine 카메라라는 좁은 가정 위에서만 성립했다. 원근 왜곡(perspective distortion)을 무시하는 affine 모델은 망원 렌즈로 멀리 있는 작은 물체를 찍을 때처럼 장면 내 깊이 차이가 카메라까지의 거리에 비해 미미할 때만 잘 맞았다. 광각 렌즈를 달고 실내를 누비거나 카메라 바로 코앞의 물체를 찍는 환경에서는 오차가 걷잡을 수 없이 커졌다. 1990년대 후반 연구자들이 factorization을 원근 모델로 확장하려 씨름하다 결국 한계에 부딪히면서, 컴퓨터 비전 진영은 다시금 사진측량의 낡지만 정직한 무기—bundle adjustment—를 찾아 나서게 된다.

---

## 3.3 Hartley & Zisserman의 교과서

2000년 출간된 Richard Hartley와 Andrew Zisserman의 교과서 *[Multiple View Geometry in Computer Vision](https://www.robots.ox.ac.uk/~vgg/hzbook/)*은 1981년 이후 20년 가까이 개별 논문 속에 파편화돼 있던 SfM의 수학을 사영기하(projective geometry)라는 단 하나의 일관된 문법으로 통합했다. essential matrix, fundamental matrix, homography, 카메라 교정, 그리고 bundle adjustment가 모두 같은 기하와 표기법으로 설명되자, 연구자들은 비로소 흩어져 있던 퍼즐 조각들이 한 그림이었음을 직관적으로 깨닫게 됐다.

특히 이 책은 그동안 사진측량의 전유물처럼 여겨지던 bundle adjustment를 전면에 내세웠다. 사영기하의 틀 안에서 reprojection error 최소화를 다시 정식화한 뒤, 현실 데이터의 오매칭(outlier)에 휘둘리지 않도록 Huber나 Cauchy 같은 *robust cost function* $\rho$를 얹고, Levenberg-Marquardt와 Jacobian의 희소 구조를 결합해 실전용 레시피로 내놓은 것이다. 이 책은 이후 20년간 거의 모든 SLAM과 VO(visual odometry) 논문의 참고 문헌 첫머리를 지켰다.

---

## 3.4 Photo Tourism과 Bundler — 인터넷 규모 SfM

교과서의 수식이 비통제 환경의 야생 데이터에서도 통한다는 사실이 2006년 SIGGRAPH 논문 "[Photo Tourism](https://doi.org/10.1145/1179352.1141964)"에서 만천하에 증명됐다. Noah Snavely, Steven Seitz, Richard Szeliski는 전문 측량 장비를 쓰는 대신 관광객들이 Flickr에 무작위로 올려놓은 사진들을 긁어모아 파리 노트르담 대성당과 로마 트레비 분수를 3차원으로 되살렸다.

인터넷에서 긁어모은 사진들은 카메라 기종도, 촬영 날짜와 날씨도, 화각도 제각각인 데다 전혀 상관없는 사진까지 뒤섞여 있었다. Snavely의 파이프라인은 먼저 SIFT로 특징점을 뽑아 이미지 쌍 사이의 대응점을 잇고, RANSAC으로 fundamental matrix를 추정해 기하에 맞지 않는 가짜 매칭을 쳐냈다. 그런 다음 대응점이 가장 탄탄한 이미지 쌍에서 출발해 카메라를 한 장씩 붙여 나가고(incremental SfM), 카메라가 추가될 때마다 bundle adjustment를 돌려 전체 자세와 점 좌표를 다듬었다.

실험 결과는 당시로선 경이로웠다. Flickr에서 모은 노트르담 대성당 사진 2,635장 중 597장을 한 공간에 정렬했고, 트레비 분수(466장 중 360장), 요세미티 하프돔(1,882장 중 325장), 트라팔가 광장(1,893장 중 278장)을 재구성했다. 한 사람이 한 카메라로 찍은 만리장성(120장 중 82장)과 프라하 구시가 광장(197장)도 거뜬히 복원했다. 등록된 이미지의 평균 해상도는 1,611×1,128 픽셀이었고 평균 reprojection error는 약 1.5 픽셀 수준이었다. Snavely가 이 파이프라인을 **Bundler**라는 이름으로 오픈소스로 공개하자, Bundler는 곧 SfM 연구 전체가 딛고 서는 출발선이 됐다.

<!-- DEMO: sfm_incremental.html -->

> 📜 **예언 vs 실제.** Snavely et al.(2006)은 논의 절에서 "Ultimately, we wish to scale up our reconstruction algorithm to handle millions of photographs"라는 포부를 적으며, 카메라가 늘수록 굼떠지는 등록 속도, 렌즈 왜곡 모델, 반복 패턴이나 텍스처 부족 장면, 서로 이어지지 않는 섬처럼 쪼개진 재구성을 숙제로 남겼다. 10년 뒤 COLMAP 논문(Schönberger & Frahm, CVPR 2016)에서는 로마 사진 75,000장 가운데 21,000장을 단일 모델로 정렬해 보였고, 같은 시기 수백만 장 규모를 겨냥한 대규모 시스템들도 차례로 등장했다(→ 1장 §1.4).

---

## 3.5 COLMAP — 공학적으로 다듬은 SfM

Bundler가 가능성을 열었다면, 그 잠재력을 누구나 믿고 쓸 수 있는 완성도로 끌어올린 것은 2016년 CVPR에 발표된 Johannes Schönberger와 Jan-Michael Frahm의 "[Structure-from-Motion Revisited](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)"였다. 두 사람은 공개 시스템인 Bundler와 비공개 시스템인 VisualSFM을 곁에 두고 incremental SfM의 매 단계를 처음부터 끝까지 해부해 다시 설계한 뒤, 그 결실을 **COLMAP**이라는 이름으로 공개했다.

이들이 가장 먼저 손을 댄 대목은 *다음에 어떤 이미지를 붙일 것인가*였다. 잘못 고른 사진 한 장이 연쇄적인 등록 실패와 왜곡된 삼각측량을 불러온다고 보았기 때문이다. COLMAP은 이미 복원된 3D 점이 새 이미지에 몇 개나 보이는지뿐 아니라 그 점들이 화면 전체에 얼마나 고르게 퍼져 있는지까지 점수로 매겼다. 실험해 보니 결국 등록되는 이미지의 총량은 같더라도, 이 분산 기준을 따랐을 때 최종 카메라 위치 오차가 가장 작았다.

최적화 주기 역시 실용적으로 손질했다. 카메라 한 장을 붙일 때마다 전체 bundle adjustment를 돌리면 규모가 커질수록 시간이 감당되지 않는다. COLMAP은 VisualSFM의 전략을 따라, 새 이미지를 붙일 때는 그 이미지와 가장 밀접히 연결된 이웃 이미지들만 묶어 local bundle adjustment를 돌리고, 전체 모델을 아우르는 global bundle adjustment는 모델 크기가 일정 비율 이상 불어났을 때만 가동했다.

여기에 기하 검증의 그물을 한층 촘촘히 짰다. 우선 이미지 쌍마다 fundamental matrix를 맞춰 inlier가 충분한 쌍만 통과시킨 뒤, 같은 쌍에 homography를 함께 맞춰 본다. homography는 평면 장면이거나 카메라가 제자리에서 회전만 한 경우를 설명하므로, homography inlier 비중이 상대적으로 낮아야 비로소 깊이감이 있는 일반 3D 장면으로 인정한다. 내부 파라미터를 아는 경우에는 essential matrix로 점을 삼각측량해 시차 각도(triangulation angle)로 제자리 회전(panoramic)과 평면 장면을 가려냈다. 심지어 사진 모서리의 워터마크나 날짜 스탬프 때문에 전혀 다른 장소의 사진이 엉뚱하게 연결되는 사고까지 이 단계에서 차단했다. 재구성은 반드시 충분한 시차가 확보된 일반 쌍에서 시작하며, 제자리 회전 쌍에서는 점을 삼각측량하지 않는다.

COLMAP이 3D 비전 생태계의 표준으로 굳어진 데에는 알고리즘 밖의 미덕도 컸다. 깔끔한 코드와 친절한 문서, 그리고 CUDA 가속 덕분에 대용량 데이터도 무리 없이 돌아갔기 때문이다. 2020년 NeRF가 등장하고 뒤이어 3D Gaussian Splatting이 붐을 일으켰을 때, 대부분의 학습 파이프라인은 COLMAP이 뽑아낸 카메라 자세와 희소 점군을 당연한 입력처럼 받아 썼다. COLMAP은 사실상 현대 3D 재구성의 관문이 됐다.

> 🔗 **차용.** COLMAP의 뼈대는 Bundler와 동일한 incremental SfM이다. essential matrix 추정, 삼각측량, Levenberg-Marquardt 같은 핵심 수학은 Hartley & Zisserman 교과서에서 한 치도 벗어나지 않았다. COLMAP을 특별하게 만든 것은 이미지 선택 순서, 다층 기하 검증, 최적화 주기를 다듬은 엔지니어링의 총합이었다.

---

## 3.6 SfM과 SLAM이 나뉘다

이처럼 SfM과 SLAM은 같은 사영기하와 최소자승 수학을 뿌리에 두고 있으면서도, 2000년대 초반에 이르러 서로 다른 문제를 푸는 별개의 분야로 갈라섰다. 갈림길을 만든 결정적 기준은 **시간**이었다.

SfM은 촬영이 모두 끝난 뒤 *오프라인*에서 데이터를 만진다. 시간 제약이 없으니 전체 사진을 몇 번이고 다시 훑으며 global bundle adjustment를 반복하고, 중간에 자세가 틀어졌으면 되돌아가 다시 풀 여유가 있다.

반면 움직이는 로봇 위에서 돌아가는 SLAM에는 그런 사치가 허락되지 않는다. 센서 데이터가 밀려드는 그 순간마다 로봇의 현재 위치를 즉각 내놓아야 한다. 지나간 프레임 전체를 매번 다시 최적화할 수도 없고, 지도가 커질수록 한 프레임에 쓸 수 있는 시간 예산은 더욱 빠듯해진다. 그러다 로봇이 한 바퀴를 돌아 처음 출발했던 장소로 되돌아오면(loop closure), 그사이 누적된 오차(drift)를 한꺼번에 바로잡아 루프 위의 과거 자세들까지 되돌려 고쳐야 한다.

불확실성을 다루는 태도 역시 갈렸다. SLAM은 새 관측이 들어올 때마다 현재 자세의 불확실성을 실시간으로 갱신해야 하므로 EKF나 factor graph 같은 확률적 상태 표현을 중심에 두는 반면, SfM은 최적화가 끝난 뒤에 사후적으로 공분산을 계산해도 무방했다.

2003년 ICCV 논문 "[Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)"에서 Andrew Davison은 바로 이 경계선을 콕 집어 말했다. 당시까지 성공을 거둔 SfM 기법들은 예외 없이 영상을 모두 모은 뒤 일괄 처리하는 오프라인 알고리즘뿐이라는 것이다. 그는 카메라로 실시간 위치 추정을 하려면 불확실성 전파를 중시해 온 로봇공학의 SLAM 계보가 오프라인 SfM보다 훨씬 가까운 출발점이라고 주장하며, 자신의 시스템을 "sequential Simultaneous Localisation and Mapping or Structure from Motion"이라 불렀다. 2000년대를 지나며 두 분야는 각자의 문제 정의와 평가 기준을 가진 독자적 영역으로 확고히 자리 잡았다.

> 🔗 **차용.** Nistér, Naroditsky, Bergen의 2004년 CVPR 논문 "Visual Odometry"는 SfM의 기하 기법을 카메라 자체의 움직임(ego-motion)을 실시간으로 추정하는 데 끌어들인 초기 대표작이다. 전체 데이터를 한꺼번에 푸는 대신 짧은 구간 안의 상대 자세를 순차적으로 풀어, 전역 최적화보다 정밀도는 다소 양보하더라도 즉각적인 응답 속도를 얻어 냈다.

---

## 아직 열린 문제

**움직이는 물체가 있는 장면의 SfM.** COLMAP을 포함한 범용 SfM 파이프라인은 세계가 완전히 정지해 있다고 가정한다. 모든 3D 점이 고정돼 있다는 전제로 bundle adjustment를 돌리기 때문에, 자동차와 보행자가 오가는 거리에서는 움직이는 물체 위의 매칭이 최적화를 통째로 왜곡한다. RANSAC이 일부를 걸러 내 주긴 하지만 근본 해법은 아니어서, Dynamic SfM 연구에서는 semantic segmentation이나 객체 단위 모션 모델을 결합해 정적 배경과 동적 물체를 갈라내려 한다.

**흐려지는 SfM과 SLAM의 경계.** 2023년 12월 Wang 등이 발표한 [DUSt3R](https://arxiv.org/abs/2312.14132)는 카메라 내부 파라미터도 자세도 모르는 사진 두 장을 Transformer에 곧바로 넣어, 픽셀마다 3D 좌표가 담긴 pointmap을 한 번에 예측해 냈다. 카메라 자세와 픽셀 대응은 오히려 그 pointmap에서 사후적으로 뽑아낸다. 사진이 석 장 이상이면 쌍마다 얻은 pointmap을 global alignment로 하나의 좌표계에 맞춘다. 뒤이은 [MASt3R](https://arxiv.org/abs/2406.09756)(Leroy et al., 2024)는 여기에 dense local feature 출력까지 더해 매칭 정밀도를 끌어올렸다. 전통적인 SfM 파이프라인의 모듈이 하나씩 신경망으로 대체되면서, NeRF와 3DGS 전처리를 COLMAP 대신 DUSt3R 계열로 넘기려는 시도도 늘고 있다. 학습 기반 모델이 COLMAP의 아성을 완전히 대신할지, 아니면 특정 영역을 나눠 갖게 될지는 아직 단언하기 이르다.

---

SfM 연구자들이 오프라인 3D 재구성의 정밀도를 벼리는 동안, 로봇공학자들은 당장 눈앞에서 굴러가는 로봇을 위해 "데이터가 다 모이기 전에, 불확실성을 껴안은 채 지금 위치를 추정하는" 전혀 다른 수학을 세우고 있었다. 그 이야기는 1986년 Randall Smith와 Peter Cheeseman이 던진 [한 가지 질문](https://people.csail.mit.edu/brooks/idocs/Smith_Cheeseman.pdf)에서 시작된다(→ 4장).
