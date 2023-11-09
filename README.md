# Autonomous Boat: KABOAT2022, Team i-Tricat221

이 레포지토리는 제3회 자율운항보트 경진대회(Korea Autonomous Boat Competition 2022, KABOAT 2022)의 참가팀 인하대학교 i-Tricat 221의 임무 수행을 위한 ROS 패키지 및 소스코드 등을 포함하고 있다.

```
This project is licensed under the terms of the GNU General Public License v3.0.
(이 레포지토리는 GNU GPL 라이선스를 따르고 있습니다. 자세한 사항은 LICENSE 파일을 참고해주세요.)
```

## Contents
1. [Documentation and Open Data](#documentation-and-open-data) (관련 문서)
2. [Installation and Usage](#installation-and-usage) (설치 및 사용)
3. [License](#license) (라이선스)


## Documentation and Open Data
### Documents
📋 [상세 개발 보고서](https://winterbloooom.github.io/autonomous%20vehicle/kaboat2022-dev-report/](https://winterbloooom.github.io/robotics/autonomous%20vehicle/2022/08/25/kaboat2022_dev_report.html)): 개발 기획 단계부터 알고리즘 설계 및 구현, 테스트 및 대회 적용 등 전반적 사항을 상세하게 기술하고 있음<br>
📝 [대회 제출 보고서](https://drive.google.com/file/d/1nP1QHVJlDKosmO9re21HN9KshMnNTI5O/view?usp=sharing): KABOAT 2022 설계 심사 제출 보고서<br>

### Open Data
💽 **rosbag recording**

| file name | description | `params/coordinates.yaml` | record data | duration | size |
|---|---|---|---|---|---|
| 220811-hopping.bag|인하대 분수대 호핑투어 테스트 | C | 2022.08.11. | 64s | 8.2 MB |
| 220817-hopping.bag|호핑투어 경기 | A | 2022.08.17. | 190s | 21.6 MB |
| 220818-auto-pre.bag|자율운항 예선 경기 | B | 2022.08.18. | 23.7s | 22.1 MB |
| 220819-auto-final.bag|자율운항 본선 경기 | A | 2022.08.19. | 27.6s | 26.4 MB |
| 220819-docking-blue-cross.bag|도킹 경기 | A | 2022.08.19. | 27.9s | 735.6 MB |
| 220819-docking-green-triangle.bag|도킹 경기 | A | 2022.08.19. | 37.7s | 991.9 MB |

📷 **video data**

| file name | corresponding rosbag file | record data | duration | size |
|---|---|---|---|---|
| blue-cross.bag | final-docking.bag | 2022.08.19. | 27.9s | 21.0 MB |
| green-triangle.bag | 220819-123821-docking-04(25초).bag | 2022.08.19. | 36s | 32.1 MB |

📺 **Competition video clips**
경기 영상 녹화본 및 시각화 결과를 편집하여 YouTube에 게시하였다. 👉 [전체 재생목록](https://youtube.com/playlist?list=PLBScO6lsHRV1a6kaPttd6ulcyLxNG6T-N)

* [호핑투어 오토파일럿](https://youtu.be/VELbh6ZdrzQ) 경기 영상<br>
* [자율운항 장애물 통과(예선+결선)](https://youtu.be/IKwgBN4L3A0) 경기 영상<br>
* [자율운항 도킹](https://youtu.be/-ghsQaKhZ-o) 경기 영상<br>


## Installation and Usage
### Repository 디렉터리 구조
```
├─ .github/                     (github actions 관련 파일)
│   └─ workflows            
│        └─  black_formatter.yaml  (Black Formatter 적용)
│
├─ Arduino/                     (아두이노 코드 모음)
│
├─ data/                        (테스트용 데이터, 프로그램 실행 결과)
│   ├─ rosbag/                      (rosbag data)
│   ├─ rviz_record/                 (Rviz Record Video)
│   ├─ sample_imgs/                 (테스트용 이미지, 각종 실행 결과 캡쳐)
│   ├─ hopping_coordinates.pdf      (대회 측 제공 호핑투어 및 경기장 좌표)
│   └─ 99-tty.rules                 (Symbolic Link 생성 파일)
│
├─ launch/                      (roslaunch files)
│   ├─ autonomous.launch            (자율운항 장애물 통과 경기용)
│   ├─ docking.launch               (자율운항 도킹 경기용)
│   ├─ fuzzy.launch                 (자율운항 장애물 통과 경기용 (Plan B))
│   ├─ hopping_tour.launch          (호핑투어 경기용)
│   ├─ sensor_test.launch           (각종 모듈 테스트)
│   ├─ simul_autonomous.launch      (자율운항 장애물 통과 테스트용)
│   ├─ simul_docking.launch         (자율운항 도킹 테스트용)
│   └─ simul_hopping.launch         (호핑투어 테스트용)
│
├─ params/                      (파라미터 yaml 파일 모음)
│   ├─ autonomous_params.yaml       (자율운항 장애물 통과 용)
│   ├─ coordinates.yaml             (좌표 원점, waypoints, station 위치 등)
│   ├─ docking_params.yaml          (자율운항 도킹 용)
│   ├─ hopping_params.yaml          (호핑투어 용)
│   ├─ lidar_params.yaml            (lidar_converter 용)
│   └─ servo_params.yaml            (servo_test 용)
│
├─ rviz/                        (Rviz 설정 파일 모음)
│   ├─ rviz_conf_2021hop.rviz       (호핑투어 2021 version 용)
│   ├─ rviz_conf_auto.rviz          (자율운항, 호핑투어 용)
│   ├─ rviz_conf_data_collect.rviz  (자율운항 도킹 데이터 수집용)
│   ├─ rviz_conf_docking.rviz       (자율운항 도킹 용)
│   └─ rviz_conf_lidar.rviz         (lidar_converter 용)
│
├─ src/                         (소스코드)
│   ├─ arduino/                     (아두이노 노드 소스코드)
│   │
│   ├─ autonomous/                  (자율운항 소스코드)
│   │
│   ├─ control/                     (제어 관련 모듈)
│   │
│   ├─ datatypes/                   (커스텀 자료형)
│   │
│   ├─ dock/                        (도킹 소스코드)
│   │
│   ├─ hopping/                     (호핑투어 소스코드)
│   │
│   ├─ utils/                       (각종 모듈 및 기능)
│   │   ├─  gnss_converter.py           (GPS -> ENU 좌표계 변환 모듈)
│   │   ├─  gps_show.py                 (GPS 데이터 확인 모듈)
│   │   ├─  heading_calculator.py       (IMU 지자기센서 -> 선수각 계산 모듈)
│   │   ├─  lidar_converter.py          (라이다 데이터 클러스터링 모듈)
│   │   ├─  obstacle_avoidance.py       (장애물 회피 함수 모음)
│   │   ├─  tools.py                    (기타 함수 모음)
│   │   └─  visualizer.py               (Rviz 시각화 모듈)
│   │
│   ├─ autonomous.py                (자율운항 장애물 통과 실행 쉘 스크립트)
│   ├─ docking.py                   (자율운항 도킹 실행 쉘 스크립트)
│   └─ hopping.sh                   (호핑투어 실행 쉘 스크립트)
│
├─ .gitignore                   (깃허브 업로드 시 제외할 파일 목록)
├─ CMakeLists.txt               (catkin make file)
├─ package.xml                  (ros package file)
├─ pyproject.toml               (black formatter 등의 configuration)
└─ requirements.txt             (추가적으로 설치해야 할 라이브러리)
```

### Installation
해당 프로젝트를 사용하기 위해서는 ROS Melodic(18.04)의 desktop-full를 설치하여 관련 도구들 모두를 설치한다. 또한 본 팀이 사용한 IMU(AHRS), GPS, Camera, LiDAR의 ROS 드라이버는 GitHub에서 쉽게 clone하여 사용할 수 있으며, 소스코드를 압축한 파일을 따로 [드라이브에 탑재](https://drive.google.com/drive/folders/1TF9xHhzc6bc-4HkW3OEvvLpgaKmAWLDm?lfhs=2)해 두었다. GPS 관련 패키지는 세부 설정이 필요하기 때문이다.

* [IMU(AHRS) 드라이버](https://github.com/robotpilot/myahrs_driver)
* [GPS 드라이버](https://github.com/ros-agriculture/ublox_f9p)와 [NTRIP Client 패키지](https://github.com/ros-agriculture/ntrip_ros) 👉 [GPS 드라이버 설치 방법](https://winterbloooom.github.io/perception/perception-ublox-gps/)
* [LiDAR 드라이버](https://github.com/Slamtec/rplidar_ros)
* [USB 카메라 드라이버](https://github.com/ros-drivers/usb_cam)

또한 추가적으로 설치해야 하는 라이브러리를 `requirements.txt`에 나타내었다. 패키지 디렉터리 위치로 이동하여 아래의 명령어를 수행한다.

```bash
pip install -r requirements.txt
```

개별적 설치를 할 때는 Python 버전을 2.x 인지 확인한 뒤 `pip3` 가 아니라 `pip` 명령어로 설치를 진행해야 ROS에서 실행할 수 있다. pymap3d 라이브러리 설치 시 `egg_info failed` 관련 에러가 나타난다면 아래의 명령어를 입력해 setuptools를 업그레이드한다. 👉 [오류 해결 출처](https://musclebear.tistory.com/131)

```bash
sudo -H pip install --upgrade --ignore-installed pip setuptools
```

### Usage
💡자율운항 (autonomous)
```bash
# 기본적 실행
roslaunch tricat221 autonomous.launch
# rosbag record 사용여부 설정
roslaunch tricat221 autonomous.launch do_record:=[rosbag_record_여부] filename:=[rosbag_파일_이름]
# shell script 실행
./src/autonomous.sh
```

💡호핑투어 (hopping_tour)
```bash
# 기본적 실행
roslaunch tricat221 hopping_tour.launch
# rosbag record 사용여부 설정
roslaunch tricat221 hopping_tour.launch do_record:=[rosbag_record_여부] filename:=[rosbag_파일_이름]
# shell script 실행
./src/hopping.sh
```

💡도킹 (docking)
```bash
# 기본적 실행
roslaunch tricat221 docking.launch
# rosbag record 사용여부 설정
roslaunch tricat221 docking.launch do_record:=[rosbag_record_여부] filename:=[rosbag_파일_이름]
# shell script 실행
./src/docking.sh
```

### Symbolic Link
각 센서의 symbolic link(일종의 바로가기)를 만들어 장치의 이름을 고정할 수 있다. 컴퓨터에 `/etc/udev/rules.d/` 경로로 해당 파일을 복사하고 udev 설정을 재로드한뒤, 컴퓨터를 재시작한다.

```bash
sudo cp 99-tty.rules /etc/udev/rules.d/99-tty.rules
sudo service udev reload
sudo service udev restart
```

## License
이 레포지토리는 GNU GPL 라이선스를 따르고 있다. 자세한 사항은 프로젝트 내 LICENSE 파일에 라이선스 내용이 있으며, 각 라이선스의 종류나 사용 방법 등은 아래 링크에서 더 볼 수 있다.

* [우노, "[GitHub] License 란?"](https://wooono.tistory.com/379)
* [황은경, "오픈소스를 사용하고, 준비하는 개발자를 위한 가이드"](https://www.slideshare.net/ifkakao/ss-113145564)
* [codeNamu, "[춘식이의 코드이야기] 대표 오픈소스 라이선스, 한 눈에 보기!"](https://codenamu.org/2014/10/10/popular-opensource-license)
* [오픈소스SW 라이선스 종합정보시스템, "라이선스 가이드"](https://olis.or.kr/license/licenseGuide.do)
* [다크 프로그래머, "공개 SW 라이센스 GPL, LGPL, BSD"](https://darkpgmr.tistory.com/89)
* [Choose an open source license](https://choosealicense.com/)
* [Open Source Guides, "The Legal Side of Open Source"](https://opensource.guide/legal/)
