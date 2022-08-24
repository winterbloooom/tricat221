# tricat221

> 링크로 각 소제목 이동하기

# To Do

## Documents

* 개발 상세 보고서
    * MarkDown version, LaTeX version
* 대회 보고서(준비 ~ 회고, 4기를 위한 제언)
    * LaTeX version, MarkDown version
    * Blog용 요약본
* 노션 정리해 팀 일지
* 노션 정리해 아카이브로 시키기
* PPT, PDF version 간단 요약 프로젝트 소개

## GitHub
* 코드 정리 및 다큐멘트 추가
* 라이선스 파일 추가
* 라이선스 표기 각 파일마다 추가
* rosbag, img, mp4 파일 등 추가 및 삭제
* 아두이노 최종 코드
* Rviz 모듈 레포지토리
* 트라이캣 organ 정리
* old auto 코딩

## 공개할 데이터
> winterbloooom 드라이브에 추가하고 TODO 등에 링크 걸기
* [경기 영상 전체 재생 목록](https://www.youtube.com/playlist?list=PLBScO6lsHRV1a6kaPttd6ulcyLxNG6T-N)
* Docker 이미지 및 사용법
* rosbag: cam rosbag은 동영상으로 따로 저장하고 나머지 토픽은 다시 녹화

## 트라이캣 내부용 데이터
* 사진, 동영상

## 개인 소장용 데이터
* 사진, 동영상

## Devices
* Nuc 웹브라우저 초기화 + 로그아웃
* Nuc catkin_ws, 다운로드, 이미지 등 데이터 삭제
* Nuc bashrc 편집

## 블로그에 올릴 글
* 프로젝트 요약, 상세 개발 보고서, 대회 보고서, 데이터셋 링크 등 링크 모음
* rviz 모듈 만들기
* Ros 추가 팁
    * roslaunch: rviz, rosbag, rqt, args, if, unless, roslaunch rosbag 노드 주석 쓰는 법
    ```xml
    <!-- <launch>
        <node name="$(anon usb_cam)" pkg="usb_cam" type="usb_cam_node" output="screen" >
        <param name="video_device" value="$(arg n)" />
        <param name="image_width" value="640" />
        <param name="image_height" value="480" />
        <param name="pixel_format" value="yuyv" />
        <param name="camera_frame_id" value="usb_cam" />
        <param name="io_method" value="mmap"/>
        <param name="exposure" value="50"/>
        </node>
        <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
        <remap from="image" to="/usb_cam/image_raw"/>
        <param name="autosize" value="true" />
        </node>
    </launch>
    -->

    <!-- roslaunch package launchfile n:=value -->

    <!-- <remap from="/camera1/usb_cam/image_raw/" to="/star_cam/image_raw"/> -->
    <!-- <param name="frame_rate???" value="mmap" /> -->

    <!-- <group ns="camera1">
            <node name="usb_cam" pkg="usb_cam" type="usb_cam_node">
                <param name="video_device" value="/dev/video0" />
                <param name="autoexposure" value="false" />
                <param name="pixel_format" value="yuyv" />
                <param name="camera_frame_id" value="usb_cam" />
                <param name="io_method" value="mmap" />
                
                <param name="image_width" value="640" />
                <param name="image_height" value="480" />
                <param name="exposure" value="150" />
                <param name="contrast" value="0" />
                <param name="saturation" value="100" />
                
            </node>
        </group> -->
        <!-- <group ns="camera2">
            <node name="usb_cam" pkg="usb_cam" type="usb_cam_node">
                <param name="video_device" value="/dev/video2" />
                <param name="autoexposure" value="false" />
                
                <param name="image_width" value="640" />
                <param name="image_height" value="480" />
                <param name="pixel_format" value="yuyv" />
                <param name="camera_frame_id" value="usb_cam" />
                <param name="io_method" value="mmap" />

                <param name="exposure" value="150" />
                <param name="contrast" value="0" />
                <param name="saturation" value="100" />
            </node>
        </group> -->
    ```
    * roslaunch와 shell script
    * msg wait 해서 노드 기다리기

- - -

# 디렉터리 구조 및 외부 라이브러리 설치

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
│   ├─ arduino/                     (아두이노 소스코드)
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
│   │   ├─  servo_test.py               (서보모터 테스트 모듈)
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

## Symbolic Link
각 센서의 symbolic link(일종의 바로가기)를 만들어 장치의 이름을 고정할 수 있다. 컴퓨터에 `/etc/udev/rules.d/` 경로로 해당 파일을 복사하고 udev 설정을 재로드한뒤, 컴퓨터를 재시작한다.

```bash
sudo cp 99-tty.rules /etc/udev/rules.d/99-tty.rules
sudo service udev reload
sudo service udev restart
```

## Requirements
ROS 설치 시 자동 설치되는 라이브러리/도구 외에 따로 설치해야 하는 라이브러리를 나타내었다. 한 번에 설치하기 위하여 패키지 디렉터리 위치로 이동하여 아래의 명령어를 수행한다.

```bash
pip install -r requirements.txt
```

개별적 설치를 할 때는 Python 버전을 2.x 인지 확인한 뒤 `pip3` 가 아니라 `pip` 명령어로 설치를 진행해야 ROS에서 실행할 수 있다. pymap3d 라이브러리 설치 시 `egg_info failed` 관련 에러가 나타난다면 아래의 명령어를 입력해 setuptools를 업그레이드한다. 👉 [오류 해결 출처](https://musclebear.tistory.com/131)

```bash
sudo -H pip install --upgrade --ignore-installed pip setuptools
```

- - -

# 공개 데이터
## rosbag recording
| file name | description | `params/coordinates.yaml` | record data | duration | size |
|---|---|---|---|---|---|
| 220811-hopping.bag|인하대 분수대 호핑투어 테스트 | C | 2022.08.11. | 64s | 8.2 MB |
| 220817-hopping.bag|호핑투어 경기 | A | 2022.08.17. | 190s | 21.6 MB |
| 220818-auto-pre.bag|자율운항 예선 경기 | B | 2022.08.18. | 23.7s | 22.1 MB |
| 220819-auto-final.bag|자율운항 본선 경기 | A | 2022.08.19. | 27.6s | 26.4 MB |
| 220819-docking-blue-cross.bag|도킹 경기 | A | 2022.08.19. | 27.9s | 735.6 MB |
| 220819-docking-green-triangle.bag|도킹 경기 | A | 2022.08.19. | 37.7s | 991.9 MB |

## video data
| file name | corresponding rosbag file | record data | duration | size |
|---|---|---|---|---|
| blue-cross.bag | final-docking.bag | 2022.08.19. | 27.9s | 21.0 MB |
| green-triangle.bag | 220819-123821-docking-04(25초).bag | 2022.08.19. | 36s | 32.1 MB |

## ROS packages
본 팀이 사용한 IMU(AHRS), GPS, Camera, LiDAR의 ROS 드라이버는 GitHub에서 쉽게 clone하여 사용할 수 있으며, 소스코드를 압축한 파일을 따로 탑재해 두었다. GPS 관련 패키지는 세부 설정이 필요하기 때문이다. 👉 [GPS 드라이버 설치 방법](https://velog.io/@717lumos/GPS-ublox-ZED-F9P-GPS-%EC%82%AC%EC%9A%A9%EB%B2%95)

* [IMU(AHRS) 드라이버](https://github.com/robotpilot/myahrs_driver)
* [GPS 드라이버](https://github.com/ros-agriculture/ublox_f9p)와 [NTRIP Client 패키지](https://github.com/ros-agriculture/ntrip_ros)
* [LiDAR 드라이버](https://github.com/Slamtec/rplidar_ros)
* [USB 카메라 드라이버](https://github.com/ros-drivers/usb_cam)

## Competition video clips
경기 영상 녹화본 및 시각화 결과를 편집하여 YouTube에 게시하였다. 👉 [전체 재생목록](https://youtube.com/playlist?list=PLBScO6lsHRV1a6kaPttd6ulcyLxNG6T-N)

🎬 [호핑투어 오토파일럿](https://youtu.be/VELbh6ZdrzQ) 경기 영상<br>
🎬 [자율운항 장애물 통과(예선+결선)](https://youtu.be/IKwgBN4L3A0) 경기 영상<br>
🎬 [자율운항 도킹](https://youtu.be/-ghsQaKhZ-o) 경기 영상<br>