# tricat221

> 링크로 각 소제목 이동하기

# To Do

## Documents

* 개발 상세 보고서
    * MarkDown version, LaTeX version
    * 각종 팁
        * alias
        * shell script
        * black format & GitHub Actions
        * 각종 문제 해결 (Ex. IMU 캘리브레이션, 라이다 수평, 
        * 시뮬링크
        * roslaunch rosbag 노드 주석 쓰는 법
* rqt_graph
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
* 다른 패키지 requirements
    * GPS는 따로 다운 및 수정 방법 정리해두기
    * pymap3d 설치 방법 및 유의사항
    * fuzzy 설치 방법
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

# 디렉터리 구조

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
│   └─ 99-tty.rules                 (Simul Link file: ~~ 참고) # TODO
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
└─ requirements                 () # TODO
```

## module_test.launch 설명