# robo-calligraphy


✍️ 드로잉 로봇팔 프로젝트 - dr_writer

두산 협동 로봇(M0609)과 OnRobot RG2 그리퍼를 활용하여 필기/드로잉 동작을 수행하는 ROS 2 기반 프로젝트입니다.
✅ 주요 기능

    한글 자모 단위 스켈레톤 경로를 따라 보드 위에 필기

    다중 스트로크 처리 및 경로 샘플링 기능

    실제 힘 제어를 활용한 ‘닿았다 떼는’ 필기 동작

    RViz를 통해 경로 시각화 가능

🔧 사전 준비
1. Teach Pendant에서 설정

    Tool/TCP 설정

        Tool: Tool Weight

        TCP: GripperDA_v1

2. ROS 2 환경 설정

~/ros2_ws/src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py에 다음 라인 추가:

# 좌표 ID 설정
DR_WHITE_BOARD2 = 110  # 사용 중인 좌표계 ID

🚀 실행 방법
터미널 1 (로봇 제어 노드 실행)

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m0609 host:=192.168.1.100

터미널 2 (UI 실행)

ros2 run wr_robot ui

터미널 3 ()

ros2 run wr_robot visual

터미널 4 (로봇 필기 제어)

ros2 run wr_robot control_robot

📷 예시 이미지

예시 이미지는 프로젝트 루트에 포함된 example1.png, example2.png, example3.png 등을 참고하세요.
🎥 참고 영상

