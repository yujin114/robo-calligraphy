# robo-calligraphy


✍️ 필기 로봇팔 프로젝트 - wr_robot

두산 협동 로봇(M0609)과 OnRobot RG2 그리퍼를 활용하여 필기 동작을 수행하는 ROS 2 기반 프로젝트입니다.


✅ 주요 기능

    한글 인덱스 단위 스켈레톤 경로를 따라 종이 위에 필기

    다중 스트로크 처리 및 경로 샘플링 기능

    로봇 제어를 활용한 ‘닿았다 떼는’ 필기 동작

    Plot을 통해 경로 시각화 가능


🔧 사전 준비

1. Teach Pendant에서 설정

    Tool/TCP 설정

        Tool: Tool Weight10

        TCP: GripperDA_v10

   
🚀 실행 방법

터미널 1 (로봇 제어 노드 실행)

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m0609 host:=192.168.1.100

터미널 2 (UI 실행)

ros2 run wr_robot ui

터미널 3 (토픽 발행)

ros2 run wr_robot visual

터미널 4 (로봇 필기 제어)

ros2 run wr_robot control_robot

📷 예시 이미지

예시 이미지는 프로젝트 루트에 포함된 example1.png, example2.png, example3.png 등을 참고하세요.

🎥 참고 영상



https://github.com/user-attachments/assets/a01edaed-d514-4e42-b9ac-6fa32bce59fb

