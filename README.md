// HW1

// 소스 파일은 HW1/ros2_ws/src/cpp_pubsub/src 존재함

프로그램 설치 방법:

    1. 프로젝트 폴더를 다운 받는다.
    
    2. 리눅스 기준 홈에 ros2_ws파일을 위치시킨다.
    
    3. 터미널창을 열어 $ ros2 run turtlesim turtlesim_node를 입력한다.
    
    4. 새로운 터미널창을 열어 다음을 차례대로 입력한다.
    
	cd ~/ros2_ws

	colcon build --packages-select cpp_pubsub

	. install/setup.bash

	ros2 run cpp_pubsub talker


    5. 새로운 터미널창을 열어 다음을 차례대로 입력한다. 
	cd ~/ros2_ws
	
	colcon build --packages-select cpp_pubsub
	
	. install/setup.bash
	
	ros2 run cpp_pubsub listener



프로젝트 사용방법

    1. talker 터미널 창에 마우스 커서를 클릭하고 방향키로 거북이를 조정한다.
    
    2. t,r,c 키를 누르면 각각 삼각형 사각형 원이 그려진다.
    
    3. q키를 누르면 노드가 종료된다.
    
    4. s키를 누르면 중앙으로 이동한다.



// HW2

프로그램 설치 방법:

    1. 프로젝트 폴더를 다운 받는다.
    
    2. 리눅스 기준 홈에 ros2_ws 파일을 위치시킨다.
    
    3. 터미널창을 열어 $ cd ~/ros2_ws를 입력한다.
    
    4. $ colcon build
    
    5. 새로운 터미널창을 열어 다음을 차례대로 입력한다.
       $ cd ~/ros2_ws
       $ source install/setup.bash
       $ ros2 run publisher publisher
       
    6. 새로운 터미널창을 열어 다음을 차례대로 입력한다.
       $ cd ~/ros2_ws
       $ source install/setup.bash
       $ ros2 run subscriber subscriber
       
       
프로젝트 사용 방법:

    1. publisher 노드를 실행한 터미널창에 마우스 커서를 가져다 대고 클릭한다.
    
    2. 터미널 창에 나오는 질문들을 차례로 입력한다.(name, grade, student_name)
    
    3. publisher subscriber가 정상 작동하는지 확인한다.
