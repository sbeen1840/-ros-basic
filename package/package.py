# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 16:52:27 2023

@author: user
"""

#%%
"""
INDEX
[1] PACKAGE
    [1-1] 패키지 설정
    [1-2] 패키지 이동
    [1-3] 패키지 설정파일 수정
[2] BUILD 설정파일 수정
[3] MESSAGE 파일생성
[4] NODE
    [4-1] pub 노드 파일생성
    [4-2] sub 노드 파일 생성
[5] BUILD
[6] RUN
[7] PARAMETER
    [7-1] 파라미터 활용한 노드작성 - pub
    [7-2] 파라미터 활용한 노드작성 - sub
    [7-3] 파라미터 입력
    [7-4] 노드실행시 파라미터 입력
    [7-5] 파라미터 검색 및 초기화
[8] LAUNCH
    [8-1] launch 파일 생성
    [8-2] launch로 여러 노드 실행

"""
#%% [1] PACKAGE
'[1-1] 패키지 설정'

$ cd ~/catkin_ws/src
$ catkin_create_pkg ros_tutorials_topic message_generation std_msgs rospy

#%% 
'[1-2] 패키지 이동'

$ cd ros_tutorials_topic

$ ls
# include → 헤더 파일 폴더
# src → 소스 코드 폴더
# CMakeLists.txt → 빌드 설정 파일
# package.xml → 패키지 설정 파일


#%% 
'[3] 패키지 설정파일 수정'

$ gedit package.xml

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
# 파일 내용
<?xml version="1.0"?>
<package format="2">
<name>ros_tutorials_topic</name>
<version>0.1.0</version>
<description>ROS turtorial package to learn the topic</description>
<license>Apache 2.0</license>
<author email="pyo@robotis.com">Yoonseok Pyo</author>
<maintainer email="pyo@robotis.com">Yoonseok Pyo</maintainer>
<url type="website">http://www.robotis.com</url>
<url type="repository">https://github.com/ROBOTIS-GIT/ros_tutorials.git</url>
<url type="bugtracker">https://github.com/ROBOTIS-GIT/ros_tutorials/issues</url >
<buildtool_depend>catkin</buildtool_depend>
<depend>rospy</depend>
<depend>std_msgs</depend>
<depend>message_generation</depend>
<export></export >
</package>
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

#%% [2] BUILD
'[2] BUILD 설정파일 수정'

$ gedit CMakeLists.txt

''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

# cmake 빌드툴의 버전쓰기
cmake_minimum_required(VERSION 2.8.3) 

# 패키지 이름이랑 일치시켜야 함
project(ros_tutorials_topic) 

## 캐킨 빌드를 할 때 요구되는 구성요소 패키지이다.
## 의존성 패키지로 message_generation, std_msgs, rospy이며 이 패키지들이 존재하지 않으면 빌드 도중에 에러가 난다.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rospy)

## 메시지 선언: MsgTutorial.msg 
add_message_files(FILES MsgTutorial.msg)

## 의존하는 메시지를 설정하는 옵션이다.
## std_msgs가 설치되어 있지 않다면 빌드 도중에 에러가 난다.
generate_messages(DEPENDENCIES std_msgs)


## 캐킨 패키지 옵션으로 라이브러리, 캐킨 빌드 의존성, 시스템 의존 패키지를 기술한다. 
catkin_package( 
LIBRARIES ros_tutorials_topic
CATKIN_DEPENDS std_msgs rospy
)


## 인클루드 디렉터리를 설정한다.
include_directories(${catkin_INCLUDE_DIRS})


## topic_publisher 노드에 대한 빌드 옵션이다. 
## 실행 파일, 타깃 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(topic_publisher src/topic_publisher.py) # 이 노드의 실행파일은 이 위치의 파일로
add_dependencies(topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} #추가 의존성 패키지 기술
${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_publisher ${catkin_LIBRARIES}) #타깃 링크 라이브러리


## topic_subscriber 노드에 대한 빌드 옵션이다. 
add_executable(topic_subscriber src/topic_subscriber.py) # 이 노드의 실행파일은 이 위치의 파일로
add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} #추가 의존성 패키지 기술
${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_subscriber ${catkin_LIBRARIES}) #타깃 링크 라이브러리
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

#%% [3] MESSAGE
'[3] MESSAGE 파일생성'

$ mkdir msg # → ros_tutorials_topic 패키지에 msg라는 메시지 폴더를 신규 작성
$ cd msg # → 작성한 msg 폴더로 이동
$ gedit MsgTutorial.msg # → MsgTutorial.msg 파일 신규 작성 및 내용 수정

''''''''''''
time stamp
int32 data
''''''''''''
$ cd .. 

#%% [4] NODE
'[4-1] pub 노드 파일생성'

$ cd src #  → ros_tutorials_topic 패키지의 소스 폴더인 src 폴더로 이동
$ gedit topic_publisher.py # → 소스 파일 신규 작성 및 내용 수정

''''''''''''''''''''''''''''''''''''''
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher')

pub = rospy.Publisher('my_topic', Int32) 

rate = rospy.Rate(2) 

count = 1

while not rospy.is_shutdown():
    pub.publish(count)
    count = count + 1
    rate.sleep()
''''''''''''''''''''''''''''''''''''''

#%%
'[4-2] sub 노드 파일 생성'

$ roscd ros_tutorials_topic/src # → 패키지의 소스 폴더인 src 폴더로 이동
$ gedit topic_subscriber.py

''''''''''''''''''''''''''''''''''''''
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def callback(msg):
    print (msg.data)

rospy.init_node('topic_subscriber') 

sub = rospy.Subscriber('my_topic', Int32, callback)

while not rospy.is_shutdown():
    pass
''''''''''''''''''''''''''''''''''''''

#%%[5] BUILD
'[5] BUILD'

$ cd ~/catkin_ws #  → catkin 폴더로 이동
$ catkin_make # → catkin 빌드 실행


#       • 소스 파일 위치
#         • ros_tutorials_topic 패키지의 소스 코드 파일: ~/catkin_ws/src/ros_tutorials_topic/src
#         • ros_tutorials_topic 패키지의 메시지 파일: ~/catkin_ws/src/ros_tutorials_topic/msg
#       • 빌드 결과물
#         • /build 폴더 : 캐킨 빌드에서 사용된 설정 내용이 저장
#         • /devel/lib/폴더 :  실행 파일이 저장
#         • /devel/include/폴더 : 메시지 파일로부터 자동 생성된 메시지 헤더 파일이 저장

#%% [6] RUN
'[6] RUN'
$ roscore
$ rosrun ros_tutorials_topic topic_publisher
$ rosrun ros_tutorials_topic topic_subscriber

'rosnode'
$ rosnode list #실행중인 노드 확인
$ rosnode cleanup #오류있는 노드 강제종료
$ rosnode kill nodename #노드 강제종료

'rostopic'
$ rostopic list #토픽 종류 확인
$ rostopic info /ros_tutorial_msg #토픽의 sub, pub등 확인
$ rostopic echo /ros_tutorial_msg #토픽의 데이터 값 보기

'etc'
$ rqt_graph #관계도
$ rqt #rqt열기
$ rviz #rivz열기
$ vscode #vscode열기

#%% [7] PARAMETER
'[7-1] 파라미터 활용한 노드작성 - pub'

$ roscd ros_tutorials_service/src #→ 패키지의 소스 코드 폴더인 src 폴더로 이동
$ gedit parameter_publisher.py #→ 소스 파일 내용 수정

''''''''''''''''''''''''''''''''''''''''''''''''''''''''
import rospy
from std_msgs.msg import String

rospy.init_node('parameter_publisher') # 초기화
pub = rospy.Publisher('my_topic', String, queue_size=10) # 토픽명, 타입, 사이즈

# Get the value of the parameter 'my_param' with default value 'Hello World'
my_param_value = rospy.get_param('my_param', 'Hello World') # 파라미터명, 디폴트값

while not rospy.is_shutdown():
    pub.publish(my_param_value) # 지정하지 않으면 Hello World를 발행함
    rospy.sleep(1.0)
''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#%%
'[7-2] 파라미터 활용한 노드작성 - sub'
$ roscd ros_tutorials_service/src #→ 패키지의 소스 코드 폴더인 src 폴더로 이동
$ gedit parameter_subscriber.py #→ 소스 파일 내용 수정

''''''''''''''''''''''''''''''''''''''''''''''''''''''''
import rospy
from std_msgs.msg import String

def callback(data): # data는 pub에서 publish한 값이고 그 데이터는 data.data로 받을 수 있다.
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

rospy.init_node('parameter_subscriber')
sub = rospy.Subscriber('my_topic', String, callback) # 토픽명, 타입, 콜백함수

# Get the value of the parameter 'my_param' with default value 'Hello World'
# If you need to use the parameter value in the subscriber node, write this line
my_param_value = rospy.get_param('my_param', 'Hello World') # 제거 가능

rospy.spin() #call back함수 호출될 때까지 대기. callback은 pub으로 토픽 날라오면 호출됨.
''''''''''''''''''''''''''''''''''''''''''''''''''''''''

#%%
'[7-3] 파라미터 입력'
$ rosparam set my_param "Hello World" 
# 뒤에 데이터타입도 지정하면 좋단다 rosparam set my_string "hello" string
# 설정되는 값이 숫자로 시작하는 문자열인 경우 유형이 명시적으로 지정되지 않으면 정수 또는 실수로 해석될 수 있습니다. 
#%%
'[7-4] 노드실행시 파라미터 입력'
$ rosrun my_package my_node _param1:=10 _param2:=hello # _ 언더바 써줘야 함

#%%
'[7-5] 파라미터 검색 및 초기화'
$ rosparam get my_param # 현재 설정된 파라미터값 검색
$ rosparam delete my_param # -> 디폴트값으로 바뀌게 됨.

$ rosparam list

#%% [8] LAUNCH
'[8-1] launch 파일 생성'
$ roscd ros_tutorials_topic
$ mkdir launch
$ cd launch
$ gedit union.launch

# 1:N    
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
<launch>
<node pkg="ros_tutorials_topic" type="topic_publisher" name="topic_publisher1"/>
<node pkg="ros_tutorials_topic" type="topic_subscriber" name="topic_subscriber1"/>
<node pkg="ros_tutorials_topic" type="topic_publisher" name="topic_publisher2"/>
<node pkg="ros_tutorials_topic" type="topic_subscriber" name="topic_subscriber2"/>
</launch> 
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
# 1:1
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
<launch>
<group ns="ns1">
<node pkg="ros_tutorials_topic" type="topic_publisher" name="topic_publisher"/>
<node pkg="ros_tutorials_topic" type="topic_subscriber" name="topic_subscriber"/> 
</group>
<group ns="ns2">
<node pkg="ros_tutorials_topic" type="topic_publisher" name="topic_publisher"/>
<node pkg="ros_tutorials_topic" type="topic_subscriber" name="topic_subscriber"/>
</group>
</launch>
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

#%%
'[8-2] launch로 여러 노드 실행'
$ roslaunch ros_tutorials_topic union.launch --screen # 실행되는 모든 노드들의 출력들이 터미널 스크린에 표시
