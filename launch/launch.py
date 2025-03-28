from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tableorder',            # 패키지 이름
            executable='table_client',            # 실행 파일 이름
            name='table_order_client',                  # 노드 이름
            output='screen',                 # 출력 설정 (화면에 출력)
        ),
    ])