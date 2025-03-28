from setuptools import setup, find_packages

package_name = 'tableorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['tableorder', 'tableorder.*']),
    data_files=[
        # 패키지 리소스 등록
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 서비스 파일 등록
        ('share/' + package_name + '/srv', ['srv/MenuOrder.srv']),
        ('share/' + package_name + '/ui', ['tableorder/ui/ui_counter_dp.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # 유지관리자의 이름
    maintainer_email='your.email@example.com',  # 유지관리자의 이메일
    description='Menu order service example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행 가능한 스크립트 등록
            'table_server = tableorder.table_server:main',
            'table_client = tableorder.table_client:main',
            'counter_node = tableorder.counter_nodev2:main',
            'counter_node2 = tableorder.counter_nodev3:main',
            'counter_node3 = tableorder.counter_nodev4:main',
            'counter_node4 = tableorder.counter_nodev6:main',
        ],
    },
)
