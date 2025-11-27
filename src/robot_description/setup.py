from setuptools import find_packages, setup
import os
from glob import glob  # 파일 패턴 매칭을 위한 모듈

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 인덱스에 등록
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch 디렉터리의 모든 .py 파일을 설치 디렉터리로 복사
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # resource 디렉터리의 모든 .urdf 파일을 설치 디렉터리로 복사
        ('share/' + package_name + '/resource', glob('resource/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot description package',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 실행 가능한 스크립트가 있다면 여기에 추가
        ],
    },
)