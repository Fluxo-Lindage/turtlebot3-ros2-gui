#!/usr/bin/env python3
"""
robot_gui 包安装脚本

安装方式:
    python3 setup.py install --user
    或
    pip3 install -e . --user
"""

from setuptools import setup, find_packages
import os


def get_data_files():
    """收集包内数据文件（YAML 配置等）"""
    data_files = []
    # 遍历 robot_gui 包下的所有目录
    for root, dirs, files in os.walk('robot_gui'):
        # 跳过 __pycache__ 目录
        dirs[:] = [d for d in dirs if d != '__pycache__']
        for f in files:
            if f.endswith('.yaml'):
                full_path = os.path.join(root, f)
                # 保持目录结构
                dest_dir = os.path.dirname(full_path)
                data_files.append((dest_dir, [full_path]))
    return data_files


setup(
    name='robot_gui',
    version='1.0.0',
    description='基于 ROS2 + PyQt5 的机器人仿真控制 GUI 平台',
    author='Robot Student',
    author_email='student@example.com',
    license='Apache License 2.0',
    packages=find_packages(),
    package_data={
        'robot_gui.config': ['*.yaml'],
    },
    include_package_data=True,
    python_requires='>=3.8',
    install_requires=[
        'PyQt5>=5.15',
    ],
    entry_points={
        'console_scripts': [
            'robot_gui = robot_gui.main:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Education',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: User Interfaces',
    ],
)
