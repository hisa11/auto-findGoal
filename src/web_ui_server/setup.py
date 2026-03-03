import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_ui_server'

def collect_web_files(web_root):
    """webディレクトリ以下のすべてのファイルをインストールリストに収集する"""
    data_files = []
    for dirpath, dirnames, filenames in os.walk(web_root):
        # .git などは除外
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_dir, files))
    return data_files

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'certs'), glob('certs/*')),
]
data_files += collect_web_files('web')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hisa',
    maintainer_email='hisahisa1120@outlook.jp',
    description='LAN内制御用Webアプリ配信サーバー (pixel-landscape-app)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'server_node = web_ui_server.server_node:main',
        ],
    },
)
