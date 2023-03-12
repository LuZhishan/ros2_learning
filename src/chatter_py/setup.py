from setuptools import setup
from setuptools import find_packages

package_name = 'chatter_py'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'launch',   # 添加launch需要
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='123@456.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pub = chatter_py.pub:main', # exe = package_name.file_name:main
            'sub = chatter_py.sub:main',
        ],
    },
)
