from setuptools import setup

package_name = 'visual_collision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haochen',
    maintainer_email='hao.chen@okstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_collision = visual_collision.visual_collision:main',
            'test_publisher   = visual_collision.test_publisher:main'
        ],
    },
)
