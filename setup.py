from setuptools import find_packages, setup
from glob import glob
package_name = 'ur_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name+"/launch", glob("launch/*.launch.py")),
        ("share/" + package_name+"/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name+"/urdf", glob("urdf/*.gazebo")),
        ("share/" + package_name+"/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + '/config', glob("config/*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='staszak.raf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        # "console_scripts": [
        #     "publisher_joint_trajectory_controller = \
        #         ur_py_control.publisher_joint_trajectory_controller:main",
        #     'show_joint_states = ur_py_control.show_joint_states:main',
        #     "publisher_joint_trajectory_controller_IK = \
        #         ur_py_control.publisher_joint_trajectory_controller_IK:main",
        # ],
        "console_scripts": [
            "detect_shapes = ur_simulation.detect_shapes:main",
            "closest_objects = ur_simulation.closest_objects:main",
            "reach_pose = ur_simulation.reach_pose:main",
        ],
    },
)
