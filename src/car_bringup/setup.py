from setuptools import setup


package_name = "car_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            ["config/bringup.yaml", "config/ydlidar_car.yaml"],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/robot_bringup.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS 2 Car Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS2小车统一启动包",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "check_topics = car_bringup.check_topics:main",
            "laser_tcp_server = car_bringup.laser_tcp_server:main",
            "lidar_supervisor = car_bringup.lidar_supervisor:main",
            "wait_for_chassis = car_bringup.wait_for_chassis:main",
        ]
    },
)
