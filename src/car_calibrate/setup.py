from setuptools import setup


package_name = "car_calibrate"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/calibrate.yaml"]),
        ("share/" + package_name + "/launch", ["launch/calibrate.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS 2 Car Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="差速小车自动动力学标定包",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calibrate_node = car_calibrate.calibration_node:main",
            "analyze_calibration = car_calibrate.analyze:main",
        ]
    },
)
