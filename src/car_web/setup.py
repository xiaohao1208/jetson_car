from setuptools import setup


package_name = "car_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/web.yaml"]),
        ("share/" + package_name + "/launch", ["launch/web.launch.py"]),
        ("share/" + package_name + "/templates", ["car_web/templates/index.html"]),
        (
            "share/" + package_name + "/static",
            [
                "car_web/static/app.js",
                "car_web/static/http.js",
                "car_web/static/style.css",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS 2 Car Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS2小车FastAPI网页控制桥接包",
    license="Apache-2.0",
    entry_points={"console_scripts": ["server = car_web.server:main"]},
)
