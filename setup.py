from setuptools import find_packages, setup

package_name = "crazyflie_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="air",
    maintainer_email="yongce.liu@outlook.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ergodic_search=crazyflie_control.ergodic_search:main",
            "micp=crazyflie_control.micp:main",
            "espc=crazyflie_control.espc:main",
        ],
    },
)
