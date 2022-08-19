from setuptools import setup

package_name = "rosboard_client"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kiwibot, Inc",
    maintainer_email="pedro.gonzalez@eia.edu.co",
    description="Package to connect to a rosboard server and republish them locally using ros2",
    license="GNU v3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rosboard_client = rosboard_client.rosboard_client:main",
            "rosboard_client_gui = rosboard_client.rosboard_client_gui:main",
            "rosboard_benchmark = rosboard_client.benchmarks.rosboard_benchmark:main",
            "rosbridge_benchmark = rosboard_client.benchmarks.rosbridge_benchmark:main",
        ],
    },
)
