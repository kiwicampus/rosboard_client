from setuptools import setup

package_name = "rosboard_desktop_client"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ada",
    maintainer_email="pedro.gonzalez@eia.edu.co",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rosboard_client = rosboard_desktop_client.rosboard_client:main",
            "rosboard_client_gui = rosboard_desktop_client.rosboard_client_gui_qt:main",
            "rosboard_benchmark = rosboard_desktop_client.benchmarks.rosboard_benchmark:main",
            "rosbridge_benchmark = rosboard_desktop_client.benchmarks.rosbridge_benchmark:main",
        ],
    },
)
