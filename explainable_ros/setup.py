from setuptools import find_packages, setup

package_name = "explainable_ros"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Sobrín-Hidalgo",
    maintainer_email="dsobh@unileon.es",
    description="Explainability tool for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explainability_node = explainable_ros.explainability_node:main",
            "explainability_client_node = explainable_ros.explainability_client_node:main",
            "vexp_node = explainable_ros.vexp_node:main",
        ],
    },
)
