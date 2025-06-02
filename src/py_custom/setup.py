from setuptools import setup

package_name = "py_custom"

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
    maintainer="jackgdn",
    maintainer_email="li_zhong_yao@foxmail.com",
    description="[TODO]: Package description",
    license="[TODO]: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coordinate_publisher = py_custom.publisher:main",
            "coordinate_subscriber = py_custom.subscriber:main",
            "transformation_server = py_custom.server:main",
            "transformation_client = py_custom.client:main",
        ],
    },
)
