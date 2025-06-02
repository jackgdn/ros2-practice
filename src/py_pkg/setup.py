from setuptools import setup

package_name = "py_pkg"

setup(
    name="py_pkg",
    version="0.0.0",
    packages=["py_pkg"],
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
            "publisher = py_pkg.publisher:main",
            "subscriber = py_pkg.subscriber:main",
            "server = py_pkg.server:main",
            "client = py_pkg.client:main",
            "parameter = py_pkg.parameter:main",
        ],
    },
)
