from glob import glob

from setuptools import find_packages, setup

package_name = "lingtu_rmf_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LingTu Engineering",
    maintainer_email="engineering@inovxio.local",
    description="Optional Open-RMF EasyFullControl sidecar for LingTu.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lingtu_rmf_adapter = lingtu_rmf_adapter.adapter:main",
        ],
    },
)
