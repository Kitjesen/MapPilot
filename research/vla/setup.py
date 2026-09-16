from setuptools import setup, find_packages

package_name = "vla_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["vla_nav/ros2/launch/vla_nav.launch.py"]),
        ("share/" + package_name + "/config", ["config/vla_nav.yaml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "torch",
        "torchvision",
        "transformers>=4.40.0",
        "accelerate",
        "peft>=0.10.0",
        "scipy",
        "opencv-python",
        "Pillow",
    ],
    zip_safe=True,
    maintainer="3dNAV Team",
    maintainer_email="robot@3dnav.dev",
    description="VLA end-to-end navigation with AdaCoT + VLingMem (VLingNav architecture)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vla_nav_node = vla_nav.ros2.vla_nav_node:main",
        ],
    },
)
