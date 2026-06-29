from setuptools import setup

package_name = "hsi_system_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrew Roberts",
    maintainer_email="a.roberts18@newcastle.ac.uk",
    description="System-level controller nodes for HSI platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_commands_controller = hsi_system_controller.camera_commands_controller:main",
        ],
    },
)
