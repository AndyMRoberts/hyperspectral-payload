from setuptools import setup

package_name = "system_profiler"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/hook",
            [
                "hook/ament_prefix_path.dsv",
                "hook/ament_prefix_path.ps1",
                "hook/ament_prefix_path.sh",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrew Roberts",
    maintainer_email="a.roberts18@newcastle.ac.uk",
    description="For monitoring on board power and memory usage.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_profiler = system_profiler.system_profiler:main",
        ],
    },
)
