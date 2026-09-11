from setuptools import setup

package_name = "drone_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mission.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmad Shahrour",
    maintainer_email="ashahrourr@gmail.com",
    description="LLM mission planner, safety guard and vehicle simulator as ROS 2 nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim_node = drone_agent.sim_node:main",
            "guard_node = drone_agent.guard_node:main",
            "planner_node = drone_agent.planner_node:main",
        ],
    },
)
