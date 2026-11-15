from setuptools import setup

package_name = "hydrofoil_control"
setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "parity_node = hydrofoil_control.parity_node:main",
        "control_node = hydrofoil_control.control_node:main",
        "env_node = hydrofoil_control.env_node:main",
        "hf = hydrofoil_control.hf_cli:main",
    ]},
)
