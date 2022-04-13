from setuptools import (
    setup,
    find_packages,
)

package_name = "interbotix_xs_modules"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        (f"share/{package_name}", ["package.xml"]),
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Luke Schmitt",
    maintainer_email="trsupport@trossenrobotics.com",
    description="The interbotix_xs_modules package",
    license="BSD",
    entry_points={},
)
