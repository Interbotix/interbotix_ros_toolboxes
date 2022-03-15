from setuptools import setup

package_name = "interbotix_common_modules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/{}".format(package_name), ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "geometry_msgs"],
    zip_safe=True,
    maintainer="Luke Schmitt",
    maintainer_email="luke@trossenrobotics.com",
    description="The interbotix_common_modules package",
    license="BSD",
    entry_points={}
)
