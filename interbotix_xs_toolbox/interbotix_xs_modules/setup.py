from setuptools import setup

package_name = "interbotix_xs_modules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            [f"resource/{package_name}"]
        ),
        (f"share/{package_name}",
            ["package.xml"]
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Luke Schmitt",
    maintainer_email="luke@trossenrobotics.com",
    description="The interbotix_xs_modules package",
    license="BSD",
    entry_points={},
)
