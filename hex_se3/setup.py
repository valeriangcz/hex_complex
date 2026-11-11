from setuptools import find_packages, setup


setup(
    name="hex_se3",
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
)
