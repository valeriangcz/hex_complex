from setuptools import find_packages, setup


setup(
    name="hex_isaacgym",
    version="0.2.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
)
