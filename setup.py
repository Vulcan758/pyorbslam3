from setuptools import setup

# The _core extension is built separately with ./build.sh (CMake) and dropped
# into the package directory; this setup only makes the package importable
# (pip install -e .).
setup(
    name="pyorbslam3",
    version="0.1.0",
    description="Python bindings for ORB-SLAM3 (inertial + non-inertial)",
    packages=["pyorbslam3"],
    package_data={"pyorbslam3": ["*.so"]},
    install_requires=["numpy"],
    python_requires=">=3.8",
)
