from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['uav_base'],
    package_dir={'': 'src'}
)
setup(**d)
