from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['ids_coordinates_setter'],
    package_dir={'': 'src'}
)
setup(**d)
