from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = "hydra_tic_tac_toe"

setup_args = generate_distutils_setup(
    packages=[package_name],
    package_dir={"": "src"},
)

setup(**setup_args)
