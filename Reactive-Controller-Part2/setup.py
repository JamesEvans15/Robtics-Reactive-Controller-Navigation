from setuptools import setup
import glob

package_name = 'project_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sim = project_4.sim:sim_main",
            "vel = project_4.sim:vel_main",
            "nav = project_4.nav:nav_main",
            "translator = project_4.nav:translator_main"
        ],
    },
)
