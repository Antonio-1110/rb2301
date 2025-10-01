from setuptools import find_packages, setup
from glob import glob

package_name = 'rb2301_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/params", glob('params/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmi-002',
    maintainer_email='e1482086@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tut = rb2301_tutorial.tutorial:main',
            'fake = rb2301_tutorial.fake:main',
            'logger = rb2301_tutorial.logger:main',
            'recorder = rb2301_tutorial.recorder:main',
            'pubs = rb2301_tutorial.publishers:main',
            'subs = rb2301_tutorial.subscribers:main',
            'prms = rb2301_tutorial.wk6:main',
            'prms_srvs = rb2301_tutorial.wk6hw:main',
            'sim = rb2301_tutorial.sim:main'
        ],
    },
)
