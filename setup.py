from setuptools import find_packages, setup

package_name = 'dshw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='g.solis.gabriel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "apf_nav = dshw.apf.apf_nav:main",
            "mock_pub = dshw.apf.mock_pub:main",
            "gauge_pub = dshw.gauge.gauge_pub:main",
        ],
    },
)
