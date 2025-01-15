from setuptools import find_packages, setup
# from glob import glob

package_name = 'lawn_mower_web_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('resource', glob('html/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivankudinov',
    maintainer_email='ikudinov@ccsteam.ru',
    description='Web UI controller',
    license='BSD-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ws_node = lawn_mower_web_controller.ws_node:main',
            'web_node = lawn_mower_web_controller.web_node:main',
        ],
    },
)
