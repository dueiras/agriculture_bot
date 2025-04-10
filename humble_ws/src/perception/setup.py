from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo Eiras',
    maintainer_email='dueiras@gmail.com',
    description='Visual Servoing navigation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'visual_servoing = perception.visual_servoing:main',
            'visual_navigation = perception.local_visual_navigation:main'
        ],
    },
)
