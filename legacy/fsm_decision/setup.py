from setuptools import setup, find_packages

package_name = 'fsm_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='FSM decision node for SAUVC AUV',
    entry_points={
        'console_scripts': [
            'fsm_node = fsm_decision.fsm_node:main',
        ],
    },
)
