from setuptools import find_packages, setup

package_name = 'control_X'

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
    maintainer='shambhuraj',
    maintainer_email='shambhuraj.careers@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_subscriber = control_X.fk_subscriber:main',
            'ik_subscriber = control_X.ik_subscriber:main',      
            
  
            'ik_service = control_X.ik_service:main',
            'pd_service = control_X.pd_service:main',
            'spawn = control_X.spawn_robot:main',
            'fk_graph = control_X.fk_graph:main',
            
            'jk_service = control_X.jk_service:main',
            'increamental = control_X.increamental:main',
            'reference_pub = control_X.reference_pub:main',
            
            
            
            
            
        
        ],
    },
)
