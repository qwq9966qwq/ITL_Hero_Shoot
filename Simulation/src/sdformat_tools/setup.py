from setuptools import setup

package_name = 'sdformat_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Zhenpeng Ge',
    author_email='zhenpeng.ge@qq.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='sdformat tools.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xmacro4sdf = sdformat_tools.xmacro4sdf:xmacro4sdf_main',
            'sdf2urdf = sdformat_tools.sdf2urdf:sdf2urdf_main',
        ]
    },
)