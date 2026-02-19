from setuptools import setup
import os

package_name = 'rfs_evaluator_app'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(directory):
    for (path, directories, filenames) in os.walk(directory):
        if filenames:
            paths = [os.path.join(path, filename) for filename in filenames]
            target = os.path.join('share', package_name, path)
            data_files.append((target, paths))

package_files('static')
package_files('templates')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taichi Hirano',
    maintainer_email='hiranotaichi8@gmail.com',
    description='FACES-IV Validation Web App',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rfs_evaluator_app = rfs_evaluator_app.app:main',
        ],
    },
)
