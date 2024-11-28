from setuptools import setup

package_name = 'utils'

setup(
    name=utils,
    version='0.0.0',
    packages=['utils'],  # 如果utils是包的一部分
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)

