from distutils.core import setup

setup(
    version = '0.0.1',
    scripts = ['src/smart_motor_driver.py']
    packages = ['smart_motor']
    package_dir = {'':'src'}
)
