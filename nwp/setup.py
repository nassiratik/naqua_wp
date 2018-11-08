from setuptools import setup

def readme():
    with open('README.rst') as f:
        return f.read()

setup(name = 'nwp_controller',
      version = '1.1',
      description='Raspberry Pi application to manage reading and transmitting water parameters',
      long_description='Initially, this project covers Fish PGO. Raspberry Pi controller unit is installed for each PGO tank',
      classifiers = [
          'development status :: 1 - Beta',
          'License :: Naqua :: ITD',
          'Programming Language :: Python :: 3.6',
          'Topic :: Naqua :: Fish Production'
          ],
      url='http://gitlab.naqua.com.sa/nassiratik/nwp_controller.git',
      author='Nassir Atik',
      author_email='nassiratik@naqua.com.sa',
      licence="IT",
      packages=['nwp_controller'],
      install_requires=[
                    ],
      include_package_data=True,
      zip_safe=False,
      )
