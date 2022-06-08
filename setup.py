import setuptools
import os
import numpy

BASEDIR  = os.path.abspath(os.path.dirname(__file__))
CPP_FILE = os.path.join(BASEDIR, 'CPP_files', 'pyransac.cpp')

module = setuptools.Extension("pyransac3d_wrapper", sources = [CPP_FILE], include_dirs=[numpy.get_include()])

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyransac3d", # Replace with your own username
    version="0.6.1",
    author="Leonardo Mariga",
    author_email="leomariga@gmail.com",
    description="A python tool for fitting primitives 3D shapes in point clouds using RANSAC algorithm ",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/leomariga/pyRANSAC-3D",
    packages=setuptools.find_packages(),
    keywords='point-cloud,segmentation,ransac,cuboid,3d-reconstruction,cylinder,planes,open3d,plane-detection,ransac-algorithm ',
    project_urls={
        'Documentation': 'https://leomariga.github.io/pyRANSAC-3D/',
        'Source': 'https://github.com/leomariga/pyRANSAC-3D',
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    ext_modules=[module]
)
