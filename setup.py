import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyransac3d", # Replace with your own username
    version="0.0.1",
    author="Leonardo Mariga",
    author_email="leomariga@gmail.com",
    description="A python tool for fitting primitives 3D shapes in point clouds using RANSAC algorithm ",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/leomariga/pyRANSAC-3D",
    packages=setuptools.find_packages(),
    keywords='truth table python generation logic pandas dataframe',
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
)
