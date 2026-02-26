from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="gwct",
    version="0.1.0",
    description="Gowin Command Line Tool",
    long_description=long_description,
    long_description_content_type="text/markdown",
    py_modules=["gwct", "gwct_server"],
    install_requires=[
        "pyserial==3.5",
    ],
    entry_points={
        "console_scripts": [
            "gwct = gwct:main",
            "gwct_server = gwct_server:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
