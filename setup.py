import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyKoalaRemote",
    version="1.0.0",
    author="Software Team: J. Parent - F. Mendels - T. Colomb",
    author_email="software@lynceetec.com",
    description="Python wrapper for dotNet Koala Remote Client provided by LyncéeTec to control Digital Holographique Microscope using proprietary Koala software",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/jejmule/pyKoalaRemote",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Microsoft :: Windows",
    ],
)