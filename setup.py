from setuptools import setup, find_packages

setup(
    name="multispectral_cam_agri",
    version="0.1.0",
    author="Your Name",
    author_email="your_email@example.com",
    description="A multispectral camera system for agricultural applications.",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/your-repo-url",  # 替换为你的项目URL
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    include_package_data=True,
    install_requires=[
        # 在这里列出你的依赖包
        "numpy",
        "opencv-python",
        "matplotlib",
        "scipy",
        "pandas",
        "open3d"
    ],
    python_requires=">=3.7",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
