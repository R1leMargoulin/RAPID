from setuptools import setup, find_packages

setup(
    name='RAPID',
    version='0.5',
    packages=find_packages(),
    description='Robotics Agent Prototyping for Intelligence Development',
    author='Erwan MARTIN',
    author_email='erwan.martin666@gmail.com',
    install_requires=[
        'numpy',
        'pygame',
        'pillow',
    ],
)