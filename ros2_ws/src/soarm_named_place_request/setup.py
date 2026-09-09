from setuptools import setup


setup(
    name="soarm_named_place_request",
    version="0.1.0",
    packages=["soarm_named_place_request"],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["prepare_named_place_request = soarm_named_place_request.prepare:main"]},
)
