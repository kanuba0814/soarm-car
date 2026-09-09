import os

from setuptools import setup


setup(
    name="soarm_named_place_request",
    version="0.1.0",
    packages=["soarm_named_place_request"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/soarm_named_place_request"]),
        (os.path.join("share", "soarm_named_place_request"), ["package.xml", "asset_identity_lock.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["prepare_named_place_request = soarm_named_place_request.prepare:main"]},
)
