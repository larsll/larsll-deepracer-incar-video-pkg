#################################################################################
#   Copyright Lars Ludvigsen.                     All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

from setuptools import setup
import os

package_name = "incar_video_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), ["launch/incar_video_pkg_launch.py"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/Amazon_Ember_Bd.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/Amazon_Ember_BdIt.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/Amazon_Ember_Lt.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/Amazon_Ember_LtIt.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/Amazon_Ember_Rg.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/Amazon_Ember_RgIt.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/AmazonEmber_Th.ttf"]),
        (os.path.join("share", package_name, "AmazonEmber"), ["resource/AmazonEmber/AmazonEmber_ThIt.ttf"]),
        (os.path.join("share", package_name, "images"), ["resource/images/DRL_video_oa_overlay_league_leaderboard.png"])
    ],
    install_requires=["setuptools","pillow"],
    zip_safe=True,
    maintainer="Lars Ludvigsen",
    maintainer_email="lars@ludvig.no",
    description="This package contains a video editing node that adds additional information to the in-car video stream.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "incar_video_edit_node = incar_video_pkg.incar_video_edit_node:main",
            "incar_video_capture_node = incar_video_pkg.incar_video_capture_node:main",
        ],
    },
)
