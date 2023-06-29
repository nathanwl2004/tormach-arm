# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Lukas Sackewitz
#
# Originally from Universal_Robots_ROS2_Description distro, updates and
# improvements for Tormach by jmorris@tormach

import os
import sys
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    description_package = "za6_description"
    description_file = "za6.xacro"
    prefix = ""

    description_package_dir = get_package_share_directory(description_package)
    description_file_path = os.path.join(
        description_package_dir, "urdf", description_file
    )

    with tempfile.NamedTemporaryFile() as fp:
        xacro_command = [
            shutil.which('xacro'),
            description_file_path,
            "-o",
            fp.name,
            f"prefix:={prefix}",
        ]
        xacro_process = subprocess.run(xacro_command, stdout=fp)
        assert xacro_process.returncode == 0, " --- XACRO command failed ---"
        for line in fp:
            sys.stdout.write(line.decode())
        check_urdf_command = [shutil.which('check_urdf'), fp.name]
        print(f"check_urdf_command:  '{' '.join(check_urdf_command)}'")
        check_urdf_process = subprocess.run(check_urdf_command, stdin=fp)

    assert check_urdf_process.returncode == 0, "URDF check failed!"


if __name__ == "__main__":
    test_urdf_xacro()
