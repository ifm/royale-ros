#!/usr/bin/env python
# -*- python -*-

#
# Copyright (C)  2017 Love Park Robotics, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distribted on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#
# Creates a deb file from `libroyale.zip` for easy integration of the Royale
# SDK with the packaging system on Debian-based systems like Ubuntu.
#

from __future__ import print_function
import argparse
import fnmatch
import os
import shutil
import stat
import sys
import tempfile

ARCH_LUT = \
  {
      "x86_64": "amd64",
      "x86_32": "amd32",
      "arm_32": "armhf"
  }

class NoValidSDK(Exception):
    pass

class RoyaleDebMaker(object):
    """Class for making a deb file from a Royale SDK"""

    def __init__(self, infile, prefix, arch, full):
        self.infile_ = infile
        self.prefix_ = prefix
        self.arch_ = arch
        self.full_ = full
        self.outdir_ = os.getcwd()

        self.tmp_ = None # temporary working directory
        self.zip_dir_ = None # where we unzip libroyale.zip
        self.target_zip_file_ = None # the arch-specific zip file to debianize

        self.deb_dir_ = None # root of deb file directory structure
        self.sdk_dir_ = None # full path to extracted sdk dir

        # Royale SDK Version
        self.royale_version_ = {'major': 0, 'minor': 0, 'patch': 0, 'rev': 0}

    def unzip(self, inzip, outdir, files=None):
        """
        Unzips the passed in `inzip` to the directory `outdir`

        NOTE: Python's `zipfile' module is very limited in that permissions and
        symlinks are lost. Let's just use the Unix shell.
        """
        if files is None:
            cmd = "unzip -q %s -d %s" % (inzip, outdir)
        else:
            cmd = "unzip -q %s %s -d %s" % (inzip, files, outdir)
        print(cmd)
        os.system(cmd)

    def find_target_zip(self, indir):
        """
        Based on the passed in directory, find the Royale-supplied
        zip file that we want to debianize. We assume it is in `indir`.
        """
        candidates = \
         [f for f in os.listdir(indir) if fnmatch.fnmatch(f, "*.zip")]
        print("All available SDK's include: %s" % candidates)

        # filter because we are on linux
        linux_sdks = [x for x in candidates if "LINUX" in x]
        print("The valid LINUX SDK's include: %s" % linux_sdks)

        # filter based on architecture
        cpu, bits = self.arch_.split("_")
        royale_arch = "%s-%sBit" % (cpu, bits)
        sdk = [x for x in linux_sdks if royale_arch in x]
        if len(sdk) != 1:
            raise NoValidSDK("Could not find a valid sdk in: %s" % sdk)

        print("Found target SDK: %s" % sdk[0])
        return sdk[0]

    def get_royale_version(self, fname):
        """
        Extracts the Royale version from the zip file name
        """
        fname_components = fname.split("-")
        if len(fname_components) < 2:
            raise NoValidSDK("Could not find version string in: %s" % fname)

        version_list = \
          [s for s in fname_components[1].split(".") if s.isdigit()]
        if len(version_list) != 4:
            raise NoValidSDK("Could not find version string in: %s" % fname)

        return tuple(version_list)

    def extract_sdk_to_deb_dir(self, inzip, deb_dir, full=False):
        """
        Creates the debian file directory structure and unpacks the SDK into
        it.
        """
        # tack on the prefix
        prefix = self.prefix_
        if prefix.startswith('/'):
            prefix = prefix[1:]

        path = "%s/%s" % (deb_dir, prefix)
        os.makedirs(path)
        if full:
            print("Extracting full SDK...")
            self.unzip(inzip, path)
        else:
            print("Extracting minimal SDK...")
            self.unzip(inzip, path,
                       "%s %s %s %s %s %s %s %s" % \
                       (
                        "libroyale-*/driver/*",
                        "libroyale-*/share/*",
                        "libroyale-*/include/royale.hpp",
                        "libroyale-*/include/royale/*",
                        "libroyale-*/include/libuvc/*",
                        "libroyale-*/bin/libroyale.so*",
                        "libroyale-*/bin/libspectre*.so",
                        "libroyale-*/bin/libuvc.so"
                       ))
        return "%s/%s/%s" % (deb_dir, prefix, os.listdir(path)[0])

    def create_debian_control_info(self):
        """
        Creates the relevant debian control file information
        """
        control_dir = "%s/DEBIAN" % self.deb_dir_
        os.makedirs(control_dir)

        # NOTE: We assume it is self-contained (i.e., we do not express
        # dependencies ... maybe not a good assumption??)
        control_txt ="""
Package: royale-sdk
Version: %s.%s.%s-%s
Section: Libraries
Priority: optional
Architecture: %s
Maintainer: Love Park Robotics, LLC <info@loveparkrobotics.com>
Description: Debianized version of the pmd Royale SDK
""" % (
    self.royale_version_["major"],
    self.royale_version_["minor"],
    self.royale_version_["patch"],
    self.royale_version_["rev"],
    ARCH_LUT[self.arch_]
    )

        symlink_dir = "%s/royale" % self.prefix_
        sdk_dir = "%s/%s" % (self.prefix_, os.path.basename(self.sdk_dir_))
        postinst_txt ="""
#!/bin/sh

if [ -e %s ]; then
  echo "Removing: %s"
  rm -f %s
fi

if [ -d %s ]; then
  echo "Creating new symlink: %s -> %s"
  ln -s %s %s
fi
""" % (
    symlink_dir,
    symlink_dir,
    symlink_dir,
    sdk_dir,
    symlink_dir,
    sdk_dir,
    sdk_dir,
    symlink_dir
    )

        prerm_txt ="""
#!/bin/sh

if [ -e %s ]; then
  echo "Removing: %s"
  rm -f %s
fi
""" % (
    symlink_dir,
    symlink_dir,
    symlink_dir
    )

        print("Creating control file...")
        with open("%s/control" % control_dir, "w") as control_file:
            control_file.write(control_txt)

        print("Creating postinst shell script...")
        with open("%s/postinst" % control_dir, "w") as postinst_file:
            postinst_file.write(postinst_txt)

        print("Creating prerm shell script...")
        with open("%s/prerm" % control_dir, "w") as prerm_file:
            prerm_file.write(prerm_txt)

        # chmod +x <postinst>
        postinst_file = "%s/postinst" % control_dir
        st = os.stat(postinst_file)
        os.chmod(postinst_file,
                 stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR |
                 stat.S_IRGRP | stat.S_IWGRP | stat.S_IXGRP |
                 stat.S_IROTH | stat.S_IXOTH)

        # chmod +x <prerm>
        prerm_file = "%s/prerm" % control_dir
        st = os.stat(prerm_file)
        os.chmod(prerm_file,
                 stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR |
                 stat.S_IRGRP | stat.S_IWGRP | stat.S_IXGRP |
                 stat.S_IROTH | stat.S_IXOTH)

        # Need to properly install the udev rules file
        print("Creating udev rules file...")
        rules_file = "%s/driver/udev/10-royale-ubuntu.rules" % self.sdk_dir_
        if os.path.isfile(rules_file):
            os.makedirs("%s/etc/udev/rules.d" % self.deb_dir_)
            shutil.copyfile(rules_file,
                            "%s/etc/udev/rules.d/10-royale-ubuntu.rules" %
                            self.deb_dir_)

    def run(self):
        """Runs the deb creation pipeline"""
        # create a temporary working directory
        self.tmp_ = tempfile.mkdtemp(prefix='debianize-royale-')
        print("Working directory is: %s" % self.tmp_)

        # unzip the libroyale.zip file to working area
        self.zip_dir_ = "%s/unzipped" % self.tmp_
        self.unzip(self.infile_, self.zip_dir_)

        # figure out which architecture-specific zip we should extract
        self.target_zip_file_ = self.find_target_zip(self.zip_dir_)

        # get the version of royale and cache it
        self.royale_version_ = \
          dict(zip(('major', 'minor', 'patch', 'rev'),
                    self.get_royale_version(self.target_zip_file_)))
        print("Parsed out Royale version as: %s" % self.royale_version_)

        # set the root of our deb file structure
        self.deb_dir_ = "%s/deb" % self.tmp_
        print("Working deb dir is: %s" % self.deb_dir_)

        # extract SDK to debian package file structure
        self.sdk_dir_ = \
          self.extract_sdk_to_deb_dir("%s/%s" %
                                        (self.zip_dir_, self.target_zip_file_),
                                        self.deb_dir_, self.full_)
        print("SDK extracted to: %s" % self.sdk_dir_)

        # create the debian control information
        self.create_debian_control_info()

        # build the deb file
        deb_file_name = "royale-sdk_%s.%s.%s-%s_%s.deb" % \
          (
              self.royale_version_["major"],
              self.royale_version_["minor"],
              self.royale_version_["patch"],
              self.royale_version_["rev"],
              ARCH_LUT[self.arch_]
          )
        dpkg_cmd = "dpkg -b %s %s" % (self.deb_dir_, deb_file_name)
        print("%s" % dpkg_cmd)
        os.system(dpkg_cmd)

        # clean up our mess
        if self.tmp_ is not None:
            print("Removing tmp directory: %s" % self.tmp_)
            shutil.rmtree(self.tmp_)

        return 0

def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='Create a deb file for the Royale SDK from libroyale.zip')

    prefix_help="""
Root directory for the deployed Royale SDK.
Specifically, the SDK will be installed in
`<prefix>/libroyale-<version>-LINUX-<arch>' and a
symlink will be be created such that:

`<prefix>/royale -> <prefix>/libroyale-<version>-LINUX-<arch>'

Typical choices for `--prefix' are `/opt' or `/usr/local'.
"""

    parser.add_argument('--infile', required=True, type=str,
                        help='Input libroyale.zip file')
    parser.add_argument('--prefix', type=str, default='/opt',
                        help=prefix_help)
    parser.add_argument('--arch', type=str,
                        choices=set(("x86_64", "x86_32", "arm_32")),
                        default="x86_64",
                        help='Architecture for target deb file')
    parser.add_argument('--full', action='store_true', default=False,
                        help='Debianize the full SDK')

    args = parser.parse_args(sys.argv[1:])
    return args

def main():
    args = get_args()
    deb = RoyaleDebMaker(args.infile, args.prefix, args.arch, args.full)
    return deb.run()

if __name__ == '__main__':
    sys.exit(main())
