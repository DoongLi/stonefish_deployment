#!/usr/bin/env python3
# Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

import os
import re
import time
import rospy
from dataclasses import dataclass
from typing import List, Tuple
from git import Repo, Remote
from git.exc import InvalidGitRepositoryError
from git.util import IterableList
from std_msgs.msg import String
from cola2_ros.diagnostic_helper import DiagnosticHelper
from diagnostic_msgs.msg import DiagnosticStatus

COLA2_LIB_VERSION = re.compile(r'set\(PACKAGE_VERSION \"([\d\.]+)\"\)')  # i.e.: set(PACKAGE_VERSION "20.10.4")

def get_cola2_lib_version() -> str:
    """Get cola2_lib version from the default installation path."""
    # Look for col2 lib not installed first
    paths = [os.getenv('COLA2_LIB_DIR'), '/usr/local/share/cola2_lib', '/usr/share/cola2_lib']
    for base_filename in paths:
        if not base_filename:
            continue
        if base_filename.endswith('/'):
            base_filename = base_filename[:-1]
        rospy.loginfo(f"Searching for cola2 lib version in {base_filename}")
        filename = f'{base_filename}/COLA2_LIBConfigVersion.cmake'
        if os.path.isfile(filename):
            lines = open(filename).readlines()
            for line in lines:
                parsed = COLA2_LIB_VERSION.search(line)
                if parsed is not None:
                    rospy.loginfo(f"Found cola2 lib version in {base_filename}")
                    return parsed.group(1)
    rospy.loginfo(f"Cola2 lib version not found.")
    return 'cannot find it'


@dataclass
class Package:
    name: str  # repository name
    branch: str  # current branch
    commit_hash: str  # current commit hash
    commit_date: str  # current commit date
    changes: bool  # are there changes on current files?
    untracked: bool  # are there untracked files?
    remotes: IterableList[Remote]  # List of remotes

    def as_tuple(self) -> Tuple[str, str, str, str, str, str, str]:
        return (
            self.name,
            'C' if self.changes else ' ',
            'U' if self.untracked else ' ',
            self.branch,
            self.commit_date,
            self.commit_hash,
            ", ".join(
                [f"{remote.url} [{remote.name}]" for remote in self.remotes]),
        )


def get_packages(folder: str, only_iquarobotics_repos: bool) -> List[Package]:
    """Get iquarobotics git packages from the given folder."""
    packages = list()
    # all subfolders with full path under the folder
    subfolders = [os.path.join(folder, f) for f in os.listdir(
        folder) if os.path.isdir(os.path.join(folder, f))]
    subfolders.sort()
    for f in subfolders:
        # try to read data as a git repo
        try:
            if f.endswith('/'):
                f = f[:-1]
            repo = Repo(f)
            try:
                name = f.split('/')[-1]
            except Exception as e:
                print(">> {} error processing {}".format(f, e))
                continue
            if (not only_iquarobotics_repos) or (any(["iquarobotics" in url for url in [remote.url for remote in repo.remotes]]) and only_iquarobotics_repos):
                packages.append(
                    Package(
                        name=name,
                        branch=str(repo.head.reference),
                        commit_hash=str(repo.head.commit),
                        commit_date=time.strftime("%Y.%m.%d", time.gmtime(
                            repo.head.commit.committed_date)),
                        changes=len(repo.head.commit.diff(None)) > 0,
                        untracked=len(repo.untracked_files) > 0,
                        remotes=repo.remotes
                    )
                )
        except InvalidGitRepositoryError:
            pass
        except Exception as e:
            rospy.logwarn(f"Error processing repo {f}: {e}")
    return packages


def show_nice_table(cola2_lib_version: str, packages: List[Package]) -> str:
    # nice table
    toprint = list()
    toprint.append(('Repository', 'C', 'U', 'Branch',
                   'Date', 'Commit', 'Remotes'))
    for package in packages:
        toprint.append(package.as_tuple())

    # check row sizes
    sizes = [0, ] * len(toprint[0])
    for p in toprint:
        for i, v in enumerate(p):
            sizes[i] = max(sizes[i], len(v))

    # create format string
    fmt = ''
    for s in sizes:
        fmt += '{:' + str(s) + 's} '

    return f'cola2_lib: {cola2_lib_version}\n\n' + '\n'.join((fmt.format(*line) for line in toprint))


def get_version(only_iquarobotics_repos: bool) -> str:
    # cola2_lib
    cola2_lib_version = get_cola2_lib_version()

    # get catkin_ws/src folder
    folder, _ = os.path.split(os.path.abspath(__file__))
    catkin_src = os.path.abspath(os.path.join(folder, '../../..'))
    packages = get_packages(
        folder=catkin_src, only_iquarobotics_repos=only_iquarobotics_repos)
    packages.sort(key=lambda p: p.name)

    # show
    return show_nice_table(cola2_lib_version=cola2_lib_version, packages=packages)


diagnostic = None


def diagnostics_timer(event):
    diagnostic.set_level_and_message(DiagnosticStatus.OK)
    diagnostic.report_valid_data(event.current_real)
    diagnostic.publish(event.current_real)


if __name__ == '__main__':
    rospy.init_node('cola2_version')
    pub = rospy.Publisher('~version', String, latch=True, queue_size=1)
    pub.publish(
        String(get_version(rospy.get_param("~only_iquarobotics_repos", True))))
    diagnostic = DiagnosticHelper("cola2_version", rospy.get_name())
    diagnostic.set_enabled(True)
    rospy.Timer(rospy.Duration(1), diagnostics_timer)
    rospy.spin()
