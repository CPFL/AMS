#!/usr/bin/env python
# coding: utf-8

import signal
from subprocess import Popen, check_call
from time import sleep
from argparse import ArgumentParser
from os import listdir
from os.path import realpath
import random

from setproctitle import setproctitle

from ams import logger


def killall_old_user_node():
    command = "killall -9 ams_user"
    try:
        check_call(command.split(" "))
    except:
        pass


if __name__ == '__main__':
    setproctitle("ams_user_generator")

    parser = ArgumentParser()
    parser.add_argument("-UIDP", "--user_initials_directory_path", type=str, required=True, help="user_initials directory path")
    parser.add_argument("-URF", "--use_ros_flag", action="store_true", help="use ros flag")
    parser.add_argument("-IF", "--identifiable_flag", action="store_true", help="use identifiable flag")
    args = parser.parse_args()

    user_initials_directory_real_path = realpath(args.user_initials_directory_path)
    initial_file_names = listdir(user_initials_directory_real_path)
    user_processes = dict(zip(initial_file_names, [None]*len(initial_file_names)))

    try:
        while True:
            for user_initial_file_name in user_processes:
                if user_processes[user_initial_file_name] is not None:
                    if user_processes[user_initial_file_name].poll() is not None:
                        user_processes[user_initial_file_name] = None

            target_user_initial_file_name = None
            unused_user_initial_file_names = list(map(lambda x: x[0], filter(lambda y: y[1] is None, user_processes.items())))
            if 0 < len(unused_user_initial_file_names):
                if 0 == len(user_processes):
                    target_user_initial_file_name = random.choice(unused_user_initial_file_names)
                else:
                    if 0.9 < random.random():
                        target_user_initial_file_name = random.choice(unused_user_initial_file_names)

            if target_user_initial_file_name is not None:
                print("launch {}".format(target_user_initial_file_name.split(".")[0]))

                command = "python ../node_launcher/user.py -KCT redis -PSCT paho -TD sim -IFP {} -SMP ./res/state_machines/user.json -ELR 0.2".format(user_initials_directory_real_path+"/"+target_user_initial_file_name)
                if args.identifiable_flag:
                    command += " -IF"
                if args.use_ros_flag:
                    command += " -URF"
                user_processes[target_user_initial_file_name] = Popen(command.split(" "))

            sleep(10)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(logger.pformat(e))
    finally:
        for process in user_processes.values():
            if process is not None:
                process.send_signal(signal.SIGINT)
