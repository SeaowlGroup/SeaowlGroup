#!/usr/bin/env python3

import rospkg

rospack = rospkg.RosPack()

input = f"{rospack.get_path('asv_common')}/input/{serial1}.txt"
output = f"{rospack.get_path('asv_common')}/output/{serial1}.txt"
