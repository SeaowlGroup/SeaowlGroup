#!/usr/bin/env python3

import numpy as np
import rospkg
import yaml

NB_PROCESS = 6
OPUS_START = 2013
OPUS_END = 2016
SERIAL_TO_UPDATE = ''



if __name__ == "__main__":

    yaml_file = open("config/param/param4.yaml", 'r')
    yaml_content = yaml.safe_load(yaml_file)

    # Output parameters
    serial = 'recovInput'
    # Write Input
    rospack = rospkg.RosPack()
    input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
    f = open(input,'a')
    f.write(f'OPUS    CLASS    U_D_ASV    LOC_PLAN    HEADING    U_D    DCPA    SIZE    PRIOR    D_DETEC    GROUP\n')
    f.close()

    params = []
    opus = 1
    lp = True
    size = 8.
    type = None
    
    try:
        for h in yaml_content['heading']:
            for u_d in yaml_content['u_d']:
                for u_d_asv in yaml_content['u_d_asv']:
                    for dcpa in yaml_content['dcpa']:
                        for d_detec in yaml_content['d_detection_adrien']: ############################################
                            if (h<340 and h>20 or np.abs(u_d-u_d_asv)>2.57) and (d_detec > np.abs(dcpa)):
                                if opus <= OPUS_END:
                                    input = f"{rospack.get_path('asv_system')}/input/{serial}.txt"
                                    f = open(input,'a')
                                    if (np.abs(h)<=20):
                                        if u_d < u_d_asv:
                                            class_scen = 'OVERTAKING'
                                            group = 1
                                        else :
                                            class_scen = 'OVERTAKEN'
                                            group = 2
                                    elif (h>20 and h<150):
                                        class_scen = 'CROSSING_LEFT'
                                        group = 3
                                    elif (h>=150 and h<=210):
                                        class_scen = 'HEAD_ON'
                                        group = 4
                                    else:
                                        class_scen = 'CROSSING_RIGHT'
                                        group = 5
                                    f.write(f'{opus}    {class_scen}   {u_d_asv}    {lp}    {h}    {u_d}    {dcpa}    {size}    {type}    {d_detec}    {group}\n')
                                    f.close()                                
                                opus += 1
                                    
    except KeyboardInterrupt:
        pass
