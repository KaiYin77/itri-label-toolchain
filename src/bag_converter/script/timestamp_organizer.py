import os
import logging
from tqdm import tqdm
from pathlib import Path
import shutil

import numpy as np
import pandas as pd

import argparse
import configparser

'''
Logger
'''
logging.basicConfig(level=logging.INFO)


'''
ConfigParser
'''
config = configparser.ConfigParser()
config.read('config.ini')

'''
Argparse
'''
parser = argparse.ArgumentParser()
parser.add_argument(
        "--debug",
        help="debug mode",
        action='store_true',
        )
args = parser.parse_args()

class TimestampOrganizer():
    def __init__(self):
        self.timestamps_dir = Path(config['data']['timestamps'])
        self.raw_pcd_dir = Path(config['data']['raw_lidar'])
        self.pcd_dir = Path(config['data']['lidar'])
        self.pcd_list = sorted(self.raw_pcd_dir.rglob("*.pcd"))
        self.pcd_timestamps = [p.stem for p in self.pcd_list]
        
        self.raw_image_dir = Path(config['data']['raw_image'])
        self.image_dir = Path(config['data']['image'])
        self.image_list = sorted(self.raw_image_dir.rglob("*.png"))
        self.image_timestamps = [i.stem for i in self.image_list]
        

    def run(self):
        self.path_check()
        self.save_timestamp_to_txt(self.pcd_timestamps, self.timestamps_dir, "lidar_timestamps")
        self.pcd_copy_rename()
        self.pcd_image_corresponding_timestamp()
        self.save_timestamp_to_txt(self.nearest_image_timestamps, self.timestamps_dir, "camera_gige_30_f_hdr_timestamps")
        self.image_copy_rename()
    
    def path_check(self):
        logging.info('check if timestamps_path exists ? ' + str(self.timestamps_dir.exists()))
        self.timestamps_dir.mkdir(parents=True, exist_ok=True)
        
        logging.info('check if raw_pcd_path exists ? ' + str(self.raw_pcd_dir.exists()))
        self.raw_pcd_dir.mkdir(parents=True, exist_ok=True)
        
        logging.info('check if target pcd_path exists ? ' + str(self.pcd_dir.exists()))
        self.pcd_dir.mkdir(parents=True, exist_ok=True)
        
        logging.info('check if raw_image_path exists ? ' + str(self.raw_image_dir.exists()))
        self.raw_image_dir.mkdir(parents=True, exist_ok=True)
        
        logging.info('check if image_path exists ? ' + str(self.image_dir.exists()))
        self.image_dir.mkdir(parents=True, exist_ok=True)

    def save_timestamp_to_txt(
            self, 
            timestamps_list,
            timestamps_dir,
            timestamps_name="lidar_timestamps"
        ):
        logging.info('record timestamp of ' + timestamps_name)

        path = timestamps_dir / Path(timestamps_name + '.txt')
        f = open(path, 'w')
        for t in timestamps_list:
            f.write(t)
            f.write('\n')
        f.close()

    def pcd_copy_rename(self):
        logging.info('copy .pcd file of lidar and rename')

        for idx, pcd_path in enumerate(self.pcd_list):
            from_file = pcd_path
            to_file = self.pcd_dir / Path(str(idx).zfill(8) + '.pcd')

            if not to_file.is_file():
                shutil.copy(str(from_file), str(to_file))
        
    def pcd_image_corresponding_timestamp(self):
        logging.info('find corresponding timestamp for lidar and camera')
        self.nearest_image_timestamps = []
        for pcd in self.pcd_list:
            pcd_timestamp = pcd.stem
            nearest_image_timestamp = self.find_nearest_timestamp(pcd_timestamp, self.image_timestamps)
            self.nearest_image_timestamps.append(nearest_image_timestamp)
    
    def find_nearest_timestamp(self, given_value, target_list):
        abs_diff_fn = lambda list_value : abs(float(list_value) - float(given_value))
        return min(target_list, key=abs_diff_fn)
    
    def image_copy_rename(self):
        logging.info('copy .png file of camera and rename')

        for idx, image_path in enumerate(self.nearest_image_timestamps):
            from_file = self.raw_image_dir / Path(image_path + '.png')
            to_file = self.image_dir / Path(str(idx).zfill(8) + '.png')

            if not to_file.is_file():
                shutil.copy(str(from_file), str(to_file))
        

if __name__ == '__main__':
    TimestampOrganizer().run()
