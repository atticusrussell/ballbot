# !/usr/bin/env python
# coding: utf-8
import logging

def setup_logger():
    logging.basicConfig(filename='log.txt', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')



