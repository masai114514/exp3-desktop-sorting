# -*- coding: utf-8 -*-
"""读取 exp3/config/*.json。也可被测试/脚本直接改字段后使用。"""
import json
import os

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def _load(name):
    with open(os.path.join(_CONFIG_DIR, name), encoding='utf-8') as f:
        return json.load(f)


def load_grid():
    return _load('grid_cells.json')


def load_bins():
    return _load('bins.json')


def load_task():
    return _load('task.json')


def load_all():
    return {'grid': load_grid(), 'bins': load_bins(), 'task': load_task()}
