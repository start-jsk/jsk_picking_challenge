#!/usr/bin/env python

import os.path as osp
from summarize_logs_train_fcn import summarize_logs


if __name__ == '__main__':
    here = osp.dirname(osp.abspath(__file__))
    logs_dir = osp.join(here, 'logs', 'train_cyclegan')
    print('# logs_dir = %s' % logs_dir)

    keys = [
        'name',
        'epoch',
        'iteration',
    ]
    summarize_logs(logs_dir, keys, target_key=keys[-1], objective='max')
