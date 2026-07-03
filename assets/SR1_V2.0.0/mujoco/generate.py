#!/usr/bin/env python

import argparse
from pathlib import Path
from marionette_emgen import em_generate
from marionette_emgen.csv_config import Config
from marionette_emgen.mjcf import *


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o',
                        help='Output path')
    parser.add_argument('--rgba', default='#C0C0C0FF',
                        help='Main color in Hex format')

    args = parser.parse_args()

    cfg = Config.load('../csv')

    em_dict = dict(
        cfg=cfg,
        taghead_body=lambda **kwargs: em_taghead_body(cfg, **kwargs),
        tag_joint=lambda **kwargs: em_tag_joint(cfg, **kwargs),
        tag_inertial=lambda **kwargs: em_tag_inertial(cfg, **kwargs),
        _main_rgba=(
            int(args.rgba[1:3], 16)/255.,
            int(args.rgba[3:5], 16)/255.,
            int(args.rgba[5:7], 16)/255.,
            int(args.rgba[7:9], 16)/255.,
        ),
    )

    em_generate(
        Path('Kirin7j-Prototype.xml.em'),
        args.output,
        em_dict=em_dict,
    )
