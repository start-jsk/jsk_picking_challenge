#!/usr/bin/env python

fg_class_names = [
    'avery_binder',
    'balloons',
    'band_aid_tape',
    'bath_sponge',
    'black_fashion_gloves',
    'burts_bees_baby_wipes',
    'colgate_toothbrush_4pk',
    'composition_book',
    'crayons',
    'duct_tape',
    'epsom_salts',
    'expo_eraser',
    'fiskars_scissors',
    'flashlight',
    'glue_sticks',
    'hand_weight',
    'hanes_socks',
    'hinged_ruled_index_cards',
    'ice_cube_tray',
    'irish_spring_soap',
    'laugh_out_loud_jokes',
    'marbles',
    'measuring_spoons',
    'mesh_cup',
    'mouse_traps',
    'pie_plates',
    'plastic_wine_glass',
    'poland_spring_water',
    'reynolds_wrap',
    'robots_dvd',
    'robots_everywhere',
    'scotch_sponges',
    'speed_stick',
    'table_cloth',
    'tennis_ball_container',
    'ticonderoga_pencils',
    'tissue_box',
    'toilet_brush',
    'white_facecloth',
    'windex',
]

for cls_id, cls_nm in enumerate(fg_class_names):
    print('%02d: %s' % (cls_id, cls_nm))
