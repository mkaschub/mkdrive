#!/usr/bin/env python3
from lxml import etree
import arxml

gen = arxml.Generator("CAN_DC_Motor")

gen.add_ecu('Main')

for i in range(1, 3):
    ecu = f'Motor{i}'
    base_id = 0x100 + (i - 1) * 32
    gen.add_ecu(ecu)

    gen.add_pydb_frame(ecu, 'Main', base_id + 0, 'PWM',      'EV', 'bBHH', ('Direction', 'Duty', 'none1', 'Timeout',))
    gen.add_pydb_frame(ecu, 'Main', base_id + 1, 'Position', 'EV', 'iH', ('target_pos', 'Timeout',))
    gen.add_pydb_frame(ecu, 'Main', base_id + 2, 'Speed',    'EV', 'iH', ('target_speed', 'Timeout',))

    #add_pydb_frame(root, ecu, 'Main', 0x10A, 'PID1',     'EV', 'ff',    ('PID_kP', 'PID_kI',))
    # add_pydb_frame(root, ecu, 'Main', 0x10B, 'PID2',     'EV', 'ff',    ('PID_kD', 'PID_max',))

    gen.add_pydb_frame(ecu, 'Main', base_id + 8, 'Status1',  'RX', 'bBBBi', ('Direction', 'Duty', 'Mode', 'None2', 'Position',))
    gen.add_pydb_frame(ecu, 'Main', base_id + 9, 'Status2',  'RX', 'fi',    ('VBAT', 'Target',))


etree.ElementTree(gen.root).write("can_motor.arxml", encoding='utf-8', xml_declaration=True, pretty_print=True)
