from lxml import etree
from arxml import *

root = get_can()

add_ecu(root, 'Motor1')
add_ecu(root, 'Main')

base_id = 0x100


add_pydb_frame(root, 'Motor1', 'Main', base_id + 0, 'PWM',      'EV', 'bBHH', ('Direction', 'Duty', 'none1', 'Timeout',))
add_pydb_frame(root, 'Motor1', 'Main', base_id + 1, 'Position', 'EV', 'iH', ('target_pos', 'Timeout',))
add_pydb_frame(root, 'Motor1', 'Main', base_id + 2, 'Speed',    'EV', 'iH', ('target_speed', 'Timeout',))

#add_pydb_frame(root, 'Motor1', 'Main', 0x10A, 'PID1',     'EV', 'ff',    ('PID_kP', 'PID_kI',))
#add_pydb_frame(root, 'Motor1', 'Main', 0x10B, 'PID2',     'EV', 'ff',    ('PID_kD', 'PID_max',))

add_pydb_frame(root, 'Motor1', 'Main', base_id + 8, 'Status1',  'RX', 'bBBBi', ('Direction', 'Duty', 'Mode', 'None2', 'Position',))
add_pydb_frame(root, 'Motor1', 'Main', base_id + 9, 'Status2',  'RX', 'fi',    ('VBAT', 'Target',))


etree.ElementTree(root).write("can_motor.arxml", encoding='utf-8', xml_declaration=True, pretty_print=True)
