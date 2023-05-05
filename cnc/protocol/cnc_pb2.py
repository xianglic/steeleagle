# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cnc.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='cnc.proto',
  package='cnc',
  syntax='proto3',
  serialized_pb=_b('\n\tcnc.proto\x12\x03\x63nc\"O\n\x08Location\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x10\n\x08latitude\x18\x02 \x01(\x01\x12\x11\n\tlongitude\x18\x03 \x01(\x01\x12\x10\n\x08\x61ltitude\x18\x04 \x01(\x01\"A\n\x07\x43ommand\x12\x14\n\x0c\x66or_drone_id\x18\x01 \x01(\t\x12\x12\n\nscript_url\x18\x02 \x01(\t\x12\x0c\n\x04halt\x18\x03 \x01(\x08\"E\n\x0b\x44roneStatus\x12\x0f\n\x07\x62\x61ttery\x18\x01 \x01(\x03\x12\x14\n\x0cgimbal_pitch\x18\x02 \x01(\x01\x12\x0f\n\x07\x62\x65\x61ring\x18\x03 \x01(\x01\"=\n\x04PCMD\x12\x0b\n\x03gaz\x18\x01 \x01(\x05\x12\x0b\n\x03yaw\x18\x02 \x01(\x05\x12\r\n\x05pitch\x18\x03 \x01(\x05\x12\x0c\n\x04roll\x18\x04 \x01(\x05\"\xf4\x01\n\x06\x45xtras\x12\x13\n\x0bregistering\x18\x01 \x01(\x08\x12\x10\n\x08\x64rone_id\x18\x02 \x01(\t\x12\x14\n\x0c\x63ommander_id\x18\x03 \x01(\t\x12\x19\n\x03\x63md\x18\x04 \x01(\x0b\x32\x0c.cnc.Command\x12\x1f\n\x08location\x18\x05 \x01(\x0b\x32\r.cnc.Location\x12 \n\x06status\x18\x06 \x01(\x0b\x32\x10.cnc.DroneStatus\x12\x17\n\x0f\x64\x65tection_model\x18\x07 \x01(\t\x12\x17\n\x04pcmd\x18\x08 \x01(\x0b\x32\t.cnc.PCMD\x12\x0f\n\x07takeoff\x18\t \x01(\x08\x12\x0c\n\x04land\x18\n \x01(\x08\x42\x1f\n\x15\x65\x64u.cmu.cs.steeleagleB\x06Protosb\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_LOCATION = _descriptor.Descriptor(
  name='Location',
  full_name='cnc.Location',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='cnc.Location.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='cnc.Location.latitude', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='cnc.Location.longitude', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='cnc.Location.altitude', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=18,
  serialized_end=97,
)


_COMMAND = _descriptor.Descriptor(
  name='Command',
  full_name='cnc.Command',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='for_drone_id', full_name='cnc.Command.for_drone_id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='script_url', full_name='cnc.Command.script_url', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='halt', full_name='cnc.Command.halt', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=99,
  serialized_end=164,
)


_DRONESTATUS = _descriptor.Descriptor(
  name='DroneStatus',
  full_name='cnc.DroneStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='battery', full_name='cnc.DroneStatus.battery', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gimbal_pitch', full_name='cnc.DroneStatus.gimbal_pitch', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bearing', full_name='cnc.DroneStatus.bearing', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=166,
  serialized_end=235,
)


_PCMD = _descriptor.Descriptor(
  name='PCMD',
  full_name='cnc.PCMD',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='gaz', full_name='cnc.PCMD.gaz', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='yaw', full_name='cnc.PCMD.yaw', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='cnc.PCMD.pitch', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='roll', full_name='cnc.PCMD.roll', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=237,
  serialized_end=298,
)


_EXTRAS = _descriptor.Descriptor(
  name='Extras',
  full_name='cnc.Extras',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='registering', full_name='cnc.Extras.registering', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='drone_id', full_name='cnc.Extras.drone_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='commander_id', full_name='cnc.Extras.commander_id', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cmd', full_name='cnc.Extras.cmd', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='location', full_name='cnc.Extras.location', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='cnc.Extras.status', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='detection_model', full_name='cnc.Extras.detection_model', index=6,
      number=7, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pcmd', full_name='cnc.Extras.pcmd', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='takeoff', full_name='cnc.Extras.takeoff', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='land', full_name='cnc.Extras.land', index=9,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=301,
  serialized_end=545,
)

_EXTRAS.fields_by_name['cmd'].message_type = _COMMAND
_EXTRAS.fields_by_name['location'].message_type = _LOCATION
_EXTRAS.fields_by_name['status'].message_type = _DRONESTATUS
_EXTRAS.fields_by_name['pcmd'].message_type = _PCMD
DESCRIPTOR.message_types_by_name['Location'] = _LOCATION
DESCRIPTOR.message_types_by_name['Command'] = _COMMAND
DESCRIPTOR.message_types_by_name['DroneStatus'] = _DRONESTATUS
DESCRIPTOR.message_types_by_name['PCMD'] = _PCMD
DESCRIPTOR.message_types_by_name['Extras'] = _EXTRAS

Location = _reflection.GeneratedProtocolMessageType('Location', (_message.Message,), dict(
  DESCRIPTOR = _LOCATION,
  __module__ = 'cnc_pb2'
  # @@protoc_insertion_point(class_scope:cnc.Location)
  ))
_sym_db.RegisterMessage(Location)

Command = _reflection.GeneratedProtocolMessageType('Command', (_message.Message,), dict(
  DESCRIPTOR = _COMMAND,
  __module__ = 'cnc_pb2'
  # @@protoc_insertion_point(class_scope:cnc.Command)
  ))
_sym_db.RegisterMessage(Command)

DroneStatus = _reflection.GeneratedProtocolMessageType('DroneStatus', (_message.Message,), dict(
  DESCRIPTOR = _DRONESTATUS,
  __module__ = 'cnc_pb2'
  # @@protoc_insertion_point(class_scope:cnc.DroneStatus)
  ))
_sym_db.RegisterMessage(DroneStatus)

PCMD = _reflection.GeneratedProtocolMessageType('PCMD', (_message.Message,), dict(
  DESCRIPTOR = _PCMD,
  __module__ = 'cnc_pb2'
  # @@protoc_insertion_point(class_scope:cnc.PCMD)
  ))
_sym_db.RegisterMessage(PCMD)

Extras = _reflection.GeneratedProtocolMessageType('Extras', (_message.Message,), dict(
  DESCRIPTOR = _EXTRAS,
  __module__ = 'cnc_pb2'
  # @@protoc_insertion_point(class_scope:cnc.Extras)
  ))
_sym_db.RegisterMessage(Extras)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\025edu.cmu.cs.steeleagleB\006Protos'))
# @@protoc_insertion_point(module_scope)
