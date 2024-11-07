# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: onboard_compute.proto

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
  name='onboard_compute.proto',
  package='steeleagle',
  syntax='proto3',
  serialized_pb=_b('\n\x15onboard_compute.proto\x12\nsteeleagle\"O\n\x0e\x43omputeRequest\x12\x12\n\nframe_data\x18\x01 \x01(\x0c\x12\x13\n\x0b\x66rame_width\x18\x02 \x01(\x05\x12\x14\n\x0c\x66rame_height\x18\x03 \x01(\x05\"@\n\rComputeResult\x12/\n\x0e\x63ompute_result\x18\x01 \x03(\x0b\x32\x17.steeleagle.AIDetection\"\xdc\x01\n\x0b\x41IDetection\x12\x14\n\x0ctimestamp_ns\x18\x01 \x01(\x03\x12\x10\n\x08\x63lass_id\x18\x02 \x01(\x05\x12\x10\n\x08\x66rame_id\x18\x03 \x01(\x05\x12\x12\n\nclass_name\x18\x04 \x01(\t\x12\x0b\n\x03\x63\x61m\x18\x05 \x01(\t\x12\x18\n\x10\x63lass_confidence\x18\x06 \x01(\x02\x12\x1c\n\x14\x64\x65tection_confidence\x18\x07 \x01(\x02\x12\r\n\x05x_min\x18\x08 \x01(\x02\x12\r\n\x05y_min\x18\t \x01(\x02\x12\r\n\x05x_max\x18\n \x01(\x02\x12\r\n\x05y_max\x18\x0b \x01(\x02\x62\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_COMPUTEREQUEST = _descriptor.Descriptor(
  name='ComputeRequest',
  full_name='steeleagle.ComputeRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame_data', full_name='steeleagle.ComputeRequest.frame_data', index=0,
      number=1, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_width', full_name='steeleagle.ComputeRequest.frame_width', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_height', full_name='steeleagle.ComputeRequest.frame_height', index=2,
      number=3, type=5, cpp_type=1, label=1,
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
  serialized_start=37,
  serialized_end=116,
)


_COMPUTERESULT = _descriptor.Descriptor(
  name='ComputeResult',
  full_name='steeleagle.ComputeResult',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='compute_result', full_name='steeleagle.ComputeResult.compute_result', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=118,
  serialized_end=182,
)


_AIDETECTION = _descriptor.Descriptor(
  name='AIDetection',
  full_name='steeleagle.AIDetection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp_ns', full_name='steeleagle.AIDetection.timestamp_ns', index=0,
      number=1, type=3, cpp_type=2, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='class_id', full_name='steeleagle.AIDetection.class_id', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='steeleagle.AIDetection.frame_id', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='class_name', full_name='steeleagle.AIDetection.class_name', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cam', full_name='steeleagle.AIDetection.cam', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='class_confidence', full_name='steeleagle.AIDetection.class_confidence', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='detection_confidence', full_name='steeleagle.AIDetection.detection_confidence', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='x_min', full_name='steeleagle.AIDetection.x_min', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y_min', full_name='steeleagle.AIDetection.y_min', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='x_max', full_name='steeleagle.AIDetection.x_max', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y_max', full_name='steeleagle.AIDetection.y_max', index=10,
      number=11, type=2, cpp_type=6, label=1,
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
  serialized_start=185,
  serialized_end=405,
)

_COMPUTERESULT.fields_by_name['compute_result'].message_type = _AIDETECTION
DESCRIPTOR.message_types_by_name['ComputeRequest'] = _COMPUTEREQUEST
DESCRIPTOR.message_types_by_name['ComputeResult'] = _COMPUTERESULT
DESCRIPTOR.message_types_by_name['AIDetection'] = _AIDETECTION

ComputeRequest = _reflection.GeneratedProtocolMessageType('ComputeRequest', (_message.Message,), dict(
  DESCRIPTOR = _COMPUTEREQUEST,
  __module__ = 'onboard_compute_pb2'
  # @@protoc_insertion_point(class_scope:steeleagle.ComputeRequest)
  ))
_sym_db.RegisterMessage(ComputeRequest)

ComputeResult = _reflection.GeneratedProtocolMessageType('ComputeResult', (_message.Message,), dict(
  DESCRIPTOR = _COMPUTERESULT,
  __module__ = 'onboard_compute_pb2'
  # @@protoc_insertion_point(class_scope:steeleagle.ComputeResult)
  ))
_sym_db.RegisterMessage(ComputeResult)

AIDetection = _reflection.GeneratedProtocolMessageType('AIDetection', (_message.Message,), dict(
  DESCRIPTOR = _AIDETECTION,
  __module__ = 'onboard_compute_pb2'
  # @@protoc_insertion_point(class_scope:steeleagle.AIDetection)
  ))
_sym_db.RegisterMessage(AIDetection)


# @@protoc_insertion_point(module_scope)