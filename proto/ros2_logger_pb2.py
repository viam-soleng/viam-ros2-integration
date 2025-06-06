# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: proto/ros2_logger.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.api import annotations_pb2 as google_dot_api_dot_annotations__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x17proto/ros2_logger.proto\x12\x1fviamlabs.service.ros2_logger.v1\x1a\x1cgoogle/api/annotations.proto\"\x1d\n\x07Request\x12\x12\n\x04name\x18\x01 \x01(\tR\x04name\"D\n\x08Response\x12\x1b\n\tros_topic\x18\x01 \x01(\tR\x08rosTopic\x12\x1b\n\tlog_level\x18\x02 \x01(\tR\x08logLevel2\xb1\x01\n\x11ROS2LoggerService\x12\x9b\x01\n\x06Status\x12(.viamlabs.service.ros2_logger.v1.Request\x1a).viamlabs.service.ros2_logger.v1.Response\"<\x82\xd3\xe4\x93\x02\x36\"4/viamsoleng/api/v1/service/ros2_logger/{name}/statusb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'proto.ros2_logger_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_ROS2LOGGERSERVICE'].methods_by_name['Status']._loaded_options = None
  _globals['_ROS2LOGGERSERVICE'].methods_by_name['Status']._serialized_options = b'\202\323\344\223\0026\"4/viamsoleng/api/v1/service/ros2_logger/{name}/status'
  _globals['_REQUEST']._serialized_start=90
  _globals['_REQUEST']._serialized_end=119
  _globals['_RESPONSE']._serialized_start=121
  _globals['_RESPONSE']._serialized_end=189
  _globals['_ROS2LOGGERSERVICE']._serialized_start=192
  _globals['_ROS2LOGGERSERVICE']._serialized_end=369
# @@protoc_insertion_point(module_scope)
