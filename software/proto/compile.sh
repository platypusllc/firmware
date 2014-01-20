#!/bin/sh
#
# Compiles the Platypus protocol buffer messages and places them into
# the Android source tree.
#
java -jar wire-compiler-1.2.0.jar \
    --proto_path=. \
    --java_out=out google/protobuf/descriptor.proto