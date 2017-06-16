## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('leach', ['internet'])
    module.includes = '.'
    module.source = [
        'model/leach-rtable.cc',
        'model/leach-packet-queue.cc',
        'model/leach-packet.cc',
        'model/leach-routing-protocol.cc',
        'helper/leach-helper.cc',
        ]

    module_test = bld.create_ns3_module_test_library('leach')
    module_test.source = [
        'test/leach-testcase.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'leach'
    headers.source = [
        'model/leach-rtable.h',
        'model/leach-packet-queue.h',
        'model/leach-packet.h',
        'model/leach-routing-protocol.h',
        'helper/leach-helper.h',
        ]
    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    bld.ns3_python_bindings()
