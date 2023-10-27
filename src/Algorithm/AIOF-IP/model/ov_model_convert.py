#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author(s): xili
# @Created: 2020/6/30
import sys
from openvino.tools.mo import convert_model
from openvino.preprocess import PrePostProcessor, ColorFormat, ResizeAlgorithm
from openvino.runtime import Core, Layout, Type, set_batch
from openvino.runtime import serialize
from openvino.runtime.passes import Manager, Serialize

if __name__ == '__main__':
    onnx_path = sys.argv[1]
    output_path = sys.argv[2]
    output_name = sys.argv[3]
    fp16_enable = sys.argv[4]

    output_xml = output_path + '/' + output_name + '.xml'
    output_bin = output_path + '/' + output_name + '.bin'

    print('INFO: onnx path : ', onnx_path)
    print('INFO: output xml : ', output_xml)
    print('INFO: output bin : ', output_bin)
    print('INFO: fp16 enable : ', fp16_enable)
    print(' ')

    if (fp16_enable == 'True' or fp16_enable == 'true'):
        model = convert_model(onnx_path, compress_to_fp16=True)
    else:
        model = convert_model(onnx_path, compress_to_fp16=False)

    # stop
    serialize(model, output_xml, output_bin)
    exit(0)

    # ======== Step 1: Preprocessing ================
    ppp = PrePostProcessor(model)
    # Declare section of desired application's input format
    ppp.input('input').tensor() \
        .set_element_type(Type.u8) \
        .set_shape([1, 480, 640, 3]) \
        .set_layout(Layout('NHWC')) \
        .set_color_format(ColorFormat.BGR) 
    
    # Specify actual model layout
    ppp.input().model().set_layout(Layout('NCHW'))

    # Explicit preprocessing steps. Layout conversion will be done automatically as last step
    ppp.input().preprocess() \
        .convert_element_type() \
        .convert_color(ColorFormat.RGB) \
        .resize(ResizeAlgorithm.RESIZE_LINEAR) \
        .mean([0, 0, 0]) \
        .scale([255, 255, 255])

    # Dump preprocessor
    print(f'Dump preprocessor: {ppp}')
    model = ppp.build()

    # ======== Step 2: Change batch size ================
    # In this example we also want to change batch size to increase throughput
    set_batch(model, 1)

    # ======== Step 3: Save the model ================
    # First method - using serialize runtime wrapper
    serialize(model, output_xml, output_bin)
    
    # Second method - using Manager and Serialize pass
    # manager = Manager()
    # manager.register_pass(Serialize('/path/to/some_model_saved.xml', '/path/to/some_model_saved.bin'))
    # manager.run_passes(model)

    print('INFO: Finish convert ')
    print('INFO: output xml : ', output_xml)
    print('INFO: output bin : ', output_bin)
    print('INFO: fp16 enable : ', fp16_enable)