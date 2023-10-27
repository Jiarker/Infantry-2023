# ovenv_activate
source ~/venv/openvino_env/bin/activate

# convert
# params:
# (str) sorce onnx file path
# (str) output dir path
# (str) output file name
# (bool) enable onnx

python3 ../ov_model_convert.py \
    "yolov5s-face.onnx" \
    "./" \
    "output" \
    True

