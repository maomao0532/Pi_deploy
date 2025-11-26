import onnx
import onnxruntime
import torch

onnx_model = onnx.load("/home/ldc/hit_robot/HITRobot_RL/src/image_processing/policy/depthPolicy1.onnx")
onnx.checker.check_model(onnx_model)
print("ONNX 模型验证成功！")
# 打印输入名称
for input_tensor in onnx_model.graph.input:
    print("Input Name:", input_tensor.name)


# # 输入张量的示例
# image_input = torch.rand(1, 58, 87)  # 示例输入
# obs_input = torch.rand(1, 47)

# # 使用 ONNX Runtime 推理
# ort_session = onnxruntime.InferenceSession("/home/ldc/hit_robot/HITRobot_RL/depthPolicy1.onnx")
# inputs = {
#     "image_input": image_input.numpy(),
#     "obs_input": obs_input.numpy()
# }
# outputs = ort_session.run(None, inputs)
# print("ONNX 推理输出:", outputs)