#%%
import torch

# 加载 JIT 模型
model = torch.jit.load("/home/ldc/hit_robot/DMEC_deploy/src/robot_pai_controller/policy/policy_pai01.04.pt")
model.eval()
print(model)

# image_input = torch.randn(1, 58, 87)
# obs_input = torch.randn(1, 47)
# with torch.no_grad():
#     output = model(image_input, obs_input)
# print(output)

# torch.onnx.export(model,
#                   (image_input, obs_input),
#                   "depthPolicy1.onnx",
#                   export_params=True,
#                   opset_version=11,
#                   do_constant_folding=True,
#                   input_names=["image_input", "obs_input"],
#                   output_names=["output"],
# )


#%%
# 尝试不同的输入维度
# for i in range(3101,3103):
for i in range(704,706):
    input_tensor = torch.randn(1, i)
    try:
        with torch.no_grad():
            output = model(input_tensor)
        print(f"成功：输入维度 {i}")
    except Exception as e:
        print(f"失败：输入维度 {i}, 错误：{e}")

#%%
# 创建一个示例输入（根据你的模型需求调整大小和类型）
# example_input = torch.randn(1, 3102)
example_input = torch.randn(1, 705)

# 导出模型
torch.onnx.export(model,               # JIT 模型
                  example_input,       # 示例输入
                  "policy_2.onnx",        # 输出文件名
                  export_params=True,  # 导出模型的参数
                  opset_version=11,    # ONNX opset 版本
                  do_constant_folding=True,  # 优化常量折叠
                  input_names=['input'],   # 输入名
                  output_names=['output'], # 输出名
                  )
