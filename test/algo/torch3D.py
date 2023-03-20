# import torch
# import torch.nn as nn

# # 创建一个Conv3d层
# conv3d = nn.Conv3d(in_channels=3, out_channels=16, kernel_size=(3, 3, 3), stride=1, padding=1)

# # 创建一个输入张量，批次大小为1，通道数为3，矩阵大小为10x10x10
# input_tensor = torch.randn(1, 3, 10, 10, 10)

# # 对输入张量进行卷积操作
# output_tensor = conv3d(input_tensor)
# print(output_tensor.shape)
# print(output_tensor.size(-3))
# print(output_tensor.size(-2))
# print(output_tensor.size(-1))

# max_pooling = nn.MaxPool3d(3, 2)

# pooled_feature = max_pooling(output_tensor)
# print(pooled_feature.shape)

# compressed_feature = pooled_feature.view(-1)

# print(compressed_feature.shape)

