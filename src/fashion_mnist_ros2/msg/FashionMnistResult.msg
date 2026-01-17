# FashionMnistResult.msg：结构化存储Fashion MNIST识别结果
# 消息编号（用于区分不同批次结果）
uint32 msg_id
# 识别分类标签（如T-shirt、Ankle boot）
string label
# 识别置信度（正确率，0~1之间，保留2位小数）
float32 confidence
