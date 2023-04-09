import pandas as pd
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['SimHei']
plt.rcParams['axes.unicode_minus'] = False


# read in the data file
df = pd.read_table('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/algo/data/RobotConfigDesign-v10/progress.txt')

# extract the two desired columns
col1 = df['TotalEnvInteracts']
# col2 = df['LossQ']
col2 = df['AverageTestEpRet']
# col2 = df['TestEpLen']

# plot the columns
plt.plot(col1, col2)
plt.xlabel('总交互次数')
# plt.ylabel('LossQ')
# plt.title('Loss curve')
plt.ylabel('平均奖励值')
plt.title('随机策略')
# plt.ylabel('steps')
# plt.title('Step')
plt.show()

