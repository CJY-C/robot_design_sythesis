from copy import deepcopy
import gym
import torch
import torch.nn as nn
import torch.nn.functional as F

# 创建 gym 环境
env = gym.make('CartPole-v0')

# 重置环境
env.reset()


# 定义模型结构（这应该与您训练时使用的结构相同）
class MyModel(nn.Module):
    def __init__(self):
        super(MyModel, self).__init__()
        # 添加层和激活函数（根据您的实际模型结构进行修改）

    def forward(self, x):
        # 定义前向传播（根据您的实际模型结构进行修改）
        pass

# 实例化模型
model = MyModel()

# 加载模型权重
model.load_state_dict(torch.load('model.pth'))

# 将模型设置为 eval 模式
model.eval()

def mlp(
    sizes: list,
    activation=nn.ReLU,
    output_activation=nn.Identity
):
    '''
    构造多层感知机

    参数 
    ---
    sizes: list
      网络结构
    activation: 激活函数
      插入到每层之后
    output_activation: softmax or Identity 
      分类使用softmax，回归用Identity

    返回值 
    ---
    torch.nn.Sequential()

    使用方法 
    ---
    >>> mlp([obs_size] + list(hidden_sizes) + [action_size], torch.nn.ReLU)

    '''
    layers = list()
    for j in range(len(sizes)-1):
        act = activation if j < len(sizes) - 2 else output_activation
        layers += [nn.Linear(sizes[j], sizes[j+1]), act()]
    return nn.Sequential(*layers)


class QNet(nn.Module):
    '''
    ## 网络类
    - 实现了网络的前向传播和选择动作的函数
    '''

    # 定义Net的一系列属性
    def __init__(self, observation_space, action_space, hidden_sizes, actiavtion=nn.ReLU):
        super(QNet, self).__init__()
        self._obs = {i: o for i, o in observation_space.sample().items()}
        self._fc_A = nn.Linear(self._obs['A'].size, 64)
        self._fc_T = nn.Linear(self._obs['T'].size, 64)

        self._conv_O = nn.Conv3d(in_channels=1, out_channels=16, kernel_size=3, stride=1, padding=1)
        self._maxPool = nn.MaxPool3d(kernel_size=3, stride=3, padding=0)
  
        self._ob_n = 64 * 3
        
        self._ac_n = action_space.n if isinstance(
            action_space, gym.spaces.Discrete) else action_space.shape[0]
        self.model = mlp([self._ob_n] + list(hidden_sizes) +
                         [self._ac_n], activation=actiavtion)

    # 定义forward函数 (x为状态)
    def forward(self, A, T, O):
        A = F.relu(self._fc_A(torch.Tensor(A)))
        T = F.relu(self._fc_T(torch.Tensor(T)))
        O = F.relu(self._conv_O(torch.Tensor(O).unsqueeze(1)))
        O = self._maxPool(O)
        O = O.view(O.size(0), -1)
        # print(A.shape, T.shape, O.shape)
        x = torch.cat([A, T, O], dim=1)
        return self.model(torch.Tensor(x))

    def act(self, A, T, O):
        with torch.no_grad():
            return torch.argmax(self.forward(A, T, O)).item()


class DQN_2015:
    '''
    ## DQN-2015
    - 实现了DQN的主要功能(主网络更新，目标网络更新，选择动作)
    '''

    def __init__(self, observation_space, action_space, hidden_sizes, lr, actiavtion=nn.Tanh) -> None:
        self.main = QNet(observation_space, action_space,
                        hidden_sizes, actiavtion)
        self.target = deepcopy(self.main)
        # self.optim = torch.optim.SGD(self.main.parameters(), lr=lr)
        # self.optim = RMSprop(self.main.parameters(), lr=lr)
        self.optim = torch.optim.Adam(self.main.parameters(), lr=lr)
        self.crite = torch.nn.MSELoss()

    def target_update(self):
        self.target.load_state_dict(self.main.state_dict())

    def act(self, A, T, O):
        return self.main.act(A, T, O)

network = DQN_2015(env.observation_space, env.action_space, ac_kwargs['hidden_sizes'], q_lr, ac_kwargs['activation'])