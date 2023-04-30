import queue

from urdfGenerator import register
from urdfGenerator import UnitConfig
from urdfGenerator import Enums

register.PathRegister.add_path('小老板')

UnitConfig.generateUnit(Enums.UnitType.BASEL)


q = queue.Queue()

q.put('A')
q.put('B')
q.put('C')

# 使用 queue.Queue.queue 属性查看队列中的元素
print(q.queue)  # 输出：deque(['A', 'B', 'C'])

# 使用 queue.Queue.queue.copy() 方法拷贝队列底层元素列表
queue_copy = q.queue.copy()

# 修改队列中的元素
q.queue[0] = 'D'
print(q.queue)  # 输出：deque(['D', 'B', 'C'])

# 修改拷贝出来的列表中的元素
queue_copy[1] = 'E'
print(queue_copy)  # 输出：['A', 'E', 'C']

# 遍历拷贝出来的列表
for item in queue_copy:
    print(item)  # 输出：A, E, C

print(q.queue)