飞机仿真器
具体用法参见test.py
先用paser解析scenarios里的yaml配置文件，产生一个dict传给AirEnv构造仿真器。
主要使用以下两个接口
obs = env.reset()
obs,reward,done,info = env.step(action)

scenarios里的XXX.yaml是配置文件，定义任务的属性，智能体的属性和障碍物的属性。
AirEnv是仿真器主体，里面的reward函数是空的，需要自己定义！！！！！observation也可以自己根据需要修改
basic定义了agent和fence的性质
paser用于解析配置文件，可以不改
rendering用于渲染，可以不改