import asyncio
import logging
import time
from asyncio import CancelledError
from cup1_example.tryMain_plot import main
import numpy as np
import os
from cup1_example.addTargets import addTargets
from cup1_example.transCoordinate import lla2ecef, ecef2lla
##
formatter = logging.Formatter('%(asctime)s - %(name)s - %(filename)s:%(lineno)s - %(levelname)s - %(message)s')
logger = logging.getLogger('algo')
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setFormatter(formatter)
handler.setLevel(logging.DEBUG)
logger.addHandler(handler)




class Algo:
    session = dict()  # 各智能体连接信息
    ins = dict()  # 各智能体惯导信息
    sensor = dict()  # 各智能体发现的目标信息
    init_info = None  # 题目信息
    start_cmd = False  # 开始指令
    stop_cmd = False  # 停止指令

    @staticmethod
    async def algo():
        start_id = 0
        _first_start = True
        # first_time = True
        targets_list = []
        agents_pos = []




        ## Start the main process
        import multiprocessing
        import pickle
        init_info_queue = multiprocessing.Queue()
        pos_point_queue = multiprocessing.Queue()
        way_point_queue = multiprocessing.Queue()
        target_queue = multiprocessing.Queue()
        init_lock = multiprocessing.Lock()
        pos_lock = multiprocessing.Lock()
        way_lock = multiprocessing.Lock()
        target_lock= multiprocessing.Lock()
        import pyproj
        proj = pyproj.Transformer.from_crs(4326, 3857, always_xy=True)
        file_path = 'init_info_file'
        # time.sleep(5)
        main_process = multiprocessing.Process(target=main, args=(init_info_queue,pos_point_queue, way_point_queue,init_lock,pos_lock,way_lock,target_queue,target_lock,proj))
        main_process.start()
        def algo_writer(queue, data,lock):
            with lock:
                queue.put(pickle.dumps(data))
            return

        def algo_reader(queue,lock):
            with lock:
                latest_item = None
                if queue.empty():
                    return latest_item
                # while not queue.empty():
                latest_item = queue.get()
            return pickle.loads(latest_item)
                


        while True:
            try:
                await asyncio.sleep(0.1)

                # 未开始，等待开始指令
                if not Algo.start_cmd and not os.path.exists(file_path):
                    # logger.info('Game not started.')
                    continue

                if os.path.exists(file_path):
                    if start_id < 100:
                        await asyncio.sleep(0.1)
                        start_id += 1
                        continue

                # 收到停止指令
                if Algo.stop_cmd:
                    await Algo.send_stop()
                    logger.info('Game stopped.')
                    continue

                # 第一次收到开始指令，根据题目信息规划航路，并下发航行指令
                # 当前策略为从当前位置驶向区域的8个点，这不是一个好的策略，仅用于示例
                if _first_start:
                    _first_start = False
                    # if local exist init_info_file.json,wirte to it
                    # else just read info from it
                    import threading
                    def start_timer():
                        asyncio.run(timer_thread())
                    async def timer_thread():
                    # 等待85分钟（85 * 60秒）
                        await asyncio.sleep(85 * 60)
                        # 执行指令a（在此处添加您要执行的操作）
                        await Algo.send_finished()
                        logger.info('Game Finished.')
                    timer = threading.Thread(target=start_timer)
                    timer.start()
                    # file_path = 'init_info_file'
                    init_task = None
                    try:
                        # already
                        with open(file_path, 'rb') as file:
                            # If the file exists, read data from it
                            init_task=pickle.load(file)
                            print("Read data from existing file:")
                            print(init_task)
                        # Algo.start_cmd = True# 开始指令
                    except FileNotFoundError:
                        # first time
                        init_task = getInitTaskInfo(proj)
                        with open(file_path, 'wb') as file:
                            pickle.dump(init_task, file)
                            print("Created and wrote data to the new file:")
                            print(init_task)
                    algo_writer(init_info_queue,init_task,init_lock)

                # Finish all task

                finished  = algo_reader(init_info_queue,init_lock)
                if finished is not None:
                    if finished['Finished']:
                        await Algo.send_finished()
                        logger.info('Game Finished.')
                        continue
                # 发送各智能体位置
                # 1. in range n
                # 2. return obj like [{id:agentId, position:[213,342]n,...]}
                for _i in range(len(init_task["agents"])):
                    _agent_new = Algo.ins[f'node{_i+1}']
                    _agent_pos_new = [_agent_new['lon'], _agent_new['lat']]
                    agents_pos.append({"id":_i+1,"position":_agent_pos_new})
                algo_writer(pos_point_queue,agents_pos,pos_lock)
                agents_pos.clear()


                # 打印各智能体发现的目标
                for _i in range(len(init_task["agents"])):
                    _targets = Algo.sensor[f'node{_i+1}']
                    _ins = Algo.ins[f'node{_i+1}']
                    if _targets != []:
                        #logger.info(f'node{_i+1} targets={_targets}')
                        if _targets[0] not in targets_list:
                            targets_list.append(_targets[0])
                            transPosition = lla2ecef(np.array([[_targets[0]["lon"],_targets[0]["lat"]]]),proj)
                            newTarget = addTargets(
                                id=_targets[0]["id"],
                                position=transPosition,
                                angle=_targets[0]["angle"],
                                restcheck=_targets[0]["restCheck"],
                                restime=_targets[0]["restTime"],
                                certainradiuslimit=_targets[0]["threatRadius"],
                                certainradius=_targets[0]["certainRadius"],
                                agentid=_i+1,
                            )
                            algo_writer(target_queue,newTarget,target_lock)
                            _abstime = _ins['absTime']
                            # datetime_obj = datetime.datetime.fromtimestamp(_abstime)
                            # formatted_date = datetime_obj.strftime('%Y-%m-%d %H:%M:%S')
                            logger.info(f'time={_abstime},len={len(targets_list)} targets_new={_targets}')
                    #logger.info(f'targets={targets_list}')
                    #logger.info(f'node{_i+1} targets={_targets}')

                # 执行算法，完成题目要求 todo.
                goals = algo_reader(way_point_queue,way_lock)
                if goals is not None:
                   #logger.info(f'agents={goals}')
                   for goal in goals:
                       _ins = Algo.ins[f'node{goal["id"]}']
                       _id = goal["id"]
                       wayPointList = goal["wayPoints"]
                       _road = []
                       speed=goal["speed"]
                       for point in wayPointList:
                            _road.append(
                                {
                                "shape": "Point",
                                "points": [
                                    {
                                        "coord": point.tolist(),
                                        "spd": speed,
                                        "acc": -1
                                    },
                                         ]
                                 }
                                        )
                       await Algo.send_road(f'node{goal["id"]}', _road)



            except CancelledError:
                logger.info(f'task cancel.')
                break

            except Exception as e:
                logger.exception(e)

    # 下发航行航路指令
    @staticmethod
    async def send_road(node_name, road):
        sess = Algo.session.get(node_name)
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {
                "arguments": {
                    "roadId": int(time.time()),
                    "startMode": "firstPoint",
                    "startPoint": 1,
                    "spdMultiple": 1.0,
                    "road": road
                }
            }
        }
        await sess.publish('/10001001/2/3', e)
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {
                "arguments": {
                    "cmd": 1,
                    "type": 1
                }
            }
        }
        await sess.publish('/10001001/2/1', e)

    # 下发航行速度方向指令
    @staticmethod
    async def send_sail(node_name, speed, angle):
        sess = Algo.session.get(node_name)
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {
                "arguments": {
                    "speed": speed,
                    "angle": angle,
                }
            }
        }
        await sess.publish('/10001001/2/5', e)
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {
                "arguments": {
                    "cmd": 1,
                    "type": 2
                }
            }
        }
        await sess.publish('/10001001/2/1', e)

    # 下发航行停止指令
    @staticmethod
    async def send_stop():
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {
                "arguments": {
                    "cmd": 2,
                    "type": 1
                }
            }
        }
        for _, sess in Algo.session.items():
            await sess.publish('/10001001/2/1', e)

    # 下发完成指令
    @staticmethod
    async def send_finished():
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {
                "arguments": {
                }
            }
        }
        for _, sess in Algo.session.items():
            await sess.publish('/cup/2/4', e)
            break

def getTaskArea(proj):
    taskArea = Algo.init_info['taskArea']
    taskArea = np.array(taskArea)
    taskPosition = lla2ecef(taskArea,proj)
    return taskPosition
    
def getAgentInitInfo(proj):
    rawAgentData = []
    Agent_info = Algo.init_info['vesInfo']
    for _agent in Agent_info:
        rawAgentData.append(_agent['position'])
    rawAgentData = np.array(rawAgentData)
    agentInitPostion = lla2ecef(rawAgentData, proj)
    return agentInitPostion
    
def getForbidArea(proj):
    forbidAreaRaw = []
    forbid_info = Algo.init_info['forbidArea']
    for _forbid in forbid_info:
        forbidAreaRaw.append(_forbid)
    forbidArea = []
    for area in forbidAreaRaw:
        if len(area) == 0:
            continue
        area = lla2ecef(np.array(area), proj)
        forbidArea.append(area)
    return forbidArea

def getAgents(proj):
    agents = []
    Agent_info = Algo.init_info['vesInfo']
    #agentInitPos = getAgentInitInfo()
    for _agent in Agent_info:
        _agent_pos = []
        _agent_pos.append(_agent['position'])
        pos = lla2ecef(np.array(_agent_pos),proj)
        agent = {
            "id": _agent['id'],
            "detectRadius": _agent['detectRadius'],
            "detectAngl": _agent['detectAngle'],
            "position": [pos[0][0], pos[0][1]],
            "sailViteLimit": _agent['sailViteLimit'],
            "threatRadius": _agent['threatRadius'],
            "certainRadiusLimit": _agent['certainRadiusLimit'],
            "velocity": [0, 0],
            "newControl": [0, 0],
            "targets": [],
            "targetsWayPoints": [],
            "wayPoints": [],
            "goals": [],
            "path": [],
            "vmax": _agent['sailViteLimit'],
            "type": 0,
        }
        agents.append(agent)
    return agents

def getInitTaskInfo(proj):
    taskInitInfo = {
        "agents": getAgents(proj),
        "taskArea": getTaskArea(proj),
        "forbidArea": getForbidArea(proj),
        'Finished': False,
    }
    return taskInitInfo

def Transcoordinate(X, Y, devX, devY):
    X = X - devX
    Y = Y - devY
    return X, Y
    

