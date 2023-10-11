import asyncio
import logging
import time
from asyncio import CancelledError

formatter = logging.Formatter(
    "%(asctime)s - %(name)s - %(filename)s:%(lineno)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("algo")
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
        _first_start = True

        ## socket call
        import socket
        import json
        import pickle

        # Create a socket object
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the server (Program B)
        server_address = ("localhost", 12345)
        sock.connect(server_address)

        # Send data to Program B

        while True:
            try:
                await asyncio.sleep(0.1)

                # 未开始，等待开始指令
                if not Algo.start_cmd:
                    logger.info("Game not started.")
                    continue

                # 收到停止指令
                if Algo.stop_cmd:
                    await Algo.send_stop()
                    logger.info("Game stopped.")
                    continue

                # 第一次收到开始指令，根据题目信息规划航路，并下发航行指令
                # 当前策略为从当前位置驶向区域的8个点，这不是一个好的策略，仅用于示例
                if _first_start:
                    _first_start = False

                    data_to_send = {"key": "value", "num": 42, "list": [1, 2, 3]}
                    pickled_data = pickle.dumps(data_to_send)

                    sock.sendall(pickled_data)
                    sock.close()

                    for _i in range(8):
                        _ins = Algo.ins[f"node{_i+1}"]
                        _road = [
                            {
                                "shape": "LineString",
                                "points": [
                                    {
                                        "coord": [_ins["lon"], _ins["lat"]],
                                        "spd": 40.0,
                                        "acc": -1,
                                    },
                                    {
                                        "coord": Algo.init_info["taskArea"][_i],
                                        "spd": 40.0,
                                        "acc": -1,
                                    },
                                ],
                            },
                        ]
                        await Algo.send_road(f"node{_i+1}", _road)
                        logger.info(f"send road {_road}")

                    pass

                # 打印各智能体发现的目标
                for _i in range(8):
                    _targets = Algo.sensor[f"node{_i+1}"]
                    # logger.info(f'node{_i+1} targets={_targets}')

                # 执行算法，完成题目要求 todo.

                pass

            except CancelledError:
                logger.info(f"task cancel.")
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
                    "road": road,
                }
            },
        }
        await sess.publish("/10001001/2/3", e)
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {"arguments": {"cmd": 1, "type": 1}},
        }
        await sess.publish("/10001001/2/1", e)

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
            },
        }
        await sess.publish("/10001001/2/5", e)
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {"arguments": {"cmd": 1, "type": 2}},
        }
        await sess.publish("/10001001/2/1", e)

    # 下发航行停止指令
    @staticmethod
    async def send_stop():
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {"arguments": {"cmd": 2, "type": 1}},
        }
        for _, sess in Algo.session.items():
            await sess.publish("/10001001/2/1", e)

    # 下发完成指令
    @staticmethod
    async def send_finished():
        e = {
            "id": int(time.time()),
            "method": "notice-event",
            "content": {"arguments": {}},
        }
        for _, sess in Algo.session.items():
            await sess.publish("/cup/2/4", e)
            break
