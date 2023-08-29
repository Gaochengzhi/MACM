import os
import time
import json

to_mains_pipe_path = 'to_mains_pipe'
from_mains_pipe_path = 'from_mains_pipe'

def send_data(data, pipe_path):
    with open(pipe_path, 'w') as pipe:
        pipe.write(json.dumps(data))

def receive_data(pipe_path):
    with open(pipe_path, 'r') as pipe:
        return json.loads(pipe.readline())

if __name__ == '__main__':
    # Loop for receiving and sending data
    for i in range(50):
        # Wait for data from algo.py
        received_data = receive_data(to_mains_pipe_path)
        print(f"mains.py received data: {received_data}")
        time.sleep(0.3)


        # Process data and send a response
        data_to_send = {'response': f"Data received: {received_data}"}
        send_data(data_to_send, from_mains_pipe_path)

