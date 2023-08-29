import os
import json

to_mains_pipe_path = 'to_mains_pipe'
from_mains_pipe_path = 'from_mains_pipe'

# Create named pipes
def create_named_pipe(pipe_path):
    try:
        os.mkfifo(pipe_path)
    except FileExistsError:
        os.remove(pipe_path)
        os.mkfifo(pipe_path)

create_named_pipe(to_mains_pipe_path)
create_named_pipe(from_mains_pipe_path)

def send_data(data, pipe_path):
    with open(pipe_path, 'w') as pipe:
        pipe.write(json.dumps(data))

def receive_data(pipe_path):
    with open(pipe_path, 'r') as pipe:
        return json.loads(pipe.readline())

if __name__ == '__main__':
    # Start mains.py
    os.system('python3 mains.py &')

    # Loop for sending and receiving data
    for i in range(50):
        # Send data to mains.py
        data_to_send = {'key': i}
        send_data(data_to_send, to_mains_pipe_path)

        # Wait for a response
        received_data = receive_data(from_mains_pipe_path)
        print(f"algo.py received data: {received_data}")

    # Remove named pipes
    os.remove(to_mains_pipe_path)
    os.remove(from_mains_pipe_path)

