import threading
import subprocess
import time
import socket

# Function to check if the server is listening on the given port
def is_server_ready(host='localhost', port=8000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((host, port))
        sock.close()
        return True
    except socket.error:
        return False

def run_server():
    subprocess.Popen(['python', 'src/server.py'])

def run_client():
    subprocess.Popen(['streamlit', 'run', 'src/client.py'])

# Create and start server thread
server_thread = threading.Thread(target=run_server)
server_thread.start()

# Wait until the server is ready (listening on the specified port)
while not is_server_ready():
    time.sleep(1)  # Wait for 1 second before checking again

# Once the server is ready, start the Streamlit client
client_thread = threading.Thread(target=run_client)
client_thread.start()

# Wait for both threads to complete
server_thread.join()
client_thread.join()
