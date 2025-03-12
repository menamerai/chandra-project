import subprocess


def run_scripts():
    server_process = subprocess.Popen(["uvicorn", "src.server:app", "--reload"])
    client_process = subprocess.Popen(["streamlit", "run", "src/client.py"])

    try:
        server_process.wait()
        client_process.wait()
    except KeyboardInterrupt:
        server_process.terminate()
        client_process.terminate()


if __name__ == "__main__":
    run_scripts()
