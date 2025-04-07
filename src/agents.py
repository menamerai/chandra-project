import threading
import time
import os
import json
import requests
from smolagents import CodeAgent, CodeAgent, tool, ToolCallingAgent, Tool
from smolagents import LiteLLMModel
from dotenv import load_dotenv
from datetime import datetime
from brigde import Direction

startTime = datetime.now()

with open("/chandra-project/src/data/system_prompt.txt", 'r') as f: SYSTEM_PROMPT = f.read()
with open("/chandra-project/src/data/agentic_prompt.txt", "r") as f: AGENTIC_PROMPT = f.read()
with open("/chandra-project/src/data/test.json", "r") as f: DATASET = json.load(f)

class Agents: 
    def __init__(self):
        pass
    
    @staticmethod
    def agentic_response(agent, prompt): 
        response = agent.run(prompt)
        return response
    
    @staticmethod
    def setup_environment(): 
        dotenv_path = "/chandra-project/.devcontainer/.env"
        load_dotenv(dotenv_path=dotenv_path)
        os.environ['GEMINI_API_KEY'] = os.getenv("GEMINI_API_KEY")
    
    @staticmethod
    def get_response_no_agent(instruction, model_id="gemini/gemini-2.0-flash") -> str:
        prompt = f"{SYSTEM_PROMPT}\nInput: {instruction}\nOutput:"
        messages = [
            {"role": "user", "content": [{"type": "text", "text": f"{prompt}"}]}
        ]
        model = LiteLLMModel(model_id=model_id)
        response = model(messages).content
        return response

    def agent(self, model_id="gemini/gemini-2.0-flash", tools=[]):
        model = LiteLLMModel(model_id=model_id)
        agent = CodeAgent(
            tools=tools,
            model=model,
            add_base_tools=False,
            max_steps=15
        )
        return agent
    
    def inference(self, agent, instruction, verbose=True):
        prompt = f"{AGENTIC_PROMPT}\nInput: {instruction}\nOutput:"
        response = agent.run(prompt)
        if verbose:
            return response

# @tool
# def call_velocity(direction: str, execution_time: float) -> dict:
#     """
#     This function sends a POST request to the local velocity endpoint with the agent's response, 
#     which is the direction and execution time for the robot to move.

#     Args:
#         direction (str): The direction to move the robot. 
#             Must be one of the following commands: "forward", "backward", "left", "right", 
#             "forward_left", "forward_right", "backward_left", "backward_right", "stop".

#         execution_time (float): The time in seconds for which the robot should move in the specified direction. Must be a positive number.

#     Returns:
#         A dict indicating the success or failure of the request.

#     Example:
#         call_velocity("forward", 5.0)
#     """
#     try:
#         # if direction is string, convert it to Direction enum
#         if isinstance(direction, str):
#             direction = Direction[direction.upper()]
        
#         response = requests.post("http://localhost:8000/robot/velocity", json={
#             "direction": direction.value,
#             "execution_time": execution_time
#         })
#         response.raise_for_status()
#         return response.json()
#     except requests.RequestException as e:
#         return {"error": str(e)}

class CallVelocity(Tool):
    name = "call_velocity"
    description = "Call the velocity endpoint to move the robot in a specific direction with a specified execution time."
    inputs = {
        "movements": {
            "type": "array",
            "description": "List of movement commands, each containing a direction (str) and duration (float). Format: [['forward', 5.0], ['left', 3.0], ...]"
        }
    }
    output_type = "object"

    def forward(self, movements: list[tuple[str, float]]) -> dict:
        """
        This function sends a POST request to the local Velocity endpoint with a list of movements,
        where each movement is a tuple of (direction, execution_time).
        
        Args:
            movements: List of [direction, execution_time] pairs.
                direction (str): One of "forward", "backward", "left", "right", 
                    "forward_left", "forward_right", "backward_left", "backward_right", "stop".
                execution_time (float): Time in seconds for the movement.

        Example:
            movements = [
                ["forward", 5.0],
                ["left", 3.0],
                ["stop", 1.0]
            ]
            call_velocity(movements)
        """
        try:
            # Convert string directions to Direction enum values
            formatted_movements = []
            for direction, execution_time in movements:
                if isinstance(direction, str):
                    direction = Direction[direction.upper()]
                formatted_movements.append([direction.value, execution_time])
            
            response = requests.post("http://localhost:8000/robot/velocity", 
                                    json=formatted_movements)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            return {"error": str(e)}


def agentic_process(instruction: str) -> dict: 
    process = Agents()
    process.setup_environment()
    tools = [CallVelocity()]
    # tools = [call_velocity]
    agent = process.agent(model_id="gemini/gemini-2.0-flash", tools=tools)
    instruction = instruction.strip()
    process.inference(agent, instruction)
    return {"status": "success", "message": "Command executed successfully"}
    
if __name__ == "__main__":  
    instruction = "Move straight for 1/5 of a minutes. Turn left for 10 seconds. Move forward and right for 5 seconds. Stop."
    agentic_process(instruction)