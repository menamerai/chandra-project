import os
import json
import requests
from smolagents import CodeAgent, Tool, ToolCallingAgent
from smolagents import LiteLLMModel
from dotenv import load_dotenv
from datetime import datetime
from enum import Enum

class CommandType(str, Enum):
    """Enum for command types"""
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    MOVE_FORWARD_LEFT = "move_forward_left"
    MOVE_FORWARD_RIGHT = "move_forward_right"
    MOVE_BACKWARD_LEFT = "move_backward_left"
    MOVE_BACKWARD_RIGHT = "move_backward_right"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    SIT = "sit"
    STAND = "stand"
    ROLL_OVER = "roll_over"
    STOP = "stop"

startTime = datetime.now()

with open("/home/rai/chandra-project/src/data/system_prompt.txt", 'r') as f: 
    SYSTEM_PROMPT = f.read()
with open("/home/rai/chandra-project/src/data/agentic_prompt.txt", "r") as f: 
    AGENTIC_PROMPT = f.read()
with open("/home/rai/chandra-project/src/data/test.json", "r") as f: 
    DATASET = json.load(f)

class Agents: 
    def __init__(self):
        pass
    
    @staticmethod
    def agentic_response(agent, prompt):
        response = agent.run(prompt)
        return response
    
    @staticmethod
    def setup_environment():
        dotenv_path = "/home/rai/chandra-project/.devcontainer/.env"
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


class CommandRobot(Tool):
    name = "command_robot"
    description = (
        "Call the velocity endpoint to move the robot in a specific direction with a specified execution time. "
        "Valid command types: move_forward, move_backward, move_left, move_right, move_forward_left, move_forward_right, "
        "move_backward_left, move_backward_right, rotate_left, rotate_right, sit, stand, roll_over, stop."
    )
    inputs = {
        "movements": {
            "type": "array",
            "description": (
                "List of movement commands, each containing a command type (str) and duration (float). "
                "Format: [['move_forward', 5.0], ['move_left', 3.0], ...]. "
                "Valid command types: move_forward, move_backward, move_left, move_right, move_forward_left, "
                "move_forward_right, move_backward_left, move_backward_right, rotate_left, rotate_right, sit, stand, roll_over, stop."
            )
        }
    }
    output_type = "object"

    def forward(self, movements: list[tuple[str, float]]) -> dict:
        """
        Sends a POST request to the local Command endpoint that controls the robot with a list of movements,
        where each movement is a tuple of (command type, execution_time).

        Args:
            movements: List of [command type, execution_time] pairs.
                command type (str): Must be one of "move_forward", "move_backward", "move_left", "move_right", 
                    "move_forward_left", "move_forward_right", "move_backward_left", "move_backward_right", 
                    "rotate_left", "rotate_right", "sit", "stand", "roll_over", "stop".
                execution_time (float): Time in seconds for the movement. This is optional for the non-movement commands.

        Example:
            movements = [
                ["move_forward", 5.0],
                ["move_left", 3.0],
                ["stop", 1.0]
            ]
            command_robot(movements)
        """
        try:
            formatted_movements = []
            valid_commands = [ct.value for ct in CommandType]
            for cmd, execution_time in movements:
                if not isinstance(cmd, str) or cmd not in valid_commands:
                    raise ValueError(f"Invalid command type: {cmd}. Must be one of {valid_commands}")
                formatted_movements.append([cmd, execution_time])
            
            response = requests.post("http://localhost:8000/robot/commands", json=formatted_movements)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            return {"error": str(e)}


def agentic_process(instruction: str) -> dict:
    process = Agents()
    process.setup_environment()
    tools = [CommandRobot()]
    agent = process.agent(model_id="gemini/gemini-2.0-flash", tools=tools)
    instruction = instruction.strip()
    process.inference(agent, instruction)
    return {"status": "success", "message": "Command executed successfully"}
    
if __name__ == "__main__":  
    instruction = "Move straight for 1/5 of a minutes. Turn left for 10 seconds. Move forward and right for 5 seconds. Stop."
    agentic_process(instruction)