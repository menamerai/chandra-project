import threading
import time
import os
import json
import requests
from smolagents import CodeAgent, CodeAgent, tool
from smolagents import LiteLLMModel
from dotenv import load_dotenv
from datetime import datetime

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
            max_steps=10
        )
        return agent
    
    def inference(self, agent, instruction, verbose=True):
        prompt = f"{AGENTIC_PROMPT}\nInput: {instruction}\nOutput:"
        response = agent.run(prompt)
        if verbose:
            return response
@tool
def call_velocity(agent_response: dict) -> dict:
    """
    This function sends a POST request to the local Velocity endpoint with the agent's response, 
    which is a dictionary containing the keys "direction" and "execution_time". It returns the server's 
    JSON response, which is a dictionary indicating whether the request was successful or not.

    Args:
        agent_response: A dictionary with "direction" and "execution_time" keys representing the agent's response.

    Returns:
        A dictionary indicating the success or failure of the request.

    Parameters type:
        agent_response: dict

    Return type:
        dict
    """
    response = requests.post("http://localhost:8000/velocity", json=agent_response)
    return response.json()

@tool
def agent_response(text_response: dict) -> dict: 
    """
    This function sends a POST request to the local agent_response endpoint with a text response, 
    which is a dictionary containing a "text" key representing robotic commands. It returns a 
    structured JSON response containing an "direction" and a "execution_time" key.

    Args:
        text_response: A dictionary with a single key "text" containing robotic command instructions.

    Returns:
        A dictionary with keys "direction" and "execution_time", representing the structured interpretation 
        of the input text.

    Parameters type:
        text_response: dict

    Return type:
        dict
    """
    response = requests.post("http://localhost:8000/agent_response", json=text_response)
    return response[0].json()


def main(): 
    process = Agents()
    process.setup_environment()
    tools = [call_velocity, agent_response]
    agent = process.agent(model_id="gemini/gemini-2.0-flash", tools=tools)
    instruction = "Brutus, go straight for 5 minutes"
    process.inference(agent, instruction, verbose=False)
    # correct = 0
    # for instruction, answer in DATASET.items():
    #     # response = Agents.get_response_no_agent(instruction=instruction, model_id="gemini/gemini-2.0-flash-lite")
    #     # agent = process.agent(model_id="gemini/gemini-2.0-flash-lite")
    #     # response = process.inference(agent, instruction=instruction, verbose=True)
    #     #NOTE: Only for non_agent
    #     response = response.strip().replace("```json", "").replace("```", "").strip()
    #     response = json.loads(response)
    #     responses.append(response)
    #     if response == answer: 
    #         correct += 1
    #     time.sleep(3.0)
    # print("Model Responses: ")
    # print(responses)
    # print(f"Model Accuracy: {correct * 100 / len(DATASET)}%")
    # print(f"Time elapsed {datetime.now() - startTime} seconds")
    
if __name__ == "__main__":  
    main()
