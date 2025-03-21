import re

from word2number import w2n

# Define the directions constant
NAME = "brutus"
FORWARD = "forward"
BACKWARD = "backward"
LEFT = "left"
RIGHT = "right"


# TODO: Use LLM to do this so that intent is extracted correctly
def extract_parameter(text):
    match = re.search(r"\d+", text)
    if match:
        return int(match.group()) if match else None

    # If no digit found, look for word representations of numbers
    words = text.split()
    for word in words:
        try:
            return w2n.word_to_num(word)
        except ValueError:
            continue

    return None


def command_parsing(text):
    text = text.lower()
    parameter = extract_parameter(text)

    if NAME not in text or parameter is None:
        return None

    if FORWARD in text:
        return {"action": FORWARD, "parameter": parameter}

    elif BACKWARD in text:
        return {"action": BACKWARD, "parameter": parameter}

    elif LEFT in text:
        return {"action": LEFT, "parameter": parameter}

    elif RIGHT in text:
        return {"action": RIGHT, "parameter": parameter}

    else:
        return None


if __name__ == "__main__":
    text = "Brutus, move forward for 20 seconds"
    print(command_parsing(text))
