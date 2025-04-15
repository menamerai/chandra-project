# Project Description

# User Interface Specification

We have plans for a **practical** interface and a **demo** interface. These differences arises mainly due to the fact that it was logisitically to procure and transport the main robot that we tested on for this project, the Ghost Robotics' V60.

## Practical Interface

Practically speaking, our main design would be installed on an edge computing device with GPU access, and there would be no visual user interface. Instead, the user would interact with the system via a voice interface similar to an Amazon Echo, where the pipeline would continously listen, and trigger upon some key word.

## Demo Interface

For our demo, we decided on another direction for user interface. We had to use a robot simulation instead of the actual robot, which we need to display on our laptop. With this setup, we figured that a visual user interface was possible, and helpful for users. Specifically, we wanted the users to be able to easily record voices, and to manually perform a text input in the case that the environment is too noisy for the system to work properly.

Addressing these needs, we decided on a tried-and-true method: a website that have a visual form that the user can interact with in order to access our systems. The website will have a recording button, a text box that serve as both the indicator to the voice transcriber's result, as well as a manual text input to the system. The website will also have a submit button for manual submission and re-submission by the user.

For our technical stack, we ended up using a JavaScript framework called ReactJS in conjuction with ViteJS for fast compilation time. We also used pre-built styled UI components by shadcn.

# Test Plan & Results

As there are no established plans to test our system, we decided to synthetically generate sets of inputs and desired outputs for our system using GPT-4o. Specifically, we prompted the model to come up with a list of adversarial examples that could mislead our system. It focuses on long, confusing chains of commands, as well as fringe rephrasing of command terms like "walk forward." To make sure that the data is quality, we manually vetted the data, and got rid of cases that were also misleading to humans. In the end, we have ~50 input-output pairs for our system.

Because the inputs are arbitrary, but the output are discrete and known, we can treat this as a sequential classification problem, where any input is mapped to a sequence of commands, also arbitrary in length. We can then programmatically pass these cases into the system, gather their outputs, then systematically compare them with the gold standard.

# User Manual

https://github.com/menamerai/chandra-project/blob/v60/Documents/User-Manual.md

# Spring Final PPT Presentation

https://github.com/menamerai/chandra-project/blob/v60/Documents/Chandra_Presentation.pdf

# Final Expo Poster

https://github.com/menamerai/chandra-project/blob/v60/Documents/CHANDRA_Poster.pdf

# Assessment

## Initial Self-Assessments

### Phan Anh Duong

For our senior design project, my teammate and I wanted to work on something that excites us: robotics and natural language processing (NLP). With our shared interests and complementary skillsets, we aim to tackle a real-world problem by designing an attachment module for a robot similar to Boston Dynamics’ Spot. This module will assist visually impaired individuals by enabling the robot to understand voice commands and announce detected objects via a camera module.

The decision to focus on this project aligns with our enthusiasm for solving practical challenges using advanced technologies. While I’ve participated in hackathons and worked on AI-powered applications, this project feels unique because it combines natural language processing, computer vision, and robotics into a single, modular solution. The attachment’s modularity is key—it allows it to be used with robots beyond the one we’re developing on, as long as they have similar capabilities. This approach ensures the work we’re doing has broader applications and impact.

I don’t see much direct overlap between my coursework and this project, as I’m taking Senior Design a year early, but CS 5134: Natural Language Processing has proven highly relevant. Taught by Professor Tianyu Jiang, the course explored NLP’s evolution from early techniques like n-grams to modern transformer-based models. This deep dive into NLP inspired me to think critically about how to build robust systems that go beyond simple implementations, like using pre-built prompts. My teammate has also taken this course, which will help us collaboratively apply what we’ve learned.

My co-op experiences have also prepared me well for this project. During my first co-op with Dr. Atluri and Cincinnati Children’s Hospital, I gained significant experience in data wrangling, visualization, and machine learning, which sharpened my problem-solving skills in AI. These skills will be invaluable for tasks like processing visual data from the camera module or training models for object detection. My second co-op provided further exposure to managing multiple projects and improving time management and collaboration skills. These soft skills will help keep our project organized and on track. My third co-op focused on web technologies and APIs, which, while not directly related, gave me experience working with backend systems that could prove useful if we need to design flexible data pipelines or modular interfaces.

Hackathons have been another significant source of inspiration. I’ve often worked on AI-powered applications, which taught me how AI can solve problems that traditional software engineering cannot. For example, AI can recognize objects, interpret natural language, or analyze sentiments—tasks impossible without advanced models. This fascination with AI stems from its ability to democratize intelligence and make specialized knowledge accessible to anyone. For this project, I hope to contribute to this democratization by enabling a robot to perform tasks that are otherwise challenging for visually impaired individuals, such as detecting obstacles or identifying objects in their environment.

Our project plan reflects our interdisciplinary goals. Initially, we’ll focus on designing the hardware for the attachment module. Using an NVIDIA Jetson, audio module, camera module, and other equipment, we’ll ensure the attachment is modular and not dependent on the robot’s native sensors (except for universal components like cameras or speakers). From a software perspective, we’ll use ROS 2, which is already installed on the robot, to integrate our systems. The first functionality we’ll implement is voice command processing, leveraging NLP techniques to ensure accurate and flexible command interpretation. Next, we’ll develop an object detection system to identify and announce nearby items via the camera and speaker.

Testing will occur in phases. During the winter break, we’ll focus on hardware and module development, using mock environments to test basic functionalities. Once we regain access to the robot, we’ll conduct real-life field tests to validate the system’s performance in assisting visually impaired individuals. Our ultimate goal is to create an attachment module that works seamlessly and can be deployed on similar robots in various settings. Success will be measured by how well the module performs under real-world conditions and by feedback from testers, particularly those who are visually impaired.

This project represents a significant challenge, but it’s one I’m deeply passionate about. It allows me to combine my skills in natural language processing, computer vision, and engineering to create a solution with tangible impact. By the end, I hope we’ll have a modular system that not only meets the needs of our target users but also inspires further innovation in this space.

## Final Self-Assessments

### Phan Anh Duong

My main contribution to this project is DevOps, UI/UX, Program Flow and Robotics. Specifically, I designed the installation scripts and processes to the project. I also designed the UI and implemented the frontend website for the demo. I also came up with the general program flow of our system. I also learned ROS 2 and Mavlink and developed our interaction interface with our robotic simulations.

For this project, I utilized my existing proficiency with web development and DevOps. I did not used much of my AI expertise here, as I offloaded that responsibility to my partner. I, however, learned a lot of robotics, especially simulation and control methodologies and systems. It was very difficult to learn robotics, as there is a notable lack of documentation and standardization across different robots.

My group made an end-to-end system that dictate arbitrary voice control of a robotic system. We also added a novel component of agentic decision-making to map arbitrary natural language commands to discrete execution, maintaining a delicate balance between safety and flexibility in robotic control.

During this process, I learned how to handle resource constrainst between different team members. Notably, I have access to robots due to being part of the Applied Autonomy lab, and my teammate does not have the same level of access. Naturally, I have to take up robotics development. But, I also learn how to "pretend" a control interface already exists so that my partner can do his work without waiting for me to finish mine. Of course, there are cases where this is simply not possible, so in that case, I unfortunately have to take on the development cycle myself.

# Summary of Hours and Justification

## Phan Anh Duong

## Cat Luong

# Summary of Expenses

# Appendix
