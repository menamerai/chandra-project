# Project Description
C.H.A.N.D.R.A (Companion Helper for Assistive Navigation and Daily Robotic Assistance) is a robotic dog designed to assist individuals with mobility challenges through natural language voice commands. It integrates agentic AI, real-time speech recognition, and LLMs to provide hands-free support for daily tasks and navigation.

In our final design, C.H.A.N.D.R.A uses an agentic AI workflow to convert spoken commands into structured robot actions via a multi-agent reasoning pipeline. Speech input is processed through Whisper for transcription and passed to an LLM for intent parsing and decision-making. The resulting structured command is then routed through a server to control the robot interface. For safety reasons, our final tests were conducted in simulation on a computer rather than on a physical robot. This allowed us to verify the end-to-end functionality of the system—including command interpretation, server communication, and simulated robotic control—without risking unintended behavior during live trials.

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

### Cat Luong


From my academic perspective, I view this senior design project as a fantastic opportunity to put all the knowledge I've accumulated at UC over the past five years into practice. It’s a chance to integrate everything I've learned, from web development to advanced Artificial Intelligence concepts. I’m also keen to grasp the practical aspects of managing a large project, including organization and execution. I want to explore what makes a product truly successful—how it not only functions technically but also meets user needs and provides real value. This project will be a crucial learning experience, presenting challenges that will foster my growth both technically and professionally. For this project, we're considering incorporating AI, whether through a "GPT-Wrapper" style approach or more theoretical work in Natural Language Processing or Reinforcement Learning. If we go with a "GPT-Wrapper", we aim to create something innovative and impactful, rather than just making a few API calls.

In the past four years, I've taken several courses that have profoundly shaped my skills and knowledge. In **Intelligent Systems**, I gained deep insights into neural networks and deep learning techniques. **AI Principles** broadened my understanding of various AI algorithms and their applications, while **Natural Language Processing** equipped me with essential techniques for analyzing and generating human language as well as allow myself to be passionate in a different field than what I had known before. Additionally, **Software Engineering and Database Design** taught me how to build and maintain scalable, deployable systems, emphasizing best practices and project management. These experiences have prepared me to tackle complex projects, integrate advanced AI concepts, and develop solutions that are both technically sound and impactful, and will be instrumental for my Senior Design project. 

As a Machine Learning Intern at Kinetic Vision, I worked on integrating Nvidia’s NeMo Automatic Speech Recognition into a speaker diarization pipeline and developed a Streamlit application for video transcription using OpenAI’s Whisper. I also applied Longformer and Bart models for recursive summarization. I designed a robust training pipeline for the YOLOv8 model to detect and classify pharmaceutical products. At FPT Software, I built an OCR model using the PaddleOCR framework to extract information from multilingual invoices and automated data labeling processes. As an Undergraduate Researcher, I standardized datasets, streamlined dataset mapping, and categorized application essays using various machine learning models, gaining extensive experience in data processing and model evaluation. These experiences are so incredibly important during my journey at UC. 

I have participated in several hackathons, focusing predominantly on AI-driven applications, and have achieved success in a few, namely RevolutionUC. My enthusiasm for AI is fueled by its capacity to address complex real-world challenges that conventional software engineering struggles with. What captivates me about AI is its power to deliver intelligence as needed—empowering individuals with tools and knowledge that were once limited to experts. This capability to distribute intelligence widely aligns with my belief in its critical role in societal advancement. I am eager to contribute to this field because I see it as essential for driving progress and fostering innovation. I believe that I will be able to carry this mentality to the project. 

I anticipate that this project will be highly impactful, and I plan to contribute significantly through both theoretical research and software engineering. My goal is to ensure that the outcomes of this project are widely distributed and accessible to a broad audience. I am committed to dedicating a substantial amount of time and effort to this endeavor, with the hope of deepening my knowledge and making a meaningful contribution. I believe that my involvement will help advance the project’s goals and enhance its potential for success. I'm looking to create something impactful like a new deep learning library or likewise. 

## Final Self-Assessments

### Phan Anh Duong

My main contribution to this project is DevOps, UI/UX, Program Flow and Robotics. Specifically, I designed the installation scripts and processes to the project. I also designed the UI and implemented the frontend website for the demo. I also came up with the general program flow of our system. I also learned ROS 2 and Mavlink and developed our interaction interface with our robotic simulations.

For this project, I utilized my existing proficiency with web development and DevOps. I did not used much of my AI expertise here, as I offloaded that responsibility to my partner. I, however, learned a lot of robotics, especially simulation and control methodologies and systems. It was very difficult to learn robotics, as there is a notable lack of documentation and standardization across different robots.

My group made an end-to-end system that dictate arbitrary voice control of a robotic system. We also added a novel component of agentic decision-making to map arbitrary natural language commands to discrete execution, maintaining a delicate balance between safety and flexibility in robotic control.

During this process, I learned how to handle resource constrainst between different team members. Notably, I have access to robots due to being part of the Applied Autonomy lab, and my teammate does not have the same level of access. Naturally, I have to take up robotics development. But, I also learn how to "pretend" a control interface already exists so that my partner can do his work without waiting for me to finish mine. Of course, there are cases where this is simply not possible, so in that case, I unfortunately have to take on the development cycle myself.

### Cat Luong

My main contribution to this project was ideation, testing, and deployment of an agentic AI workflow that interfaced seamlessly with both our robot control system and backend server. I led the design of our multi-agent reasoning pipeline, which interprets voice-based natural language instructions into structured, executable robot commands. I also implemented robust testing and refinement loops to ensure the system remained generalizable while maintaining control safety and accuracy across diverse command types.

Throughout this project, I applied my knowledge in AI system design, language model prompting, and backend integration. I worked closely with both the server interface and the robotic control logic to ensure consistent behavior and graceful failure handling in real-time interactions.

From my teammate’s perspective, the project posed significant challenges, especially in terms of coordinating development work across vastly different hardware and access limitations. Unlike me, he had limited access to the physical robots and had to simulate or abstract parts of the system during development. He often had to assume the control interfaces I was building already existed so that he could continue working in parallel. In many cases, that meant working with mocked APIs or fabricated command outputs, which introduced extra complexity and occasional delays. In some cases, when integration was impossible to fake, he had to wait on my updates or even take on parts of the robotics development process himself to ensure end-to-end functionality. Despite these obstacles, we both adapted, enabling our system to evolve iteratively and collaboratively.

Together, we developed an end-to-end pipeline capable of interpreting arbitrary spoken commands and executing them through a robotic system using multi-step agentic reasoning. Our final system balanced interpretability, flexibility, and safety—core to building trustworthy robotic agents.

# Summary of Hours and Justification

## Phan Anh Duong
  - Testing: ~10 hours
  - Developing: ~10 hours
  - Documenting: ~10 hours
  - Learning: ~20 hours (has to learn ROS which takes more time)
  - Ideating: ~10 hours

## Cat Luong
  - Testing: ~10 hours
  - Developing: ~10 hours
  - Documenting: ~10 hours
  - Learning: ~10 hours
  - Ideating: ~10 hours

# Summary of Expenses

All the stuff in this project is free since they are all open-source software, but some of the hardware that we utilized are:
- Jetson Orin Nano + Supplemental Components x 2: 640 x 2 = $1280
- V60 robotic dog: ~$150,000 (borrowed for testing purposes)
  
# Appendix

- The GitHub repository commit history and the deliverables submitted throughout the project should justify 45+ hours of commitment per team member. 
- References:
  - Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074
  - Radford, A., Kim, J. W., Xu, T., Brockman, G., McLeavey, C., & Sutskever, I. (2023). Robust Speech Recognition via Large-Scale Weak Supervision. In International Conference on Machine Learning (pp. 28492–28518). PMLR.
