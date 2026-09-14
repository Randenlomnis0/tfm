#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import subprocess

from ollama import Client

import time

class NaturalLanguageProcessor(Node):

    def __init__(self):

        super().__init__("natural_language_processor")

        self.declare_parameter("input_topic", "/instruction")
        self.declare_parameter("output_topic", "/prompt")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.create_subscription(
            String,
            self.input_topic,
            self.instruction_callback,
            10,
        )

        self.publisher = self.create_publisher(
            String,
            self.output_topic,
            10,
        )

        self.get_logger().info("Natural Language Processor ready.")

    def instruction_callback(self, msg: String):
        instruction = msg.data.strip()

        self.get_logger().info(f"Received new instruction: {instruction}")

        if instruction == "":
            return

        self.process_instruction(instruction)

    def process_instruction(self, instruction):
        orin_ip = "http://192.168.8.190:11434"
        # orin_ip = "http://192.168.0.233:11434"

        client = Client(host=orin_ip)

        inicio = time.perf_counter()

        prompt = f"""
You are an expert semantic parser for a mobile robot performing object-centric navigation.

Your task is to interpret a natural-language instruction from a user and convert it into exactly THREE lines of output:

LINE 1 — NAVIGATION TASK
Return exactly one of:

"goto" — the user wants the robot to navigate to an object once.
"follow" — the user wants the robot to continuously follow an object.
"null" — the user does not request either going to or continuously following an object, or the intended object/task cannot be determined reliably.

LINE 2 — DETAILED OBJECT DESCRIPTION
Return a detailed, semantically rich description of the object that is relevant to the requested navigation task.

The description will be used for visual grounding, so it should preserve all information from the user's instruction that can help identify the intended object. Include relevant visual properties such as category, color, material, size, shape, appearance, distinctive features, or other identifying characteristics when they are stated or clearly implied.

You may infer reasonable missing information from the user's wording and common-sense context. For example, if the user refers to "the chair I mentioned" or "my red backpack," use the information available in the instruction to construct the most specific faithful description possible.

Do NOT invent arbitrary properties that are unsupported by the user's instruction.

Return "null" if the user didn't provide enough information to reference a specific object.

LINE 3 — SIMPLE OBJECT DESCRIPTION
Return a short description suitable for open-vocabulary object detection.

It must:

Be a simple noun or a short noun phrase describing a visually identifiable, non-abstract physical object.
Usually contain only the object category.
Include a visual adjective only when it is necessary to distinguish the intended object and the adjective is visually detectable.
Contain no negations.
Contain no conjunctions such as "and" or "or".
Contain no spatial relationships such as "next to", "behind", "in front of", "near", "on", "under", etc.
Contain no actions or navigation instructions.
Avoid unnecessary adjectives.
Prefer common object-category terminology that a visual detector is likely to recognize.

Examples:
"red backpack" → red backpack
"the chair by the window" → chair
"the large blue suitcase" → blue suitcase
"the person wearing a red shirt" → person
"the cup on the table" → cup

It is OKAY and preferable if the concept is more general, as long as the more general concept is just as identifiable. For example "person" would be better than "woman".

Return "null" if the user didn't provide enough information to reference a specific object.

Output discipline.
Output EXACTLY three lines:

navigation task
detailed object description
simple object description

Do not output labels, explanations, bullet points, quotation marks, JSON, Markdown, or any additional text.

OUTPUT FORMAT:
<goto|follow|null>
<detailed object description|null>
<simple object description|null>

IMPORTANT SEMANTIC RULES

Understand implied intent.
The user may use indirect, colloquial, abbreviated, or context-dependent language. Infer whether they want the robot to go to an object, continuously follow an object, or if they have no preference.

Distinguish "goto" from "follow".
Use "goto" when the robot is expected to navigate to the object as a destination.
Use "follow" when the robot is expected to continuously track and move after the object/person.
Words such as "come to," "go to," "get to," "head to," "approach," or "find" generally imply "goto."
Words such as "follow," "stay with," "keep following," or "follow me" generally imply "follow."
Do not interpret merely observing, identifying, finding information about, or interacting with an object as a navigation request unless navigation is clearly implied.

Resolve references.
Resolve expressions such as "that chair," "the red one," "my bag," or "the person over there" as far as possible from the instruction itself. Use common-sense interpretation when appropriate.

Preserve the user's intended object.
If the user specifies distinguishing properties, preserve them in the detailed description. Do not replace the intended object with a more generic object when the distinguishing information is useful.

Handle people as physical visual targets.
A person can be the navigation object. For example, "follow me" should produce "follow" and a detailed description referring to the user/person, with "person" as the simple description.

Ignore irrelevant information.
Only describe the physical object relevant to the navigation task. Do not include unrelated objects, destinations, or conversational details.

Spatial information.
Spatial relationships may be included in the DETAILED OBJECT DESCRIPTION when they help identify the target object.
Spatial relationships MUST NOT appear in the SIMPLE OBJECT DESCRIPTION.

Multiple objects.
If several objects are mentioned, identify the object that is the actual navigation target. If the target cannot be determined reliably, return "null" for the task and both descriptions.

Abstract or non-visual targets.
Do not produce object descriptions for abstract concepts, locations that are not themselves objects, actions, or properties that cannot serve as a visually grounded object target. Return "null" when an appropriate physical visual target cannot be determined.

Faithfulness takes priority over specificity.
Infer implied information when it is strongly supported by the wording, but never fabricate visual characteristics.

The first line of output can be "null" without the other two lines being "null". This simply means that a specific operating mode is not preferred, but the object should still be identified.

THE INSTRUCTION YOU MUST INTERPRET:
    {instruction}
    """

        self.get_logger().info("Sending request to Orin")

        models = [
            "qwen3.8:27b",
            "qwen3.6:35b",
            "qwen3.6:27b",
            "qwen3.5:9b",
            "gemma4:31b-it-q8_0",
            "gemma4:31b",
            "qwen3-vl:4b-instruct",
            "minicpm-v4.6",
            "minicpm-v4.5:8b"
        ]

        response = client.generate(
            model=models[0],
            prompt=prompt,
            keep_alive="1h",
            think='false',
            options={
                "temperature": 0.0,
                "num_predict": 150,
                "num_ctx": 1024,
                "top_k": 1
            }
        )

        fin = time.perf_counter()
        tiempo_total = fin - inicio

        self.get_logger().info(f"Total response time: {tiempo_total:.2f}s")

        raw_output = response['response']

        self.get_logger().info(f"Output:\n")
        self.get_logger().info(f"{raw_output}")

        mode, detailed_prompt, simple_prompt = raw_output.splitlines()

        if mode == "goto":
            self.get_logger().info(f"goto mode was specified")

            result = subprocess.run([
                'ros2',
                'param',
                'set',
                '/prompt_follower',
                'mode',
                'goto'
            ])

            if result.returncode != 0:
                self.get_logger().warn(f'Something went wrong when changing modes')
        elif mode == "follow":
            self.get_logger().info(f"follow mode was specified")

            result = subprocess.run([
                'ros2',
                'param',
                'set',
                '/prompt_follower',
                'mode',
                'follow'
            ])

            if result.returncode != 0:
                self.get_logger().warn(f'Something went wrong when changing modes')
        else:
            self.get_logger().info(f"no mode was specified")

        self.get_logger().info(f"generated detailed prompt: '{detailed_prompt}'")
        self.get_logger().info(f"generated simple prompt: '{simple_prompt}'")

        msg = String()
        msg.data = detailed_prompt + ":" + simple_prompt

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = NaturalLanguageProcessor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
